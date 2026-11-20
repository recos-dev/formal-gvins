/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/dataset/euroc.hpp>
#include <formal_gvins/vision/tracker.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
using Clock = std::chrono::steady_clock;
using formal_gvins::dataset::EurocSequence;
using formal_gvins::vision::Execution;
using formal_gvins::vision::OrbTracker;
using formal_gvins::vision::TrackedImage;
using formal_gvins::vision::TrackingOptions;
using formal_gvins::vision::VisionStatus;

struct Profile
{
    std::size_t first = 0;
    std::size_t count = 0;
    int seed = 0;
    int dx = 0;
    int dy = 0;
    TrackingOptions tracking;
};

int integer(cv::FileNode const & node, char const * name)
{
    if (!node[name].isInt())
    {
        throw std::runtime_error(std::string("Expected integer profile field: ") + name);
    }
    return static_cast<int>(node[name]);
}

double real(cv::FileNode const & node, char const * name)
{
    auto const value = node[name];
    if ((!value.isInt() && !value.isReal()) || !std::isfinite(static_cast<double>(value)))
    {
        throw std::runtime_error(std::string("Expected finite profile field: ") + name);
    }
    return static_cast<double>(value);
}

Profile read_profile(char const * path, Execution execution)
{
    cv::FileStorage file(path, cv::FileStorage::READ);
    if (!file.isOpened())
    {
        throw std::runtime_error("Cannot read benchmark profile");
    }
    auto const node = file.root();
    int const first = integer(node, "first_frame"), count = integer(node, "frame_count");
    if (static_cast<std::string>(node["protocol"]) != "euroc-orb-v2" || integer(node, "camera") != 0 || first < 0 ||
        count < 2 || first > std::numeric_limits<int>::max() - count)
    {
        throw std::runtime_error("Unsupported benchmark profile");
    }
    Profile result;
    result.first = static_cast<std::size_t>(first);
    result.count = static_cast<std::size_t>(count);
    result.seed = integer(node, "rng_seed");
    result.dx = integer(node, "translation_x_px");
    result.dy = integer(node, "translation_y_px");
    auto const tracking = node["tracking"];
    auto & options = result.tracking;
    options.orb.execution = execution;
    options.orb.feature_count = integer(tracking, "feature_count");
    options.orb.scale_factor = static_cast<float>(real(tracking, "scale_factor"));
    options.orb.pyramid_levels = integer(tracking, "pyramid_levels");
    options.orb.fast_threshold = integer(tracking, "fast_threshold");
    options.feature_capacity = integer(tracking, "feature_capacity");
    options.grid_columns = integer(tracking, "grid_columns");
    options.grid_rows = integer(tracking, "grid_rows");
    options.ratio = static_cast<float>(real(tracking, "ratio"));
    options.epipolar_threshold_px = real(tracking, "epipolar_threshold_px");
    options.confidence = real(tracking, "confidence");
    options.ransac_iterations = integer(tracking, "ransac_iterations");
    return result;
}

double milliseconds(Clock::time_point start, Clock::time_point finish)
{
    return std::chrono::duration<double, std::milli>(finish - start).count();
}

void process(OrbTracker & tracker, cv::Mat const & image, cv::Mat const & mask, std::uint64_t image_id,
             std::uint64_t time_ns, TrackedImage & output)
{
    formal_gvins::vision::ImageIdentity const identity{1, 0, image_id, time_ns, 1, 1};
    VisionStatus const status = tracker.process(identity, image, mask, output);
    if (status != VisionStatus::success)
    {
        throw std::runtime_error("ORB process failed, status=" + std::to_string(static_cast<int>(status)));
    }
}

void snapshot(std::filesystem::path const & path, cv::Mat const & previous, cv::Mat const & current,
              TrackedImage const & before, TrackedImage const & after)
{
    cv::Mat image;
    cv::drawMatches(previous, before.features.keypoints, current, after.features.keypoints, after.matches.inliers,
                    image, cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), {},
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    if (!cv::imwrite(path.string(), image))
    {
        throw std::runtime_error("Cannot write " + path.string());
    }
}

void write_options(cv::FileStorage & output, TrackingOptions const & options)
{
    output << "options"
           << "{"
           << "feature_count" << options.orb.feature_count << "scale_factor" << options.orb.scale_factor
           << "pyramid_levels" << options.orb.pyramid_levels << "fast_threshold" << options.orb.fast_threshold
           << "feature_capacity" << options.feature_capacity << "grid_columns" << options.grid_columns << "grid_rows"
           << options.grid_rows << "ratio" << options.ratio << "epipolar_threshold_px" << options.epipolar_threshold_px
           << "confidence" << options.confidence << "ransac_iterations" << options.ransac_iterations << "}";
    // These are fixed properties of the production adapter, not benchmark switches.
    output << "adapter"
           << "{"
           << "wta_k" << 2 << "patch_size" << 31 << "edge_threshold" << 31 << "score_type"
           << "HARRIS"
           << "descriptor_smoothing" << (options.orb.execution == Execution::cpu ? "enabled" : "disabled") << "}";
}

void write_failure(cv::FileStorage & output, std::size_t index, std::uint64_t time_ns, char const * error)
{
    output << "{"
           << "frame_index" << static_cast<int>(index) << "time_ns" << std::to_string(time_ns) << "status"
           << "failed"
           << "error" << error << "}";
}

void write_temporal(cv::FileStorage & output, TrackedImage const & tracked, double read_ms, double frontend_ms)
{
    output << "{"
           << "frame_index" << static_cast<int>(tracked.identity.image_id - 1) << "time_ns"
           << std::to_string(tracked.identity.time_ns) << "status"
           << "success"
           << "features" << static_cast<int>(tracked.features.keypoints.size()) << "ratio_passed"
           << static_cast<int>(tracked.matches.ratio_passed) << "inliers"
           << static_cast<int>(tracked.matches.inliers.size()) << "geometry_valid"
           << static_cast<int>(tracked.matches.geometry_valid) << "read_remap_ms" << read_ms << "frontend_ms"
           << frontend_ms << "observations"
           << "[";
    for (auto const & observation : tracked.observations)
    {
        output << "[:" << observation.u << observation.v << std::to_string(observation.track_id) << "]";
    }
    output << "]"
           << "}";
}

void write_points(cv::FileStorage & output, char const * name, TrackedImage const & tracked,
                  cv::Mat const & opposite_mask, cv::Point2f offset)
{
    output << name << "[";
    for (auto const & key : tracked.features.keypoints)
    {
        auto const point = key.pt + offset;
        bool const visible = std::isfinite(point.x) && std::isfinite(point.y) && point.x >= 0 && point.y >= 0 &&
                             point.x < opposite_mask.cols && point.y < opposite_mask.rows &&
                             opposite_mask.at<unsigned char>(static_cast<int>(point.y), static_cast<int>(point.x)) != 0;
        output << "[:" << key.pt.x << key.pt.y << static_cast<int>(visible) << "]";
    }
    output << "]";
}

void write_translation(cv::FileStorage & output, std::size_t index, TrackedImage const & before,
                       TrackedImage const & after, cv::Mat const & mask, cv::Mat const & shifted_mask,
                       cv::Point2f offset, double frontend_ms)
{
    output << "{"
           << "frame_index" << static_cast<int>(index) << "time_ns" << std::to_string(before.identity.time_ns)
           << "status"
           << "success"
           << "frontend_ms" << frontend_ms << "geometry_valid" << static_cast<int>(after.matches.geometry_valid);
    write_points(output, "source", before, shifted_mask, offset);
    write_points(output, "target", after, mask, -offset);
    output << "matches"
           << "[";
    for (auto const & match : after.matches.inliers)
    {
        output << "[:" << match.queryIdx << match.trainIdx << "]";
    }
    output << "]"
           << "}";
}

int run_temporal(EurocSequence const & sequence, Profile const & profile, cv::FileStorage & output,
                 std::filesystem::path const & directory)
{
    OrbTracker tracker(profile.tracking);
    TrackedImage previous;
    cv::Mat previous_image;
    int failures = 0;
    for (std::size_t index = profile.first; index < profile.first + profile.count; ++index)
    {
        auto const time_ns = sequence.images(0)[index].time_ns;
        cv::Mat image;
        TrackedImage current;
        double read_ms = 0, frontend_ms = 0;
        try
        {
            auto const start = Clock::now();
            image = sequence.read_image(0, index);
            auto const loaded = Clock::now();
            process(tracker, image, sequence.camera(0).valid_mask, index + 1, time_ns, current);
            auto const processed = Clock::now();
            read_ms = milliseconds(start, loaded);
            frontend_ms = milliseconds(loaded, processed);
        }
        catch (std::exception const & error)
        {
            write_failure(output, index, time_ns, error.what());
            ++failures;
            tracker.reset();
            previous_image.release();
            continue;
        }
        // I/O failures abort this workload; they do not masquerade as tracking failures.
        write_temporal(output, current, read_ms, frontend_ms);
        if (index + 1 == profile.first + profile.count && !previous_image.empty())
        {
            snapshot(directory / "temporal.png", previous_image, image, previous, current);
        }
        previous = std::move(current);
        previous_image = image;
    }
    return failures;
}

int run_translation(EurocSequence const & sequence, Profile const & profile, cv::FileStorage & output,
                    std::filesystem::path const & directory)
{
    auto const & camera = sequence.camera(0);
    if (std::abs(static_cast<double>(profile.dx)) >= camera.resolution.width ||
        std::abs(static_cast<double>(profile.dy)) >= camera.resolution.height)
    {
        throw std::runtime_error("Translation exceeds the image extent");
    }
    cv::Matx23d const transform(1, 0, profile.dx, 0, 1, profile.dy);
    cv::Point2f const offset(static_cast<float>(profile.dx), static_cast<float>(profile.dy));
    cv::Mat shifted_mask;
    cv::warpAffine(camera.valid_mask, shifted_mask, transform, camera.resolution, cv::INTER_NEAREST,
                   cv::BORDER_CONSTANT);
    OrbTracker tracker(profile.tracking);
    int failures = 0;
    for (std::size_t index = profile.first; index < profile.first + profile.count; ++index)
    {
        auto const time_ns = sequence.images(0)[index].time_ns;
        cv::Mat image, shifted;
        TrackedImage before, after;
        double frontend_ms = 0;
        try
        {
            image = sequence.read_image(0, index);
            cv::warpAffine(image, shifted, transform, image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            tracker.reset();
            cv::setRNGSeed(profile.seed);
            auto const start = Clock::now();
            process(tracker, image, camera.valid_mask, 1, time_ns, before);
            process(tracker, shifted, shifted_mask, 2, time_ns + 1, after);
            frontend_ms = milliseconds(start, Clock::now());
        }
        catch (std::exception const & error)
        {
            write_failure(output, index, time_ns, error.what());
            ++failures;
            continue;
        }
        write_translation(output, index, before, after, camera.valid_mask, shifted_mask, offset, frontend_ms);
        if (index + 1 == profile.first + profile.count)
        {
            snapshot(directory / "translation.png", image, shifted, before, after);
        }
    }
    return failures;
}

void run(char const * input, Profile profile, std::filesystem::path const & directory, std::string const & workload)
{
    auto const sequence = EurocSequence::load(input);
    if (profile.first > sequence.images(0).size() || profile.count > sequence.images(0).size() - profile.first)
    {
        throw std::runtime_error("Requested frames exceed the dataset extent");
    }
    profile.tracking.camera_matrix = sequence.camera(0).K;
    cv::setNumThreads(1);
    cv::setRNGSeed(profile.seed);
    std::filesystem::create_directories(directory);
    cv::FileStorage output((directory / (workload + ".json")).string(),
                           cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    std::ofstream build(directory / "opencv-build.txt");
    if (!output.isOpened() || !build)
    {
        throw std::runtime_error("Cannot create benchmark outputs");
    }
    build << cv::getBuildInformation();
    build.close();
    if (!build)
    {
        throw std::runtime_error("Cannot write OpenCV build identity");
    }
    output << "protocol"
           << "euroc-orb-v2"
           << "workload" << workload << "execution"
           << (profile.tracking.orb.execution == Execution::cpu ? "cpu" : "cuda") << "opencv" << CV_VERSION
           << "compiler" << BENCHMARK_COMPILER << "build_type" << BENCHMARK_BUILD_TYPE << "width"
           << sequence.camera(0).resolution.width << "height" << sequence.camera(0).resolution.height;
    write_options(output, profile.tracking);
    output << "frames"
           << "[";
    int const failures = workload == "temporal" ? run_temporal(sequence, profile, output, directory)
                                                : run_translation(sequence, profile, output, directory);
    output << "]";
    output.release();
    if (failures != 0)
    {
        throw std::runtime_error(std::to_string(failures) + " frames failed in " + workload);
    }
}
} /* end anonymous namespace */

int main(int argc, char ** argv)
{
    if (argc != 5 && argc != 6)
    {
        std::cerr << "Usage: orb_benchmark MAV0_DIR PROFILE_JSON OUTPUT_DIR temporal|translation [--cuda]\n";
        return 1;
    }
    try
    {
        std::string const workload(argv[4]);
        if ((workload != "temporal" && workload != "translation") ||
            (argc == 6 && std::string_view(argv[5]) != "--cuda"))
        {
            throw std::runtime_error("Unknown workload or execution option");
        }
        auto const execution = argc == 6 ? Execution::cuda : Execution::cpu;
        run(argv[1], read_profile(argv[2], execution), argv[3], workload);
        return 0;
    }
    catch (std::exception const & error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
