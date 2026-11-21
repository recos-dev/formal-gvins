/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/dataset/euroc.hpp>
#include <formal_gvins/vision/orb_tracker.hpp>
#include <algorithm>
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
using formal_gvins::vision::OrbTrackingDiagnostics;
using formal_gvins::vision::OrbTrackingOptions;
using formal_gvins::vision::TrackedImage;
using formal_gvins::vision::VisionStatus;

struct Frame
{
    TrackedImage tracks;
    OrbTrackingDiagnostics diagnostics;
};

struct Profile
{
    std::size_t first = 0;
    std::size_t count = 0;
    int seed = 0;
    int dx = 0;
    int dy = 0;
    OrbTrackingOptions tracking;
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
    if (static_cast<std::string>(node["protocol"]) != "euroc-orb-v4" || integer(node, "camera") != 0 || first < 0 ||
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
    options.selection.capacity = integer(tracking, "feature_capacity");
    options.selection.columns = integer(tracking, "grid_columns");
    options.selection.rows = integer(tracking, "grid_rows");
    options.selection.minimum_distance = real(tracking, "minimum_distance_px");
    options.ratio = static_cast<float>(real(tracking, "ratio"));
    options.epipolar_threshold = real(tracking, "epipolar_threshold");
    options.confidence = real(tracking, "confidence");
    options.ransac_iterations = integer(tracking, "ransac_iterations");
    options.orb.localization.window_radius = integer(tracking, "localization_window_radius");
    options.orb.localization.iterations = integer(tracking, "localization_iterations");
    options.orb.localization.epsilon = real(tracking, "localization_epsilon");
    options.orb.localization.maximum_displacement = real(tracking, "localization_maximum_displacement_px");
    options.orb.localization.minimum_eigenvalue = real(tracking, "localization_minimum_eigenvalue");
    options.orb.localization.maximum_condition = real(tracking, "localization_maximum_condition");
    options.inverse.radius = real(tracking, "inverse_radius");
    options.inverse.residual_px = real(tracking, "inverse_residual_px");
    options.inverse.maximum_step = real(tracking, "inverse_maximum_step");
    options.inverse.minimum_slope = real(tracking, "inverse_minimum_slope");
    options.inverse.iterations = integer(tracking, "inverse_iterations");
    options.minimum_epipolar_denominator = real(tracking, "epipolar_minimum_denominator");
    options.rotation_iterations = integer(tracking, "rotation_iterations");
    options.rotation_support_fraction = real(tracking, "rotation_support_fraction");
    return result;
}

double milliseconds(Clock::time_point start, Clock::time_point finish)
{
    return std::chrono::duration<double, std::milli>(finish - start).count();
}

void process(OrbTracker & tracker, cv::Mat const & image, cv::Mat const & mask, std::uint64_t image_id,
             std::uint64_t time_ns, Frame & output)
{
    formal_gvins::vision::ImageIdentity const identity{1, 0, image_id, time_ns, 1, 1};
    VisionStatus const status = tracker.process(identity, image, mask, output.tracks, &output.diagnostics);
    if (status != VisionStatus::success)
    {
        throw std::runtime_error("ORB process failed, status=" + std::to_string(static_cast<int>(status)));
    }
}

void snapshot(std::filesystem::path const & path, cv::Mat const & previous, cv::Mat const & current,
              Frame const & before, Frame const & after)
{
    cv::Mat image;
    cv::drawMatches(previous, before.diagnostics.features.keypoints, current, after.diagnostics.features.keypoints,
                    after.diagnostics.matches.inliers, image, cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), {},
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    if (!cv::imwrite(path.string(), image))
    {
        throw std::runtime_error("Cannot write " + path.string());
    }
}

void write_options(cv::FileStorage & output, OrbTrackingOptions const & options)
{
    output << "options"
           << "{"
           << "feature_count" << options.orb.feature_count << "scale_factor" << options.orb.scale_factor
           << "pyramid_levels" << options.orb.pyramid_levels << "fast_threshold" << options.orb.fast_threshold
           << "feature_capacity" << options.selection.capacity << "grid_columns" << options.selection.columns
           << "grid_rows" << options.selection.rows << "minimum_distance_px" << options.selection.minimum_distance
           << "ratio" << options.ratio << "epipolar_threshold" << options.epipolar_threshold << "confidence"
           << options.confidence << "ransac_iterations" << options.ransac_iterations << "localization_window_radius"
           << options.orb.localization.window_radius << "localization_iterations" << options.orb.localization.iterations
           << "localization_epsilon" << options.orb.localization.epsilon << "localization_maximum_displacement_px"
           << options.orb.localization.maximum_displacement << "localization_minimum_eigenvalue"
           << options.orb.localization.minimum_eigenvalue << "localization_maximum_condition"
           << options.orb.localization.maximum_condition << "inverse_radius" << options.inverse.radius
           << "inverse_residual_px" << options.inverse.residual_px << "inverse_maximum_step"
           << options.inverse.maximum_step << "inverse_minimum_slope" << options.inverse.minimum_slope
           << "inverse_iterations" << static_cast<int>(options.inverse.iterations) << "epipolar_minimum_denominator"
           << options.minimum_epipolar_denominator << "rotation_iterations" << options.rotation_iterations
           << "rotation_support_fraction" << options.rotation_support_fraction << "}";
    // These are fixed properties of the production adapter, not benchmark switches.
    output << "adapter"
           << "{"
           << "wta_k" << 2 << "patch_size" << 31 << "edge_threshold" << 31 << "score_type"
           << "HARRIS"
           << "descriptor_smoothing"
           << "enabled"
           << "}";
}

void write_device(cv::FileStorage & output, Execution execution)
{
    if (execution != Execution::cuda)
    {
        return;
    }
#if defined(FORMAL_GVINS_FRONTEND_CUDA)
    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        throw std::runtime_error("CUDA requested but no enabled device is available");
    }
    cv::cuda::DeviceInfo const device;
    if (!device.isCompatible())
    {
        throw std::runtime_error("OpenCV CUDA build is incompatible with the selected device");
    }
    output << "cuda_device"
           << "{"
           << "index" << device.deviceID() << "name" << device.name() << "compute_capability"
           << std::to_string(device.majorVersion()) + "." + std::to_string(device.minorVersion()) << "memory_bytes"
           << std::to_string(device.totalGlobalMem()) << "}";
#else
    (void)output;
    throw std::runtime_error("CUDA requested but this benchmark was built without CUDA support");
#endif
}

void write_failure(cv::FileStorage & output, std::size_t index, std::uint64_t time_ns, char const * error)
{
    output << "{"
           << "frame_index" << static_cast<int>(index) << "time_ns" << std::to_string(time_ns) << "status"
           << "failed"
           << "error" << error << "}";
}

void write_telemetry(cv::FileStorage & output, Frame const & frame,
                     formal_gvins::vision::FrontendMetrics const & stages)
{
    auto const & d = frame.diagnostics;
    auto const & l = d.features.localization;
    output << "diagnostics"
           << "{"
           << "detected" << static_cast<int>(l.detected) << "support_rejected" << static_cast<int>(l.support_rejected)
           << "displacement_rejected" << static_cast<int>(l.displacement_rejected) << "quality_rejected"
           << static_cast<int>(l.quality_rejected) << "inverse_rejected" << static_cast<int>(d.inverse_rejected)
           << "forward_ratio_passed" << static_cast<int>(d.matches.descriptors.forward_ratio_passed)
           << "reverse_ratio_passed" << static_cast<int>(d.matches.descriptors.reverse_ratio_passed)
           << "reciprocal_candidates" << static_cast<int>(d.reciprocal_candidates) << "geometric_candidates"
           << static_cast<int>(d.geometric_candidates) << "quota_rejected"
           << static_cast<int>(d.selection.quota_rejected) << "spacing_rejected"
           << static_cast<int>(d.selection.spacing_rejected) << "invalid_rejected"
           << static_cast<int>(d.selection.invalid_rejected) << "rotation_checked"
           << static_cast<int>(d.matches.rotation_checked) << "}";
    output << "stages"
           << "{";
    for (std::size_t i = 0; i < stages.size(); ++i)
    {
        output << formal_gvins::vision::frontend_stage_names[i] << "{"
               << "executed" << static_cast<int>(stages[i].executed) << "ms" << stages[i].milliseconds
               << "process_peak_kib" << std::to_string(stages[i].process_peak_kib) << "}";
    }
    output << "}"
           << "ages"
           << "[";
    for (auto age : d.ages)
    {
        output << std::to_string(age);
    }
    output << "]"
           << "events"
           << "[";
    constexpr char const * changes[] = {"started", "continued", "ended"};
    for (auto const & event : frame.tracks.events)
    {
        output << "[:" << std::to_string(event.track_id) << changes[static_cast<std::size_t>(event.change)] << "]";
    }
    output << "]";
}

void write_temporal(cv::FileStorage & output, Frame const & tracked, double read_ms, double frontend_ms)
{
    output << "{"
           << "frame_index" << static_cast<int>(tracked.tracks.identity.image_id - 1) << "time_ns"
           << std::to_string(tracked.tracks.identity.time_ns) << "status"
           << "success"
           << "features" << static_cast<int>(tracked.diagnostics.features.keypoints.size()) << "ratio_passed"
           << static_cast<int>(tracked.diagnostics.matches.descriptors.forward_ratio_passed) << "inliers"
           << static_cast<int>(tracked.diagnostics.matches.inliers.size()) << "geometry_valid"
           << static_cast<int>(tracked.diagnostics.matches.geometry_valid) << "read_image_ms" << read_ms
           << "frontend_ms" << frontend_ms << "observations"
           << "[";
    for (auto const & observation : tracked.tracks.observations)
    {
        output << "[:" << observation.u << observation.v << std::to_string(observation.track_id) << "]";
    }
    output << "]";
    write_telemetry(output, tracked, tracked.diagnostics.stages);
    output << "}";
}

void write_points(cv::FileStorage & output, char const * name, Frame const & tracked, cv::Mat const & opposite_mask,
                  cv::Point2f offset)
{
    output << name << "[";
    for (auto const & key : tracked.diagnostics.features.keypoints)
    {
        auto const point = key.pt + offset;
        bool const visible = std::isfinite(point.x) && std::isfinite(point.y) && point.x >= 0 && point.y >= 0 &&
                             point.x < opposite_mask.cols && point.y < opposite_mask.rows &&
                             opposite_mask.at<unsigned char>(static_cast<int>(point.y), static_cast<int>(point.x)) != 0;
        output << "[:" << key.pt.x << key.pt.y << static_cast<int>(visible) << "]";
    }
    output << "]";
}

void write_translation(cv::FileStorage & output, std::size_t index, Frame const & before, Frame const & after,
                       cv::Mat const & mask, cv::Mat const & shifted_mask, cv::Point2f offset, double frontend_ms)
{
    output << "{"
           << "frame_index" << static_cast<int>(index) << "time_ns" << std::to_string(before.tracks.identity.time_ns)
           << "status"
           << "success"
           << "frontend_ms" << frontend_ms << "geometry_valid"
           << static_cast<int>(after.diagnostics.matches.geometry_valid);
    write_points(output, "source", before, shifted_mask, offset);
    write_points(output, "target", after, mask, -offset);
    output << "matches"
           << "[";
    for (auto const & match : after.diagnostics.matches.inliers)
    {
        output << "[:" << match.queryIdx << match.trainIdx << "]";
    }
    output << "]";
    auto stages = after.diagnostics.stages;
    for (std::size_t i = 0; i < stages.size(); ++i)
    {
        stages[i].executed = stages[i].executed || before.diagnostics.stages[i].executed;
        stages[i].milliseconds += before.diagnostics.stages[i].milliseconds;
        stages[i].process_peak_kib =
            std::max(stages[i].process_peak_kib, before.diagnostics.stages[i].process_peak_kib);
    }
    write_telemetry(output, after, stages);
    output << "}";
}

int run_temporal(EurocSequence const & sequence, Profile const & profile, cv::FileStorage & output,
                 std::filesystem::path const & directory)
{
    OrbTracker tracker(profile.tracking);
    Frame previous;
    cv::Mat previous_image;
    int failures = 0;
    for (std::size_t index = profile.first; index < profile.first + profile.count; ++index)
    {
        auto const time_ns = sequence.images(0)[index].time_ns;
        cv::Mat image;
        Frame current;
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
        Frame before, after;
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
    profile.tracking.camera = sequence.camera(0).intrinsics;
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
           << "euroc-orb-v4"
           << "workload" << workload << "execution"
           << (profile.tracking.orb.execution == Execution::cpu ? "cpu" : "cuda") << "opencv" << CV_VERSION
           << "compiler" << BENCHMARK_COMPILER << "build_type" << BENCHMARK_BUILD_TYPE << "width"
           << sequence.camera(0).resolution.width << "height" << sequence.camera(0).resolution.height;
    write_options(output, profile.tracking);
    write_device(output, profile.tracking.orb.execution);
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
