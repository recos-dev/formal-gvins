/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/dataset/euroc.hpp>
#include <formal_gvins/vision/orb_tracker.hpp>
#include <algorithm>
#include <charconv>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
using Clock = std::chrono::steady_clock;

std::size_t count(std::string_view text)
{
    std::size_t value = 0;
    auto const result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || value == 0)
    {
        throw std::runtime_error("Expected a positive frame count");
    }
    return value;
}

double milliseconds(Clock::time_point start, Clock::time_point finish)
{
    return std::chrono::duration<double, std::milli>(finish - start).count();
}
} /* end anonymous namespace */

int main(int argc, char ** argv)
{
    using namespace formal_gvins::vision;
    if (argc < 2)
    {
        std::cerr << "Usage: euroc_frontend MAV0_DIR [--cuda] [--frames N] [--output DIR]\n";
        return 1;
    }
    try
    {
        OrbTrackingOptions options;
        std::size_t frame_limit = std::numeric_limits<std::size_t>::max();
        std::filesystem::path output_directory;
        for (int i = 2; i < argc; ++i)
        {
            std::string_view const argument(argv[i]);
            if (argument == "--cuda")
            {
                options.orb.execution = Execution::cuda;
            }
            else if (argument == "--frames" && i + 1 < argc)
            {
                frame_limit = count(argv[++i]);
            }
            else if (argument == "--output" && i + 1 < argc)
            {
                output_directory = argv[++i];
            }
            else
            {
                throw std::runtime_error("Unknown or incomplete option: " + std::string(argument));
            }
        }
        std::filesystem::path const input = std::filesystem::canonical(argv[1]);
        char const * execution = options.orb.execution == Execution::cuda ? "cuda" : "cpu";
        if (output_directory.empty())
        {
            output_directory = std::filesystem::path("output") / input.parent_path().filename() / execution;
        }
        auto const sequence = formal_gvins::dataset::EurocSequence::load(input);
        options.camera = sequence.camera(0).intrinsics;
        auto const & images = sequence.images(0);
        frame_limit = std::min(frame_limit, images.size());
        std::filesystem::create_directories(output_directory);
        std::ofstream metrics(output_directory / "frames.csv");
        std::ofstream observations(output_directory / "observations.csv");
        std::ofstream profile(output_directory / "profile.txt");
        std::ofstream events(output_directory / "events.csv");
        if (!metrics || !observations || !profile || !events)
        {
            throw std::runtime_error("Cannot open output files");
        }
        metrics << "image_id,time_ns,features,ratio_passed,inliers,geometry_valid,read_image_ms,frontend_ms"
                << ",detected,support_rejected,displacement_rejected,quality_rejected,inverse_rejected"
                << ",quota_rejected,spacing_rejected";
        for (auto const * name : frontend_stage_names)
        {
            metrics << ',' << name << "_ms," << name << "_process_peak_kib";
        }
        metrics << '\n';
        observations << "image_id,time_ns,track_id,u,v,x,y,age\n";
        events << "image_id,track_id,change\n";
        observations << std::setprecision(12);
        profile << "Input: " << input << "\nExecution: " << execution << "\nCamera: cam0 (mono)\n"
                << "Session: 1; calibration binding: 1; noise-model binding: 1 (unqualified pixel model)\n"
                << "ORB: candidates=" << options.orb.feature_count << "; levels=" << options.orb.pyramid_levels
                << "; scale=" << options.orb.scale_factor << "; FAST=" << options.orb.fast_threshold
                << "; WTA_K=2; Hamming\nSelection: " << options.selection.capacity
                << "; grid=" << options.selection.columns << 'x' << options.selection.rows
                << "; ratio=" << options.ratio
                << "; five-point essential RANSAC threshold=" << options.epipolar_threshold
                << " normalized; confidence=" << options.confidence << "; max iterations=" << options.ransac_iterations
                << '\n'
                << std::setprecision(17) << "Intrinsics fx fy cx cy: " << options.camera.fx << ' ' << options.camera.fy
                << ' ' << options.camera.cx << ' ' << options.camera.cy << "\nRadtan k1 k2 p1 p2: " << options.camera.k1
                << ' ' << options.camera.k2 << ' ' << options.camera.p1 << ' ' << options.camera.p2
                << "\nT_BC: " << sequence.camera(0).T_BC << '\n'
                << "Frames: " << frame_limit << "; IMU samples: " << sequence.imu().size()
                << "; ground-truth states: " << sequence.ground_truth().size() << '\n'
                << "Frontend only; IMU and GT are read but not fused or scored. No trajectory estimate.\n"
                << cv::getBuildInformation();
        cv::setNumThreads(1);
        OrbTracker tracker(options);
        OrbTrackingDiagnostics previous;
        cv::Mat previous_image;
        std::vector<double> timings;
        std::size_t total_inliers = 0;
        for (std::size_t index = 0; index < frame_limit; ++index)
        {
            auto const start = Clock::now();
            cv::Mat const image = sequence.read_image(0, index);
            auto const loaded = Clock::now();
            TrackedImage tracked;
            OrbTrackingDiagnostics diagnostics;
            ImageIdentity const identity{1, 0, index + 1, images[index].time_ns, 1, 1};
            if (tracker.process(identity, image, sequence.camera(0).valid_mask, tracked, &diagnostics) !=
                VisionStatus::success)
            {
                throw std::runtime_error("Frontend failed at image " + std::to_string(index));
            }
            auto const processed = Clock::now();
            double const elapsed = milliseconds(loaded, processed);
            metrics << index + 1 << ',' << identity.time_ns << ',' << tracked.observations.size() << ','
                    << diagnostics.matches.descriptors.forward_ratio_passed << ',' << diagnostics.matches.inliers.size()
                    << ',' << diagnostics.matches.geometry_valid << ',' << milliseconds(start, loaded) << ',' << elapsed
                    << ',' << diagnostics.features.localization.detected << ','
                    << diagnostics.features.localization.support_rejected << ','
                    << diagnostics.features.localization.displacement_rejected << ','
                    << diagnostics.features.localization.quality_rejected << ',' << diagnostics.inverse_rejected << ','
                    << diagnostics.selection.quota_rejected << ',' << diagnostics.selection.spacing_rejected;
            for (auto const & stage : diagnostics.stages)
            {
                metrics << ',';
                if (stage.executed)
                {
                    metrics << stage.milliseconds;
                }
                metrics << ',';
                if (stage.process_peak_kib != 0)
                {
                    metrics << stage.process_peak_kib;
                }
            }
            metrics << '\n';
            for (std::size_t i = 0; i < tracked.observations.size(); ++i)
            {
                auto const & observation = tracked.observations[i];
                observations << index + 1 << ',' << identity.time_ns << ',' << observation.track_id << ','
                             << observation.u << ',' << observation.v << ',' << observation.x << ',' << observation.y
                             << ',' << diagnostics.ages[i] << '\n';
            }
            for (auto const & event : tracked.events)
            {
                char const * change = event.change == TrackChange::started     ? "started"
                                      : event.change == TrackChange::continued ? "continued"
                                                                               : "ended";
                events << index + 1 << ',' << event.track_id << ',' << change << '\n';
            }
            if (index >= 10)
            {
                timings.push_back(elapsed);
            }
            total_inliers += diagnostics.matches.inliers.size();
            if (index > 0 && (index % 100 == 0 || index + 1 == frame_limit))
            {
                cv::Mat visualization;
                cv::drawMatches(previous_image, previous.features.keypoints, image, diagnostics.features.keypoints,
                                diagnostics.matches.inliers, visualization, cv::Scalar(0, 255, 0),
                                cv::Scalar(0, 0, 255), {}, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                if (!cv::imwrite((output_directory / ("tracks-" + std::to_string(index + 1) + ".png")).string(),
                                 visualization))
                {
                    throw std::runtime_error("Cannot write visualization");
                }
            }
            previous = std::move(diagnostics);
            previous_image = image;
            if ((index + 1) % 100 == 0 || index + 1 == frame_limit)
            {
                std::cout << '\r' << execution << ": " << index + 1 << '/' << frame_limit << " images" << std::flush;
            }
        }
        metrics.flush();
        observations.flush();
        profile.flush();
        events.flush();
        if (!metrics || !observations || !profile || !events)
        {
            throw std::runtime_error("Output write failed");
        }
        std::cout << "\nGeometrically accepted temporal matches: " << total_inliers << '\n';
        if (!timings.empty())
        {
            std::sort(timings.begin(), timings.end());
            std::cout << "Frontend latency (after 10 warmup frames, includes CUDA transfers/synchronization): "
                      << "median=" << timings[timings.size() / 2]
                      << " ms; p95=" << timings[static_cast<std::size_t>(0.95 * (timings.size() - 1))] << " ms\n";
        }
        std::cout << "Output: " << output_directory << '\n';
        return 0;
    }
    catch (std::exception const & error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
