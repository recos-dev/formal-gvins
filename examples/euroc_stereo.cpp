/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/dataset/euroc.hpp>
#include <formal_gvins/vision/stereo_tracker.hpp>
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

int main(int argc, char ** argv)
{
    using namespace formal_gvins::vision;
    if (argc < 2)
    {
        std::cerr << "Usage: euroc_stereo MAV0_DIR [--cuda] [--frames N] [--output DIR]\n";
        return 1;
    }
    try
    {
        Execution execution = Execution::cpu;
        std::size_t limit = std::numeric_limits<std::size_t>::max();
        std::filesystem::path directory;
        for (int i = 2; i < argc; ++i)
        {
            std::string_view const arg(argv[i]);
            if (arg == "--cuda")
            {
                execution = Execution::cuda;
            }
            else if (arg == "--frames" && i + 1 < argc)
            {
                std::string_view const text(argv[++i]);
                auto const result = std::from_chars(text.data(), text.data() + text.size(), limit);
                if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || limit == 0)
                {
                    throw std::runtime_error("Expected a positive frame count");
                }
            }
            else if (arg == "--output" && i + 1 < argc)
            {
                directory = argv[++i];
            }
            else
            {
                throw std::runtime_error("Unknown or incomplete option: " + std::string(arg));
            }
        }
        auto const input = std::filesystem::canonical(argv[1]);
        auto const sequence = formal_gvins::dataset::EurocSequence::load(input);
        char const * backend = execution == Execution::cpu ? "cpu" : "cuda";
        if (directory.empty())
        {
            directory =
                std::filesystem::path("output") / input.parent_path().filename() / (std::string("stereo-") + backend);
        }
        StereoTrackingOptions options;
        std::array<cv::Mat, 2> masks;
        std::array<cv::Matx33d, 2> R_BC;
        std::array<cv::Vec3d, 2> p_BC;
        for (std::size_t c = 0; c < 2; ++c)
        {
            auto const & calibration = sequence.camera(c);
            options.cameras[c].camera = calibration.intrinsics;
            options.cameras[c].orb.execution = execution;
            masks[c] = calibration.valid_mask;
            for (int row = 0; row < 3; ++row)
            {
                p_BC[c][row] = calibration.T_BC(row, 3);
                for (int col = 0; col < 3; ++col)
                {
                    R_BC[c](row, col) = calibration.T_BC(row, col);
                }
            }
        }
        options.rig.R_RL = R_BC[1].t() * R_BC[0];
        options.rig.t_RL = R_BC[1].t() * (p_BC[0] - p_BC[1]);
        std::filesystem::create_directories(directory);
        std::ofstream frames(directory / "frames.csv"), observations(directory / "observations.csv"),
            events(directory / "events.csv"), profile(directory / "profile.txt");
        if (!frames || !observations || !events || !profile)
        {
            throw std::runtime_error("Cannot create output files");
        }
        frames << "pair,time_ns,left_features,right_features,stereo_links,geometry_rejected,id_conflicts,frontend_ms\n";
        observations << "camera_id,image_id,time_ns,track_id,u,v,x,y,age\n" << std::setprecision(17);
        events << "camera_id,image_id,track_id,change\n";
        profile << std::setprecision(17) << "Input: " << input << "\nExecution: " << backend
                << "\nPairing: exact integer acquisition timestamps; no nearest-frame pairing or motion compensation"
                << "\nRaw global-shutter stereo; session=1; calibration IDs=1,2; noise ID=1 (unqualified trial model)"
                << "\nR_RL: " << options.rig.R_RL << "\nt_RL (m): " << options.rig.t_RL
                << "\nStereo ratio: " << options.ratio
                << "; normalized Sampson threshold: " << options.rig.epipolar_threshold
                << "; denominator floor: " << options.rig.minimum_epipolar_denominator
                << "\nMono tracking: library trial defaults; see the ORB frontend profile."
                << "\nNo triangulation, IMU fusion or trajectory estimate.\n"
                << cv::getBuildInformation();
        cv::setNumThreads(1);
        StereoOrbTracker tracker(options);
        std::array<std::size_t, 2> indices{};
        std::size_t pairs = 0, skipped = 0, total_links = 0;
        auto const & left = sequence.images(0);
        auto const & right = sequence.images(1);
        while (pairs < limit && indices[0] < left.size() && indices[1] < right.size())
        {
            if (left[indices[0]].time_ns != right[indices[1]].time_ns)
            {
                ++indices[left[indices[0]].time_ns < right[indices[1]].time_ns ? 0 : 1];
                ++skipped;
                continue;
            }
            std::array<cv::Mat, 2> images{sequence.read_image(0, indices[0]), sequence.read_image(1, indices[1])};
            std::array<ImageIdentity, 2> identities;
            for (std::size_t c = 0; c < 2; ++c)
            {
                identities[c] = {
                    1, static_cast<std::uint32_t>(c), indices[c] + 1, sequence.images(c)[indices[c]].time_ns, c + 1, 1};
            }
            std::array<TrackedImage, 2> tracked;
            StereoTrackingDiagnostics details;
            auto const start = std::chrono::steady_clock::now();
            auto const status = tracker.process(identities, images, masks, tracked, &details);
            double const elapsed =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            if (status != VisionStatus::success || details.association == StereoStatus::backend_failure)
            {
                throw std::runtime_error("Stereo frontend failed at pair " + std::to_string(pairs));
            }
            ++pairs;
            total_links += details.matches.inliers.size();
            frames << pairs << ',' << identities[0].time_ns << ',' << tracked[0].observations.size() << ','
                   << tracked[1].observations.size() << ',' << details.matches.inliers.size() << ','
                   << details.matches.geometry_rejected << ',' << details.identity_conflicts << ',' << elapsed << '\n';
            for (std::size_t c = 0; c < 2; ++c)
            {
                for (std::size_t i = 0; i < tracked[c].observations.size(); ++i)
                {
                    auto const & o = tracked[c].observations[i];
                    observations << c << ',' << identities[c].image_id << ',' << identities[c].time_ns << ','
                                 << o.track_id << ',' << o.u << ',' << o.v << ',' << o.x << ',' << o.y << ','
                                 << details.cameras[c].ages[i] << '\n';
                }
                for (auto const & event : tracked[c].events)
                {
                    char const * change = event.change == TrackChange::started     ? "started"
                                          : event.change == TrackChange::continued ? "continued"
                                                                                   : "ended";
                    events << c << ',' << identities[c].image_id << ',' << event.track_id << ',' << change << '\n';
                }
            }
            if (pairs % 100 == 0 || pairs == limit || indices[0] + 1 == left.size() || indices[1] + 1 == right.size())
            {
                cv::Mat picture;
                cv::drawMatches(images[0], details.cameras[0].features.keypoints, images[1],
                                details.cameras[1].features.keypoints, details.matches.inliers, picture,
                                cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), {},
                                cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                if (!cv::imwrite((directory / ("stereo-" + std::to_string(pairs) + ".png")).string(), picture))
                {
                    throw std::runtime_error("Cannot write stereo visualization");
                }
                std::cout << '\r' << backend << ": " << pairs << " stereo pairs" << std::flush;
            }
            ++indices[0];
            ++indices[1];
        }
        frames.flush();
        observations.flush();
        events.flush();
        profile.flush();
        if (!frames || !observations || !events || !profile || pairs == 0)
        {
            throw std::runtime_error("Output write failed or no synchronized pairs");
        }
        std::cout << "\nStereo links: " << total_links << "; unpaired exposures skipped: " << skipped
                  << "\nOutput: " << directory << '\n';
    }
    catch (std::exception const & error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
