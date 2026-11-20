/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/vision/tracker.hpp>
#include "test_support.hpp"
#include <set>
#include <opencv2/imgproc.hpp>

int main(int argc, char ** argv)
{
    using namespace formal_gvins::vision;
    formal_gvins::test::Context test;
    cv::setNumThreads(1);
    cv::RNG random(71);
    TrackingOptions options;
    options.camera_matrix = {300, 0, 300, 0, 300, 200, 0, 0, 1};
    options.orb.execution = argc == 2 && std::string_view(argv[1]) == "--cuda" ? Execution::cuda : Execution::cpu;

    OrbFeatures previous, current;
    std::vector<std::vector<cv::DMatch>> neighbors;
    for (int i = 0; i < 80; ++i)
    {
        double const x = random.uniform(-2.0, 2.0), y = random.uniform(-1.5, 1.5), z = random.uniform(3.0, 8.0);
        previous.keypoints.emplace_back(
            cv::Point2f(300.0F + static_cast<float>(300 * x / z), 200.0F + static_cast<float>(300 * y / z)), 1);
        double const x2 = std::cos(0.08) * x + std::sin(0.08) * z + 0.3;
        double const z2 = -std::sin(0.08) * x + std::cos(0.08) * z;
        cv::Point2f p(300.0F + static_cast<float>(300 * x2 / z2), 200.0F + static_cast<float>(300 * (y + 0.05) / z2));
        if (i >= 64)
        {
            p = {random.uniform(50.0F, 550.0F), random.uniform(50.0F, 350.0F)};
        }
        current.keypoints.emplace_back(p, 1);
        neighbors.push_back({cv::DMatch(i, i, 10), cv::DMatch(i, (i + 1) % 80, 100)});
    }
    MatchResult matches;
    TrackingOptions geometry_options = options;
    geometry_options.epipolar_threshold_px = 0.1;
    test.expect(verify_matches(previous, current, neighbors, geometry_options, matches) == VisionStatus::success &&
                    matches.geometry_valid,
                "geometry", "nonplanar calibrated scene");
    std::size_t true_inliers = 0, false_inliers = 0;
    for (auto const & match : matches.inliers)
    {
        if (match.queryIdx < 64)
        {
            ++true_inliers;
        }
        else
        {
            ++false_inliers;
        }
    }
    test.expect(true_inliers >= 60 && false_inliers <= 2, "geometry", "known outlier rejection");
    auto conflicting = neighbors;
    conflicting.push_back({cv::DMatch(64, 0, 20), cv::DMatch(64, 1, 100)});
    test.expect(verify_matches(previous, current, conflicting, geometry_options, matches) == VisionStatus::success,
                "matching", "conflicting descriptor proposals");
    std::set<int> query_indices, train_indices;
    for (auto const & match : matches.inliers)
    {
        test.expect(query_indices.insert(match.queryIdx).second && train_indices.insert(match.trainIdx).second,
                    "matching", "one-to-one associations");
    }
    auto const retained_count = matches.inliers.size();
    auto malformed = neighbors;
    malformed[0][0].trainIdx = -1;
    test.expect(verify_matches(previous, current, malformed, options, matches) == VisionStatus::invalid_input &&
                    matches.inliers.size() == retained_count,
                "matching", "invalid index preserves result");
    auto tied = neighbors;
    for (auto & pair : tied)
    {
        pair[0].distance = pair[1].distance = 0;
    }
    test.expect(verify_matches(previous, current, tied, options, matches) == VisionStatus::success &&
                    matches.ratio_passed == 0 && matches.inliers.empty() && !matches.geometry_valid,
                "matching", "zero-distance ties rejected");
    neighbors.resize(7);
    test.expect(verify_matches(previous, current, neighbors, options, matches) == VisionStatus::success &&
                    matches.inliers.empty() && !matches.geometry_valid,
                "matching", "no all-inlier fallback below profile candidate minimum");

    TrackingOptions invalid = options;
    invalid.camera_matrix(0, 0) = 0;
    test.expect(verify_matches(previous, current, neighbors, invalid, matches) == VisionStatus::invalid_input,
                "geometry", "invalid camera calibration rejected");

    OrbTracker tracker(options);
    cv::Mat image(480, 752, CV_8UC1);
    random.fill(image, cv::RNG::UNIFORM, 0, 256);
    ImageIdentity identity{1, 0, 1, 1000000000, 1, 1};
    TrackedImage output;
    test.expect(tracker.process(identity, image, {}, output) == VisionStatus::success, "tracking", "initial frame");
    test.expect(!output.observations.empty() && output.observations.size() <= 400U, "tracking", "feature budget");
    std::vector<int> occupancy(48, 0);
    std::set<std::uint64_t> ids;
    for (auto const & observation : output.observations)
    {
        int const cell =
            static_cast<int>(observation.v * 6 / image.rows) * 8 + static_cast<int>(observation.u * 8 / image.cols);
        ++occupancy[static_cast<std::size_t>(cell)];
        ids.insert(observation.track_id);
    }
    for (std::size_t cell = 0; cell < occupancy.size(); ++cell)
    {
        test.expect(occupancy[cell] <= 400 / 48 + (cell < 400 % 48 ? 1 : 0), "tracking", "spatial quota");
    }
    test.expect(ids.size() == output.observations.size(), "tracking", "unique IDs");
    auto const first_ids = ids;
    output.features.descriptors.setTo(0);
    test.expect(tracker.process(identity, image, {}, output) == VisionStatus::invalid_input &&
                    output.identity.image_id == 1,
                "tracking", "duplicate image rejected atomically");
    cv::Mat shifted;
    cv::warpAffine(image, shifted, cv::Matx23d(1, 0, 3, 0, 1, 2), image.size());
    ++identity.image_id;
    identity.time_ns += 50000000;
    test.expect(tracker.process(identity, shifted, {}, output) == VisionStatus::success, "tracking", "next frame");
    test.expect(output.matches.geometry_valid && output.matches.inliers.size() > 30U, "tracking",
                "temporal continuations survive caller mutation of published descriptors");
    std::size_t continued = 0;
    for (auto const & observation : output.observations)
    {
        continued += first_ids.count(observation.track_id);
    }
    test.expect(continued == output.matches.inliers.size(), "tracking", "only verified matches inherit IDs");
    auto const last_id = output.identity.image_id;
    ++identity.calibration_id;
    test.expect(tracker.process(identity, shifted, {}, output) == VisionStatus::invalid_input &&
                    output.identity.image_id == last_id,
                "tracking", "calibration change requires reset");
    --identity.calibration_id;
    ++identity.image_id;
    identity.time_ns += 50000000;
    test.expect(tracker.process(identity, cv::Mat::zeros(image.size(), CV_8UC1), {}, output) == VisionStatus::success &&
                    output.observations.empty(),
                "tracking", "empty frame retires history");
    tracker.reset();
    ++identity.image_id;
    identity.time_ns += 50000000;
    test.expect(tracker.process(identity, image, {}, output) == VisionStatus::success, "tracking", "restart");
    for (auto const & observation : output.observations)
    {
        test.expect(first_ids.count(observation.track_id) == 0, "tracking", "reset does not reuse IDs");
    }
    return test.failures() == 0 ? 0 : 1;
}
