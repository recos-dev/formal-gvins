/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/vision/orb_tracker.hpp>
#include "test_support.hpp"
#include <algorithm>
#include <set>
#include <opencv2/imgproc.hpp>

namespace
{
using formal_gvins::vision::FeatureObservation;
using formal_gvins::vision::ImageIdentity;
using formal_gvins::vision::OrbTracker;
using formal_gvins::vision::OrbTrackingDiagnostics;
using formal_gvins::vision::OrbTrackingOptions;
using formal_gvins::vision::PixelDomain;
using formal_gvins::vision::TrackedImage;
using formal_gvins::vision::VisionStatus;

bool same_observations(TrackedImage const & a, TrackedImage const & b)
{
    auto const & x = a.identity;
    auto const & y = b.identity;
    return x.session_id == y.session_id && x.camera_id == y.camera_id && x.image_id == y.image_id &&
           x.time_ns == y.time_ns && x.calibration_id == y.calibration_id && x.noise_model_id == y.noise_model_id &&
           a.pixel_domain == b.pixel_domain &&
           std::equal(a.observations.begin(), a.observations.end(), b.observations.begin(), b.observations.end(),
                      [](FeatureObservation const & p, FeatureObservation const & q)
                      { return p.track_id == q.track_id && p.u == q.u && p.v == q.v; });
}

void test_optional_diagnostics(formal_gvins::test::Context & test, OrbTrackingOptions const & options,
                               cv::Mat const & image)
{
    // Seed each invocation to compare the same RANSAC execution on both paths.
    OrbTracker plain(options), inspected(options);
    TrackedImage output, reference;
    OrbTrackingDiagnostics diagnostics;
    ImageIdentity identity{4, 2, 1, 1000000000, 7, 9};
    cv::Mat shifted;
    cv::warpAffine(image, shifted, cv::Matx23d(1, 0, 3, 0, 1, 2), image.size());
    auto compare = [&](cv::Mat const & frame, VisionStatus expected)
    {
        auto const saved = output;
        cv::setRNGSeed(71);
        auto const status = plain.process(identity, frame, {}, output);
        cv::setRNGSeed(71);
        auto const inspected_status = inspected.process(identity, frame, {}, reference, &diagnostics);
        test.expect(status == expected && inspected_status == expected, "interface", "matching operation status");
        test.expect(same_observations(output, reference), "interface", "diagnostics do not change observations or IDs");
        if (status != VisionStatus::success)
        {
            test.expect(same_observations(output, saved), "interface", "failure preserves the complete batch");
        }
    };
    compare(image, VisionStatus::success);
    test.expect(output.pixel_domain == PixelDomain::undistorted && !output.observations.empty(), "interface",
                "successful output declares the actual coordinate domain");
    auto const first = output;
    compare(image, VisionStatus::invalid_input); // Duplicate frame.
    ++identity.image_id;
    identity.time_ns += 50000000;
    compare({}, VisionStatus::invalid_input);
    compare(shifted, VisionStatus::success);
    // Mutating an owning common output must not change private tracking history.
    output.observations.clear();
    ++identity.image_id;
    identity.time_ns += 50000000;
    compare(image, VisionStatus::success);
    ++identity.image_id;
    identity.time_ns += 50000000;
    compare(cv::Mat::zeros(image.size(), CV_8UC1), VisionStatus::success);
    test.expect(output.observations.empty(), "interface", "successful empty frame retires all continuations");
    plain.reset();
    inspected.reset();
    ++identity.calibration_id;
    ++identity.image_id;
    identity.time_ns += 50000000;
    compare(image, VisionStatus::success);
    for (auto const & observation : output.observations)
    {
        test.expect(std::none_of(first.observations.begin(), first.observations.end(),
                                 [&](FeatureObservation const & old) { return old.track_id == observation.track_id; }),
                    "interface", "reset changes bindings without reusing IDs");
    }
}
} /* end anonymous namespace */

int main(int argc, char ** argv)
{
    using namespace formal_gvins::vision;
    formal_gvins::test::Context test;
    cv::setNumThreads(1);
    cv::RNG random(71);
    OrbTrackingOptions options;
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
    OrbMatchResult matches;
    OrbTrackingOptions geometry_options = options;
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

    OrbTrackingOptions invalid = options;
    invalid.camera_matrix(0, 0) = 0;
    test.expect(verify_matches(previous, current, neighbors, invalid, matches) == VisionStatus::invalid_input,
                "geometry", "invalid camera calibration rejected");

    OrbTracker tracker(options);
    cv::Mat image(480, 752, CV_8UC1);
    random.fill(image, cv::RNG::UNIFORM, 0, 256);
    ImageIdentity identity{1, 0, 1, 1000000000, 1, 1};
    TrackedImage output;
    OrbTrackingDiagnostics diagnostics;
    test.expect(tracker.process(identity, image, {}, output, &diagnostics) == VisionStatus::success, "tracking",
                "initial frame");
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
    test.expect(output.pixel_domain == PixelDomain::undistorted &&
                    output.observations.size() == diagnostics.features.keypoints.size() &&
                    diagnostics.features.descriptors.rows == static_cast<int>(output.observations.size()),
                "interface", "domain and diagnostic pairing");
    for (std::size_t i = 0; i < output.observations.size(); ++i)
    {
        auto const & pixel = diagnostics.features.keypoints[i].pt;
        test.expect(output.observations[i].u == pixel.x && output.observations[i].v == pixel.y, "interface",
                    "diagnostic keypoints retain observation order");
    }
    auto const first_ids = ids;
    diagnostics.features.descriptors.setTo(0);
    auto const saved_output = output;
    auto const saved_descriptor_data = diagnostics.features.descriptors.data;
    diagnostics.matches.ratio_passed = 1234;
    test.expect(tracker.process(identity, image, {}, output, &diagnostics) == VisionStatus::invalid_input &&
                    same_observations(output, saved_output) &&
                    diagnostics.features.descriptors.data == saved_descriptor_data &&
                    cv::countNonZero(diagnostics.features.descriptors) == 0 && diagnostics.matches.ratio_passed == 1234,
                "tracking", "duplicate image rejected atomically");
    cv::Mat shifted;
    cv::warpAffine(image, shifted, cv::Matx23d(1, 0, 3, 0, 1, 2), image.size());
    ++identity.image_id;
    identity.time_ns += 50000000;
    test.expect(tracker.process(identity, shifted, {}, output, &diagnostics) == VisionStatus::success, "tracking",
                "next frame");
    test.expect(diagnostics.matches.geometry_valid && diagnostics.matches.inliers.size() > 30U, "tracking",
                "temporal continuations survive caller mutation of published descriptors");
    std::size_t continued = 0;
    for (auto const & observation : output.observations)
    {
        continued += first_ids.count(observation.track_id);
    }
    test.expect(continued == diagnostics.matches.inliers.size(), "tracking", "only verified matches inherit IDs");
    auto const last_id = output.identity.image_id;
    ++identity.calibration_id;
    test.expect(tracker.process(identity, shifted, {}, output, &diagnostics) == VisionStatus::invalid_input &&
                    output.identity.image_id == last_id,
                "tracking", "calibration change requires reset");
    --identity.calibration_id;
    ++identity.image_id;
    identity.time_ns += 50000000;
    test.expect(tracker.process(identity, cv::Mat::zeros(image.size(), CV_8UC1), {}, output, &diagnostics) ==
                        VisionStatus::success &&
                    output.observations.empty(),
                "tracking", "empty frame retires history");
    tracker.reset();
    ++identity.image_id;
    identity.time_ns += 50000000;
    test.expect(tracker.process(identity, image, {}, output, &diagnostics) == VisionStatus::success, "tracking",
                "restart");
    for (auto const & observation : output.observations)
    {
        test.expect(first_ids.count(observation.track_id) == 0, "tracking", "reset does not reuse IDs");
    }
    if (options.orb.execution == Execution::cpu)
    {
        test_optional_diagnostics(test, options, image);
    }
#if !defined(FORMAL_GVINS_FRONTEND_CUDA)
    auto cuda_options = options;
    cuda_options.orb.execution = Execution::cuda;
    OrbTracker unavailable(cuda_options);
    auto const retained = output;
    diagnostics.matches.ratio_passed = 5678;
    test.expect(unavailable.process(identity, image, {}, output, &diagnostics) == VisionStatus::backend_unavailable &&
                    same_observations(output, retained) && diagnostics.matches.ratio_passed == 5678,
                "interface", "unavailable backend preserves both outputs without fallback");
    identity.session_id = 0;
    test.expect(unavailable.process(identity, image, {}, output, &diagnostics) == VisionStatus::invalid_input &&
                    same_observations(output, retained) && diagnostics.matches.ratio_passed == 5678,
                "interface", "tracker admission precedes extractor availability checks");
#endif
    return test.failures() == 0 ? 0 : 1;
}
