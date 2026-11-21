/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <formal_gvins/vision/orb.hpp>
#include <formal_gvins/camera/radtan.hpp>
#include <formal_gvins/vision/tracker.hpp>
#include <cstddef>
#include <vector>

namespace formal_gvins::vision
{

struct OrbTrackingOptions
{
    OrbOptions orb;
    camera::Radtan<double> camera;
    camera::InverseDomain<double> inverse;
    int feature_capacity = 400;
    int grid_columns = 8;
    int grid_rows = 6;
    float ratio = 0.75F;
    double epipolar_threshold = 0.003; // Normalized coordinates, not pixels.
    double confidence = 0.99;
    int ransac_iterations = 2000;
};

struct DescriptorMatches
{
    std::vector<cv::DMatch> matches; // Reciprocal, unique query/train pairs.
    std::size_t forward_ratio_passed = 0, reverse_ratio_passed = 0;
};

/** Strict ratio in both directions plus mutual best. Ties and missing runners-up reject.
 * Malformed/duplicate query entries fail atomically; input lists may omit empty queries.
 */
[[nodiscard]] VisionStatus reciprocal_matches(std::vector<std::vector<cv::DMatch>> const & forward,
                                              std::vector<std::vector<cv::DMatch>> const & reverse,
                                              std::size_t query_count, std::size_t train_count, float ratio,
                                              DescriptorMatches & output);

struct OrbMatchResult
{
    std::vector<cv::DMatch> inliers; // query=previous, train=current
    DescriptorMatches descriptors;
    bool geometry_valid = false;
};

/** Reciprocal descriptor association followed by calibrated geometry. */
[[nodiscard]] VisionStatus verify_matches(OrbFeatures const & previous, OrbFeatures const & current,
                                          std::vector<std::vector<cv::DMatch>> const & neighbors,
                                          std::vector<std::vector<cv::DMatch>> const & reverse,
                                          OrbTrackingOptions const & options, OrbMatchResult & output);

/** Optional ORB data for inspection; keypoints/descriptors follow the observation order. */
struct OrbTrackingDiagnostics
{
    OrbFeatures features;
    OrbMatchResult matches;
};

/**
 * One camera stream, operating on raw CV_8UC1 images.
 * Failed operations preserve tracks and output. Successful empty frames retire
 * previous tracks. Geometry failure creates new IDs, never false continuations.
 */
class OrbTracker
{
public:
    explicit OrbTracker(OrbTrackingOptions const & options);
    /** Diagnostics, when requested, own their data and are unchanged on failure. */
    [[nodiscard]] VisionStatus process(ImageIdentity const & identity, cv::Mat const & image, cv::Mat const & mask,
                                       TrackedImage & output, OrbTrackingDiagnostics * diagnostics = nullptr);
    /** Retire temporal history; do not reuse track IDs. Required before changing bindings. */
    void reset();

private:
    OrbTrackingOptions m_options;
    OrbExtractor m_extractor;
    bool m_valid = false;
    bool m_have_previous = false;
    ImageIdentity m_identity;
    cv::Size m_image_size;
    OrbFeatures m_previous;
    std::vector<FeatureObservation> m_observations;
    std::uint64_t m_next_track_id = 1;
};

} /* end namespace formal_gvins::vision */
