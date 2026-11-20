/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <formal_gvins/vision/observation.hpp>
#include <formal_gvins/vision/orb.hpp>

namespace formal_gvins::vision
{

struct TrackingOptions
{
    OrbOptions orb;
    cv::Matx33d camera_matrix = cv::Matx33d::zeros(); // Required K for undistorted pixels, zero skew
    int feature_capacity = 400;
    int grid_columns = 8;
    int grid_rows = 6;
    float ratio = 0.75F;
    double epipolar_threshold_px = 1.5;
    double confidence = 0.99;
    int ransac_iterations = 2000;
};

struct MatchResult
{
    std::vector<cv::DMatch> inliers; // query=previous, train=current
    std::size_t ratio_passed = 0;
    bool geometry_valid = false;
};

/** Strict ratio, one-to-one selection, then calibrated five-point RANSAC; at least eight candidates. */
[[nodiscard]] VisionStatus verify_matches(OrbFeatures const & previous, OrbFeatures const & current,
                                          std::vector<std::vector<cv::DMatch>> const & neighbors,
                                          TrackingOptions const & options, MatchResult & output);

struct TrackedImage
{
    ImageIdentity identity;
    OrbFeatures features;
    std::vector<FeatureObservation> observations; // same order as features
    MatchResult matches;
};

/**
 * One camera stream, operating on undistorted CV_8UC1 images.
 * Failed operations preserve tracks and output. Successful empty frames retire
 * previous tracks. Geometry failure creates new IDs, never false continuations.
 */
class OrbTracker
{
public:
    explicit OrbTracker(TrackingOptions const & options);
    [[nodiscard]] VisionStatus process(ImageIdentity const & identity, cv::Mat const & image, cv::Mat const & mask,
                                       TrackedImage & output);
    /** Retire temporal history; do not reuse track IDs. Required before changing bindings. */
    void reset();

private:
    TrackingOptions m_options;
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
