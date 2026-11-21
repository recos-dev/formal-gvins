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
#include <formal_gvins/vision/selection.hpp>
#include <cstddef>
#include <span>
#include <vector>

namespace formal_gvins::vision
{

struct OrbTrackingOptions
{
    OrbOptions orb;
    camera::Radtan<double> camera;
    camera::InverseDomain<double> inverse;
    SelectionOptions selection;
    float ratio = 0.75F;
    double epipolar_threshold = 0.003;           // Normalized coordinates, not pixels.
    double minimum_epipolar_denominator = 1e-12; // E has unit Frobenius norm.
    double confidence = 0.99;
    int ransac_iterations = 2000;
    int rotation_iterations = 100;
    double rotation_support_fraction = 0.9; // Additional screen only when E inliers support rotation.

    [[nodiscard]] bool valid() const;
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
    bool rotation_checked = false;
    bool geometry_valid = false;
};

/** Reciprocal association and temporal geometry using already admitted normalized rays.
 * The producer owns raw/bearing pairing, inverse-domain and calibration identity checks.
 * Malformed matching indices or nonfinite referenced rays fail atomically.
 */
[[nodiscard]] VisionStatus verify_temporal(std::span<FeatureObservation const> previous,
                                           std::span<FeatureObservation const> current,
                                           std::vector<std::vector<cv::DMatch>> const & neighbors,
                                           std::vector<std::vector<cv::DMatch>> const & reverse,
                                           OrbTrackingOptions const & options, OrbMatchResult & output);

/** Optional ORB data for inspection; keypoints/descriptors follow the observation order. */
struct OrbTrackingDiagnostics
{
    OrbFeatures features;
    FrontendMetrics stages{};
    OrbMatchResult matches;
    SelectionStats selection;
    std::vector<std::uint64_t> ages;
    std::size_t inverse_rejected = 0;
    std::size_t reciprocal_candidates = 0, geometric_candidates = 0;
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
    friend class StereoOrbTracker;
    struct PendingFrame
    {
        TrackedImage tracked;
        OrbTrackingDiagnostics diagnostics;
        cv::Size image_size;
        OrbFeatures history;
        std::vector<FeatureObservation> observations;
        std::vector<std::uint64_t> ages;
    };
    [[nodiscard]] VisionStatus prepare(ImageIdentity const & identity, cv::Mat const & image, cv::Mat const & mask,
                                       PendingFrame & pending, bool inspect);
    [[nodiscard]] VisionStatus finish(PendingFrame & pending, std::uint64_t & next_id, bool inspect);
    void commit(PendingFrame & pending, TrackedImage & output, OrbTrackingDiagnostics * diagnostics);

    OrbTrackingOptions m_options;
    OrbExtractor m_extractor;
    bool m_valid = false;
    bool m_have_previous = false;
    ImageIdentity m_identity;
    cv::Size m_image_size;
    OrbFeatures m_previous;
    std::vector<FeatureObservation> m_observations;
    std::vector<std::uint64_t> m_ages;
    std::uint64_t m_next_track_id = 1;
};

} /* end namespace formal_gvins::vision */
