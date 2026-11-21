/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <formal_gvins/vision/tracker.hpp>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#if defined(FORMAL_GVINS_FRONTEND_CUDA)
#include <opencv2/cudafeatures2d.hpp>
#endif

namespace formal_gvins::vision
{

struct LocalizationOptions
{
    int window_radius = 5;
    int iterations = 20;
    double epsilon = 0.001;            // Pyramid pixels; a stopping tolerance, not an error bound.
    double maximum_displacement = 2.0; // Original-image pixels, at every level.
    double minimum_eigenvalue = 1e-4;  // OpenCV normalized gradient measure.
    double maximum_condition = 100;
};

struct LocalizationStats
{
    std::size_t detected = 0, support_rejected = 0, displacement_rejected = 0, quality_rejected = 0;
};

struct OrbOptions
{
    Execution execution = Execution::cpu;
    int feature_count = 1000;
    float scale_factor = 1.2F;
    int pyramid_levels = 8;
    int fast_threshold = 20;
    LocalizationOptions localization;
};

/** Owning keypoints and corresponding WTA_K=2 descriptor rows. */
struct OrbFeatures
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    LocalizationStats localization;
};

/** Refine in each octave, check raw-image displacement/support and refresh orientation.
 * Points are returned in octave order; no descriptors exist yet. Failure preserves both outputs.
 */
[[nodiscard]] VisionStatus localize_orb(cv::Mat const & image, cv::Mat const & mask,
                                        std::vector<cv::KeyPoint> const & detected, OrbOptions const & options,
                                        std::vector<cv::KeyPoint> & output, LocalizationStats & statistics);

/**
 * Blocking, single-owner image adapter. Do not call concurrently on one instance.
 * CUDA operations finish before return; returned descriptors own host storage.
 * OpenCV types never enter the numerical core. No automatic CPU fallback.
 */
class OrbExtractor
{
public:
    explicit OrbExtractor(OrbOptions const & options = {});
    OrbExtractor(OrbExtractor const &) = delete;
    OrbExtractor & operator=(OrbExtractor const &) = delete;

    /** CV_8UC1 image, optional same-size CV_8UC1 mask. Failure preserves output. */
    [[nodiscard]] VisionStatus extract(cv::Mat const & image, cv::Mat const & mask, OrbFeatures & output);

    /** Two-nearest Hamming candidates. Fewer than two train rows yields no candidates. */
    [[nodiscard]] VisionStatus match(OrbFeatures const & query, OrbFeatures const & train,
                                     std::vector<std::vector<cv::DMatch>> & output);

private:
    VisionStatus m_status = VisionStatus::invalid_input;
    Execution m_execution = Execution::cpu;
    OrbOptions m_options;
    cv::Ptr<cv::ORB> m_cpu;
#if defined(FORMAL_GVINS_FRONTEND_CUDA)
    cv::Ptr<cv::cuda::ORB> m_gpu;
    cv::Ptr<cv::cuda::DescriptorMatcher> m_matcher;
    cv::Ptr<cv::cuda::Stream> m_stream;
#endif
};

} /* end namespace formal_gvins::vision */
