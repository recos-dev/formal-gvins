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

struct OrbOptions
{
    Execution execution = Execution::cpu;
    int feature_count = 1000;
    float scale_factor = 1.2F;
    int pyramid_levels = 8;
    int fast_threshold = 20;
};

/** Owning keypoints and corresponding WTA_K=2 descriptor rows. */
struct OrbFeatures
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

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
    cv::Ptr<cv::ORB> m_cpu;
#if defined(FORMAL_GVINS_FRONTEND_CUDA)
    cv::Ptr<cv::cuda::ORB> m_gpu;
    cv::Ptr<cv::cuda::DescriptorMatcher> m_matcher;
    cv::Ptr<cv::cuda::Stream> m_stream;
#endif
};

} /* end namespace formal_gvins::vision */
