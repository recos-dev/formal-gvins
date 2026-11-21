/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/vision/orb.hpp>
#include "timing.hpp"
#include <cmath>
#include <utility>

namespace formal_gvins::vision
{

OrbExtractor::OrbExtractor(OrbOptions const & options) : m_execution(options.execution), m_options(options)
{
    if ((m_execution != Execution::cpu && m_execution != Execution::cuda) || options.feature_count <= 0 ||
        options.feature_count > 10000 || !std::isfinite(options.scale_factor) || options.scale_factor <= 1.0F ||
        options.scale_factor > 2.0F || options.pyramid_levels <= 0 || options.pyramid_levels > 16 ||
        options.fast_threshold < 0 || options.fast_threshold > 255)
    {
        return;
    }
    try
    {
        if (m_execution == Execution::cpu)
        {
            m_cpu = cv::ORB::create(options.feature_count, options.scale_factor, options.pyramid_levels, 31, 0, 2,
                                    cv::ORB::HARRIS_SCORE, 31, options.fast_threshold);
        }
        else
        {
#if defined(FORMAL_GVINS_FRONTEND_CUDA)
            if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
            {
                m_status = VisionStatus::backend_unavailable;
                return;
            }
            m_gpu = cv::cuda::ORB::create(options.feature_count, options.scale_factor, options.pyramid_levels, 31, 0, 2,
                                          cv::ORB::HARRIS_SCORE, 31, options.fast_threshold, true);
            m_matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
            m_stream = cv::makePtr<cv::cuda::Stream>();
#else
            m_status = VisionStatus::backend_unavailable;
            return;
#endif
        }
        m_status = VisionStatus::success;
    }
    catch (cv::Exception const &)
    {
        m_status = VisionStatus::backend_failure;
    }
}

VisionStatus OrbExtractor::extract(cv::Mat const & image, cv::Mat const & mask, OrbFeatures & output,
                                   FrontendMetrics * metrics)
{
    if (m_status != VisionStatus::success)
    {
        return m_status;
    }
    if (image.empty() || image.type() != CV_8UC1 ||
        (!mask.empty() && (mask.type() != CV_8UC1 || mask.size() != image.size())))
    {
        return VisionStatus::invalid_input;
    }
    try
    {
        OrbFeatures candidate;
        FrontendMetrics measured{};
        StageTimer timer(metrics != nullptr ? &measured : nullptr, FrontendStage::detection);
        if (m_execution == Execution::cpu)
        {
            m_cpu->detect(image, candidate.keypoints, mask);
        }
#if defined(FORMAL_GVINS_FRONTEND_CUDA)
        else
        {
            cv::cuda::GpuMat device_image, device_mask, device_keypoints;
            device_image.upload(image, *m_stream);
            if (!mask.empty())
            {
                device_mask.upload(mask, *m_stream);
            }
            m_gpu->detectAsync(device_image, device_keypoints, device_mask, *m_stream);
            m_stream->waitForCompletion();
            m_gpu->convert(device_keypoints, candidate.keypoints);
        }
#endif
        timer.next(FrontendStage::localization);
        auto const status =
            localize_orb(image, mask, candidate.keypoints, m_options, candidate.keypoints, candidate.localization);
        if (status != VisionStatus::success)
        {
            return status;
        }
        timer.next(FrontendStage::description);
        if (!candidate.keypoints.empty())
        {
            if (m_execution == Execution::cpu)
            {
                m_cpu->compute(image, candidate.keypoints, candidate.descriptors);
            }
#if defined(FORMAL_GVINS_FRONTEND_CUDA)
            else
            {
                // The provided-keypoint adapter groups by octave and retains per-level
                // counts internally. A fresh descriptor instance prevents stale counts
                // when accepted octaves have gaps; the detector keeps its configured levels.
                auto descriptor =
                    cv::cuda::ORB::create(m_options.feature_count, m_options.scale_factor, m_options.pyramid_levels, 31,
                                          0, 2, cv::ORB::HARRIS_SCORE, 31, m_options.fast_threshold, true);
                cv::cuda::GpuMat device_image(image), device_descriptors;
                descriptor->compute(device_image, candidate.keypoints, device_descriptors);
                device_descriptors.download(candidate.descriptors);
            }
#endif
        }
        if (candidate.descriptors.rows != static_cast<int>(candidate.keypoints.size()))
        {
            return VisionStatus::backend_failure;
        }
        timer.finish();
        if (metrics != nullptr)
        {
            *metrics = measured;
        }
        output = std::move(candidate);
        return VisionStatus::success;
    }
    catch (cv::Exception const &)
    {
        return VisionStatus::backend_failure;
    }
}

VisionStatus OrbExtractor::match(OrbFeatures const & query, OrbFeatures const & train,
                                 std::vector<std::vector<cv::DMatch>> & output)
{
    if (m_status != VisionStatus::success)
    {
        return m_status;
    }
    for (auto const * features : {&query, &train})
    {
        if (features->descriptors.rows != static_cast<int>(features->keypoints.size()) ||
            (!features->descriptors.empty() &&
             (features->descriptors.type() != CV_8UC1 || features->descriptors.cols != 32)))
        {
            return VisionStatus::invalid_input;
        }
    }
    try
    {
        std::vector<std::vector<cv::DMatch>> candidate;
        if (!query.keypoints.empty() && train.keypoints.size() >= 2U)
        {
            if (m_execution == Execution::cpu)
            {
                cv::BFMatcher(cv::NORM_HAMMING).knnMatch(query.descriptors, train.descriptors, candidate, 2);
            }
#if defined(FORMAL_GVINS_FRONTEND_CUDA)
            else
            {
                cv::cuda::GpuMat device_query, device_train, device_matches;
                device_query.upload(query.descriptors, *m_stream);
                device_train.upload(train.descriptors, *m_stream);
                m_matcher->knnMatchAsync(device_query, device_train, device_matches, 2, cv::noArray(), *m_stream);
                m_stream->waitForCompletion();
                m_matcher->knnMatchConvert(device_matches, candidate);
            }
#endif
        }
        output = std::move(candidate);
        return VisionStatus::success;
    }
    catch (cv::Exception const &)
    {
        return VisionStatus::backend_failure;
    }
}

} /* end namespace formal_gvins::vision */
