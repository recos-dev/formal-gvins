/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/orb.hpp>
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace formal_gvins::vision
{
namespace
{
// Descriptor border 31 also covers the orientation disk and refinement gradients.
bool supported(cv::Point2f point, float scale, cv::Size level_size, cv::Size raw_size, cv::Mat const & mask_integral)
{
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 32 || point.y < 32 ||
        point.x >= level_size.width - 32 || point.y >= level_size.height - 32)
    {
        return false;
    }
    int const left = static_cast<int>(std::floor((point.x - 32) * scale));
    int const top = static_cast<int>(std::floor((point.y - 32) * scale));
    int const right = static_cast<int>(std::ceil((point.x + 32) * scale)) + 1;
    int const bottom = static_cast<int>(std::ceil((point.y + 32) * scale)) + 1;
    if (left < 0 || top < 0 || right > raw_size.width || bottom > raw_size.height)
    {
        return false;
    }
    if (mask_integral.empty())
    {
        return true;
    }
    double const sum = mask_integral.at<double>(bottom, right) - mask_integral.at<double>(top, right) -
                       mask_integral.at<double>(bottom, left) + mask_integral.at<double>(top, left);
    return sum == 0; // Integral of excluded pixels.
}

float orientation(cv::Mat const & image, cv::Point2f point)
{
    cv::Mat patch;
    cv::getRectSubPix(image, {31, 31}, point, patch, CV_32F);
    double mx = 0, my = 0;
    for (int y = -15; y <= 15; ++y)
    {
        for (int x = -15; x <= 15; ++x)
        {
            if (x * x + y * y <= 225)
            {
                float const value = patch.at<float>(y + 15, x + 15);
                mx += x * value;
                my += y * value;
            }
        }
    }
    return cv::fastAtan2(static_cast<float>(my), static_cast<float>(mx));
}
} /* end anonymous namespace */

VisionStatus localize_orb(cv::Mat const & image, cv::Mat const & mask, std::vector<cv::KeyPoint> const & detected,
                          OrbOptions const & options, std::vector<cv::KeyPoint> & output,
                          LocalizationStats & statistics)
{
    auto const & limit = options.localization;
    if (detected.size() > 10000 || image.empty() || image.type() != CV_8UC1 || image.cols > 16384 ||
        image.rows > 16384 || (!mask.empty() && (mask.type() != CV_8UC1 || mask.size() != image.size())) ||
        options.pyramid_levels < 1 || options.pyramid_levels > 16 || !std::isfinite(options.scale_factor) ||
        options.scale_factor <= 1 || options.scale_factor > 2 || limit.window_radius < 1 || limit.window_radius > 15 ||
        limit.iterations < 1 || limit.iterations > 100 || !std::isfinite(limit.epsilon) || limit.epsilon <= 0 ||
        !std::isfinite(limit.maximum_displacement) || limit.maximum_displacement <= 0 ||
        !std::isfinite(limit.minimum_eigenvalue) || limit.minimum_eigenvalue <= 0 ||
        !std::isfinite(limit.maximum_condition) || limit.maximum_condition < 1)
    {
        return VisionStatus::invalid_input;
    }
    for (auto const & key : detected)
    {
        if (key.octave < 0 || key.octave >= options.pyramid_levels)
        {
            return VisionStatus::invalid_input;
        }
    }
    try
    {
        LocalizationStats counts;
        counts.detected = detected.size();
        std::vector<cv::KeyPoint> candidate;
        cv::Mat integral;
        if (!mask.empty())
        {
            cv::Mat excluded;
            cv::compare(mask, 0, excluded, cv::CMP_EQ);
            cv::integral(excluded, integral, CV_64F);
        }
        cv::Mat previous = image;
        for (int octave = 0; octave < options.pyramid_levels; ++octave)
        {
            float const scale = std::pow(options.scale_factor, octave);
            cv::Size const size(cvRound(image.cols / scale), cvRound(image.rows / scale));
            if (size.width < 65 || size.height < 65)
            {
                for (auto const & key : detected)
                {
                    counts.support_rejected += key.octave == octave;
                }
                continue;
            }
            cv::Mat level;
            if (octave == 0)
            {
                level = image;
            }
            else
            {
                cv::resize(previous, level, size, 0, 0, cv::INTER_LINEAR_EXACT);
            }
            previous = level;
            std::vector<cv::Point2f> points;
            std::vector<std::size_t> indices;
            for (std::size_t i = 0; i < detected.size(); ++i)
            {
                auto const & key = detected[i];
                if (key.octave != octave)
                {
                    continue;
                }
                auto const point = key.pt / scale;
                if (!supported(point, scale, size, image.size(), integral) || !std::isfinite(key.response))
                {
                    ++counts.support_rejected;
                    continue;
                }
                points.push_back(point);
                indices.push_back(i);
            }
            if (points.empty())
            {
                continue;
            }
            // Observe every sampled center: a monolithic library call does not
            // expose its intermediate path through a validity mask.
            std::vector<bool> active(points.size(), true), rejected(points.size(), false);
            for (int iteration = 0; iteration < limit.iterations; ++iteration)
            {
                std::vector<cv::Point2f> step;
                std::vector<std::size_t> slots;
                for (std::size_t i = 0; i < points.size(); ++i)
                {
                    if (active[i])
                    {
                        step.push_back(points[i]);
                        slots.push_back(i);
                    }
                }
                if (step.empty())
                {
                    break;
                }
                cv::cornerSubPix(level, step, {limit.window_radius, limit.window_radius}, {-1, -1},
                                 {cv::TermCriteria::COUNT, 1, 0});
                for (std::size_t j = 0; j < step.size(); ++j)
                {
                    auto const i = slots[j];
                    if (!supported(step[j], scale, size, image.size(), integral))
                    {
                        ++counts.support_rejected;
                        rejected[i] = true;
                        active[i] = false;
                        continue;
                    }
                    if (cv::norm(step[j] * scale - detected[indices[i]].pt) > limit.maximum_displacement)
                    {
                        ++counts.displacement_rejected;
                        rejected[i] = true;
                        active[i] = false;
                        continue;
                    }
                    active[i] = cv::norm(step[j] - points[i]) > limit.epsilon;
                    points[i] = step[j];
                }
            }
            cv::Mat eigenvalues;
            cv::cornerEigenValsAndVecs(level, eigenvalues, 3, 3, cv::BORDER_REFLECT_101);
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                if (rejected[i])
                {
                    continue;
                }
                auto key = detected[indices[i]];
                auto const point = points[i];
                if (!supported(point, scale, size, image.size(), integral))
                {
                    ++counts.support_rejected;
                    continue;
                }
                if (cv::norm(point * scale - key.pt) > limit.maximum_displacement)
                {
                    ++counts.displacement_rejected;
                    continue;
                }
                auto const eigen = eigenvalues.at<cv::Vec<float, 6>>(cvRound(point.y), cvRound(point.x));
                double const low = std::min(eigen[0], eigen[1]), high = std::max(eigen[0], eigen[1]);
                if (!std::isfinite(low) || !std::isfinite(high) || low < limit.minimum_eigenvalue ||
                    high > limit.maximum_condition * low)
                {
                    ++counts.quality_rejected;
                    continue;
                }
                key.pt = point * scale;
                key.angle = orientation(level, point);
                candidate.push_back(key);
            }
        }
        output = std::move(candidate);
        statistics = counts;
        return VisionStatus::success;
    }
    catch (cv::Exception const &)
    {
        return VisionStatus::backend_failure;
    }
}
} /* end namespace formal_gvins::vision */
