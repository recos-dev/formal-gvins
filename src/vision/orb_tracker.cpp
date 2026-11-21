/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/vision/orb_tracker.hpp>
#include <formal_gvins/scalar/backend/standard.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <opencv2/calib3d.hpp>

namespace formal_gvins::vision
{
namespace
{

bool valid(OrbTrackingOptions const & options)
{
    return succeeded(camera::validate_inverse<scalar::StandardMath<double>>(options.camera, options.inverse)) &&
           options.feature_capacity > 0 && options.feature_capacity <= 10000 && options.grid_columns > 0 &&
           options.grid_columns <= 64 && options.grid_rows > 0 && options.grid_rows <= 64 &&
           options.feature_capacity >= options.grid_columns * options.grid_rows &&
           options.orb.feature_count >= options.feature_capacity && std::isfinite(options.ratio) && options.ratio > 0 &&
           options.ratio < 1 && std::isfinite(options.epipolar_threshold) && options.epipolar_threshold > 0 &&
           std::isfinite(options.confidence) && options.confidence > 0 && options.confidence < 1 &&
           options.ransac_iterations > 0 && options.ransac_iterations <= 10000;
}

OrbFeatures spatial_selection(OrbFeatures const & input, cv::Size size, cv::Mat const & mask,
                              OrbTrackingOptions const & options)
{
    std::vector<std::size_t> order(input.keypoints.size());
    std::iota(order.begin(), order.end(), 0);
    // Filter before sorting so a non-finite response cannot violate strict ordering.
    order.erase(std::remove_if(order.begin(), order.end(),
                               [&](std::size_t index)
                               {
                                   auto const & key = input.keypoints[index];
                                   return !std::isfinite(key.response) || !std::isfinite(key.pt.x) ||
                                          !std::isfinite(key.pt.y) || key.pt.x < 0 || key.pt.y < 0 ||
                                          key.pt.x >= size.width || key.pt.y >= size.height ||
                                          (!mask.empty() && mask.at<unsigned char>(static_cast<int>(key.pt.y),
                                                                                   static_cast<int>(key.pt.x)) == 0);
                               }),
                order.end());
    std::stable_sort(order.begin(), order.end(),
                     [&](std::size_t a, std::size_t b)
                     { return input.keypoints[a].response > input.keypoints[b].response; });
    int const cells = options.grid_columns * options.grid_rows;
    std::vector<int> occupancy(static_cast<std::size_t>(cells), 0);
    OrbFeatures result;
    for (std::size_t index : order)
    {
        auto const & key = input.keypoints[index];
        int const column = static_cast<int>(static_cast<double>(key.pt.x) * options.grid_columns / size.width);
        int const row = static_cast<int>(static_cast<double>(key.pt.y) * options.grid_rows / size.height);
        int const cell = row * options.grid_columns + column;
        int const quota = options.feature_capacity / cells + (cell < options.feature_capacity % cells ? 1 : 0);
        if (occupancy[static_cast<std::size_t>(cell)] < quota)
        {
            camera::Bearing<double> bearing;
            if (!succeeded(camera::unproject<scalar::StandardMath<double>>(key.pt.x, key.pt.y, options.camera,
                                                                           options.inverse, bearing)))
            {
                continue;
            }
            ++occupancy[static_cast<std::size_t>(cell)];
            result.keypoints.push_back(key);
            result.descriptors.push_back(input.descriptors.row(static_cast<int>(index)));
        }
    }
    return result;
}

} /* end anonymous namespace */

VisionStatus verify_matches(OrbFeatures const & previous, OrbFeatures const & current,
                            std::vector<std::vector<cv::DMatch>> const & neighbors, OrbTrackingOptions const & options,
                            OrbMatchResult & output)
{
    if (!valid(options))
    {
        return VisionStatus::invalid_input;
    }
    OrbMatchResult candidate;
    std::vector<cv::DMatch> proposals;
    for (auto const & pair : neighbors)
    {
        if (pair.size() < 2)
        {
            continue;
        }
        auto const & first = pair[0];
        auto const & second = pair[1];
        if (first.queryIdx < 0 || first.trainIdx < 0 || second.trainIdx < 0 ||
            static_cast<std::size_t>(first.queryIdx) >= previous.keypoints.size() ||
            static_cast<std::size_t>(first.trainIdx) >= current.keypoints.size() ||
            static_cast<std::size_t>(second.trainIdx) >= current.keypoints.size() ||
            second.queryIdx != first.queryIdx || second.trainIdx == first.trainIdx || !std::isfinite(first.distance) ||
            !std::isfinite(second.distance) || first.distance < 0 || second.distance < first.distance)
        {
            return VisionStatus::invalid_input;
        }
        if (first.distance < options.ratio * second.distance)
        {
            proposals.push_back(first);
        }
    }
    candidate.ratio_passed = proposals.size();
    std::sort(proposals.begin(), proposals.end(),
              [](cv::DMatch const & a, cv::DMatch const & b)
              {
                  if (a.distance != b.distance)
                  {
                      return a.distance < b.distance;
                  }
                  if (a.queryIdx != b.queryIdx)
                  {
                      return a.queryIdx < b.queryIdx;
                  }
                  return a.trainIdx < b.trainIdx;
              });
    std::vector<bool> used_previous(previous.keypoints.size(), false), used_current(current.keypoints.size(), false);
    std::vector<cv::DMatch> unique;
    std::vector<cv::Point2f> points_previous, points_current;
    for (auto const & match : proposals)
    {
        auto const query = static_cast<std::size_t>(match.queryIdx);
        auto const train = static_cast<std::size_t>(match.trainIdx);
        if (used_previous[query] || used_current[train])
        {
            continue;
        }
        auto const p = previous.keypoints[query].pt;
        auto const q = current.keypoints[train].pt;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(q.x) || !std::isfinite(q.y))
        {
            return VisionStatus::invalid_input;
        }
        used_previous[query] = used_current[train] = true;
        camera::Bearing<double> a, b;
        if (!succeeded(camera::unproject<scalar::StandardMath<double>>(p.x, p.y, options.camera, options.inverse, a)) ||
            !succeeded(camera::unproject<scalar::StandardMath<double>>(q.x, q.y, options.camera, options.inverse, b)))
        {
            continue;
        }
        unique.push_back(match);
        points_previous.emplace_back(a.x, a.y);
        points_current.emplace_back(b.x, b.y);
    }
    try
    {
        // Known intrinsics constrain the model to E = [t]_x R. Require more
        // candidates than the five-point minimal sample before confirming tracks.
        if (unique.size() >= 8U)
        {
            cv::Mat mask;
            cv::Mat const E =
                cv::findEssentialMat(points_previous, points_current, cv::Matx33d::eye(), cv::RANSAC,
                                     options.confidence, options.epipolar_threshold, options.ransac_iterations, mask);
            if (E.rows == 3 && E.cols == 3 && cv::checkRange(E) && cv::norm(E) > 0 && mask.total() == unique.size())
            {
                for (std::size_t index = 0; index < unique.size(); ++index)
                {
                    if (mask.ptr<unsigned char>()[index] != 0)
                    {
                        candidate.inliers.push_back(unique[index]);
                    }
                }
                candidate.geometry_valid = candidate.inliers.size() >= 8U;
                if (!candidate.geometry_valid)
                {
                    candidate.inliers.clear();
                }
            }
        }
        output = std::move(candidate);
        return VisionStatus::success;
    }
    catch (cv::Exception const &)
    {
        return VisionStatus::backend_failure;
    }
}

OrbTracker::OrbTracker(OrbTrackingOptions const & options)
    : m_options(options), m_extractor(options.orb), m_valid(valid(options))
{
}

VisionStatus OrbTracker::process(ImageIdentity const & identity, cv::Mat const & image, cv::Mat const & mask,
                                 TrackedImage & output, OrbTrackingDiagnostics * diagnostics)
{
    if (!m_valid || identity.session_id == 0 || identity.calibration_id == 0 || identity.noise_model_id == 0 ||
        (m_have_previous &&
         (identity.session_id != m_identity.session_id || identity.camera_id != m_identity.camera_id ||
          identity.calibration_id != m_identity.calibration_id ||
          identity.noise_model_id != m_identity.noise_model_id || identity.image_id <= m_identity.image_id ||
          identity.time_ns <= m_identity.time_ns || image.size() != m_image_size)))
    {
        return VisionStatus::invalid_input;
    }
    OrbFeatures detected;
    VisionStatus status = m_extractor.extract(image, mask, detected);
    if (status != VisionStatus::success)
    {
        return status;
    }
    try
    {
        TrackedImage candidate;
        candidate.identity = identity;
        candidate.pixel_domain = PixelDomain::raw;
        OrbTrackingDiagnostics details;
        details.features = spatial_selection(detected, image.size(), mask, m_options);
        candidate.observations.resize(details.features.keypoints.size());
        if (m_have_previous)
        {
            std::vector<std::vector<cv::DMatch>> neighbors;
            status = m_extractor.match(m_previous, details.features, neighbors);
            if (status != VisionStatus::success)
            {
                return status;
            }
            status = verify_matches(m_previous, details.features, neighbors, m_options, details.matches);
            if (status != VisionStatus::success)
            {
                return status;
            }
            for (auto const & match : details.matches.inliers)
            {
                candidate.observations[static_cast<std::size_t>(match.trainIdx)].track_id =
                    m_observations[static_cast<std::size_t>(match.queryIdx)].track_id;
            }
        }
        auto next_id = m_next_track_id;
        for (std::size_t index = 0; index < candidate.observations.size(); ++index)
        {
            auto & observation = candidate.observations[index];
            if (observation.track_id == 0)
            {
                if (next_id == std::numeric_limits<std::uint64_t>::max())
                {
                    return VisionStatus::invalid_input;
                }
                observation.track_id = next_id++;
            }
            observation.u = details.features.keypoints[index].pt.x;
            observation.v = details.features.keypoints[index].pt.y;
            camera::Bearing<double> bearing;
            if (!succeeded(camera::unproject<scalar::StandardMath<double>>(
                    observation.u, observation.v, m_options.camera, m_options.inverse, bearing)))
            {
                return VisionStatus::backend_failure;
            }
            observation.x = bearing.x;
            observation.y = bearing.y;
        }
        // Complete allocations before committing history. Caller-owned results
        // cannot mutate descriptors retained for the next temporal match.
        OrbFeatures history;
        if (diagnostics != nullptr)
        {
            history = {details.features.keypoints, details.features.descriptors.clone()};
        }
        else
        {
            history = std::move(details.features);
        }
        auto observations = candidate.observations;
        m_previous = std::move(history);
        m_observations = std::move(observations);
        m_identity = identity;
        m_image_size = image.size();
        m_next_track_id = next_id;
        m_have_previous = true;
        output = std::move(candidate);
        if (diagnostics != nullptr)
        {
            *diagnostics = std::move(details);
        }
        return VisionStatus::success;
    }
    catch (cv::Exception const &)
    {
        return VisionStatus::backend_failure;
    }
}

void OrbTracker::reset()
{
    m_previous = {};
    m_observations.clear();
    m_have_previous = false;
}

} /* end namespace formal_gvins::vision */
