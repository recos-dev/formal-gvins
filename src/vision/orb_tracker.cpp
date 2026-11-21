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

namespace formal_gvins::vision
{
bool OrbTrackingOptions::valid() const
{
    auto const & options = *this;
    return succeeded(camera::validate_inverse<scalar::StandardMath<double>>(options.camera, options.inverse)) &&
           options.feature_capacity > 0 && options.feature_capacity <= 10000 && options.grid_columns > 0 &&
           options.grid_columns <= 64 && options.grid_rows > 0 && options.grid_rows <= 64 &&
           options.feature_capacity >= options.grid_columns * options.grid_rows &&
           options.orb.feature_count >= options.feature_capacity && std::isfinite(options.ratio) && options.ratio > 0 &&
           options.ratio < 1 && std::isfinite(options.epipolar_threshold) && options.epipolar_threshold > 0 &&
           std::isfinite(options.minimum_epipolar_denominator) && options.minimum_epipolar_denominator > 0 &&
           std::isfinite(options.confidence) && options.confidence > 0 && options.confidence < 1 &&
           options.ransac_iterations > 0 && options.ransac_iterations <= 10000 && options.rotation_iterations > 0 &&
           options.rotation_iterations <= 1000 && std::isfinite(options.rotation_support_fraction) &&
           options.rotation_support_fraction >= 0.8 && options.rotation_support_fraction <= 1;
}

namespace
{

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
    result.localization = input.localization;
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

OrbTracker::OrbTracker(OrbTrackingOptions const & options)
    : m_options(options), m_extractor(options.orb), m_valid(options.valid())
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
            std::vector<std::vector<cv::DMatch>> reverse;
            status = m_extractor.match(details.features, m_previous, reverse);
            if (status != VisionStatus::success)
            {
                return status;
            }
            status = verify_matches(m_previous, details.features, neighbors, reverse, m_options, details.matches);
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
            history = {details.features.keypoints, details.features.descriptors.clone(), details.features.localization};
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
