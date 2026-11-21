/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/vision/orb_tracker.hpp>
#include "timing.hpp"
#include <formal_gvins/scalar/backend/standard.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace formal_gvins::vision
{
bool OrbTrackingOptions::valid() const
{
    auto const & options = *this;
    return succeeded(camera::validate_inverse<scalar::StandardMath<double>>(options.camera, options.inverse)) &&
           options.selection.valid() && options.orb.feature_count >= options.selection.capacity &&
           std::isfinite(options.ratio) && options.ratio > 0 && options.ratio < 1 &&
           std::isfinite(options.epipolar_threshold) && options.epipolar_threshold > 0 &&
           std::isfinite(options.minimum_epipolar_denominator) && options.minimum_epipolar_denominator > 0 &&
           std::isfinite(options.confidence) && options.confidence > 0 && options.confidence < 1 &&
           options.ransac_iterations > 0 && options.ransac_iterations <= 10000 && options.rotation_iterations > 0 &&
           options.rotation_iterations <= 1000 && std::isfinite(options.rotation_support_fraction) &&
           options.rotation_support_fraction >= 0.8 && options.rotation_support_fraction <= 1;
}

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
    OrbTrackingDiagnostics details;
    VisionStatus status =
        m_extractor.extract(image, mask, detected, diagnostics != nullptr ? &details.stages : nullptr);
    if (status != VisionStatus::success)
    {
        return status;
    }
    try
    {
        TrackedImage candidate;
        candidate.identity = identity;
        candidate.pixel_domain = PixelDomain::raw;
        StageTimer timer(diagnostics != nullptr ? &details.stages : nullptr, FrontendStage::unprojection);
        details.features.localization = detected.localization;
        for (std::size_t i = 0; i < detected.keypoints.size(); ++i)
        {
            auto const & key = detected.keypoints[i];
            camera::Bearing<double> bearing;
            if (!succeeded(camera::unproject<scalar::StandardMath<double>>(key.pt.x, key.pt.y, m_options.camera,
                                                                           m_options.inverse, bearing)))
            {
                ++details.inverse_rejected;
                continue;
            }
            details.features.keypoints.push_back(key);
            details.features.descriptors.push_back(detected.descriptors.row(static_cast<int>(i)));
            candidate.observations.push_back({0, key.pt.x, key.pt.y, bearing.x, bearing.y});
        }
        details.ages.resize(candidate.observations.size(), 0);
        if (m_have_previous)
        {
            timer.next(FrontendStage::matching);
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
            timer.next(FrontendStage::geometry);
            status = verify_matches(m_previous, details.features, neighbors, reverse, m_options, details.matches);
            if (status != VisionStatus::success)
            {
                return status;
            }
            for (auto const & match : details.matches.inliers)
            {
                auto const query = static_cast<std::size_t>(match.queryIdx),
                           train = static_cast<std::size_t>(match.trainIdx);
                if (m_ages[query] == std::numeric_limits<std::uint64_t>::max())
                {
                    return VisionStatus::invalid_input;
                }
                candidate.observations[train].track_id = m_observations[query].track_id;
                details.ages[train] = m_ages[query] + 1;
            }
        }
        timer.next(FrontendStage::selection);
        details.reciprocal_candidates = details.matches.descriptors.matches.size();
        details.geometric_candidates = details.matches.inliers.size();
        std::vector<SelectionCandidate> priorities;
        for (std::size_t i = 0; i < candidate.observations.size(); ++i)
        {
            auto const & observation = candidate.observations[i];
            priorities.push_back({observation.u, observation.v, details.features.keypoints[i].response,
                                  observation.track_id, details.ages[i]});
        }
        SelectionResult selection;
        status = select_features(priorities, image.cols, image.rows, m_options.selection, selection);
        if (status != VisionStatus::success)
        {
            return status;
        }
        details.selection = selection.stats;
        OrbFeatures selected;
        selected.localization = details.features.localization;
        std::vector<FeatureObservation> observations;
        std::vector<std::uint64_t> ages;
        std::vector<int> remap(candidate.observations.size(), -1);
        for (auto index : selection.indices)
        {
            remap[index] = static_cast<int>(observations.size());
            observations.push_back(candidate.observations[index]);
            ages.push_back(details.ages[index]);
            selected.keypoints.push_back(details.features.keypoints[index]);
            selected.descriptors.push_back(details.features.descriptors.row(static_cast<int>(index)));
        }
        for (auto * matches : {&details.matches.inliers, &details.matches.descriptors.matches})
        {
            matches->erase(std::remove_if(matches->begin(), matches->end(),
                                          [&](cv::DMatch const & m)
                                          { return remap[static_cast<std::size_t>(m.trainIdx)] < 0; }),
                           matches->end());
            for (auto & match : *matches)
            {
                match.trainIdx = remap[static_cast<std::size_t>(match.trainIdx)];
            }
        }
        candidate.observations = std::move(observations);
        details.features = std::move(selected);
        details.ages = std::move(ages);
        timer.next(FrontendStage::publication);
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
            if (details.ages[index] == 0)
            {
                details.ages[index] = 1;
            }
        }
        std::vector<std::uint64_t> active_ids;
        for (std::size_t i = 0; i < candidate.observations.size(); ++i)
        {
            auto const id = candidate.observations[i].track_id;
            active_ids.push_back(id);
            candidate.events.push_back({id, details.ages[i] == 1 ? TrackChange::started : TrackChange::continued});
        }
        std::sort(active_ids.begin(), active_ids.end());
        for (auto const & previous : m_observations)
        {
            if (!std::binary_search(active_ids.begin(), active_ids.end(), previous.track_id))
            {
                candidate.events.push_back({previous.track_id, TrackChange::ended});
            }
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
        observations = candidate.observations;
        ages = details.ages;
        timer.finish();
        m_previous = std::move(history);
        m_observations = std::move(observations);
        m_ages = std::move(ages);
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
    m_ages.clear();
    m_have_previous = false;
}

} /* end namespace formal_gvins::vision */
