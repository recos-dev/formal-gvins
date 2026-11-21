/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/vision/stereo_tracker.hpp>
#include <formal_gvins/vision/epipolar.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace formal_gvins::vision
{
bool StereoRig::essential(cv::Matx33d & E) const
{
    if (!std::isfinite(epipolar_threshold) || epipolar_threshold <= 0 || !std::isfinite(minimum_epipolar_denominator) ||
        minimum_epipolar_denominator <= 0)
    {
        return false;
    }
    for (auto value : R_RL.val)
    {
        if (!std::isfinite(value) || std::abs(value) > 1 + 1e-8)
        {
            return false;
        }
    }
    double scale = 0;
    for (auto value : t_RL.val)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
        scale = std::max(scale, std::abs(value));
    }
    if (scale == 0 || cv::norm(R_RL.t() * R_RL - cv::Matx33d::eye(), cv::NORM_INF) > 1e-8 ||
        std::abs(cv::determinant(R_RL) - 1) > 1e-8)
    {
        return false;
    }
    // Epipolar scale is arbitrary. Normalize before multiplication to avoid overflow.
    cv::Vec3d const t{t_RL[0] / scale, t_RL[1] / scale, t_RL[2] / scale};
    cv::Matx33d candidate = cv::Matx33d(0, -t[2], t[1], t[2], 0, -t[0], -t[1], t[0], 0) * R_RL;
    if (!normalize_epipolar(candidate))
    {
        return false;
    }
    E = candidate;
    return true;
}

VisionStatus verify_stereo(std::span<FeatureObservation const> left, std::span<FeatureObservation const> right,
                           std::vector<std::vector<cv::DMatch>> const & forward,
                           std::vector<std::vector<cv::DMatch>> const & reverse, StereoRig const & rig, float ratio,
                           StereoMatchResult & output)
{
    cv::Matx33d E;
    if (!rig.essential(E))
    {
        return VisionStatus::invalid_input;
    }
    StereoMatchResult candidate;
    auto const status = reciprocal_matches(forward, reverse, left.size(), right.size(), ratio, candidate.descriptors);
    if (status != VisionStatus::success)
    {
        return status;
    }
    for (auto const & match : candidate.descriptors.matches)
    {
        auto const & a = left[static_cast<std::size_t>(match.queryIdx)];
        auto const & b = right[static_cast<std::size_t>(match.trainIdx)];
        if (epipolar_inlier(E, {a.x, a.y}, {b.x, b.y}, rig.epipolar_threshold, rig.minimum_epipolar_denominator))
        {
            candidate.inliers.push_back(match);
        }
        else
        {
            ++candidate.geometry_rejected;
        }
    }
    output = std::move(candidate);
    return VisionStatus::success;
}

StereoOrbTracker::StereoOrbTracker(StereoTrackingOptions const & options)
    : m_options(options), m_trackers{OrbTracker(options.cameras[0]), OrbTracker(options.cameras[1])}
{
    cv::Matx33d E;
    m_valid = options.rig.essential(E) && std::isfinite(options.ratio) && options.ratio > 0 && options.ratio < 1;
}

VisionStatus StereoOrbTracker::associate(std::array<OrbTracker::PendingFrame, 2> & pending,
                                         StereoTrackingDiagnostics & diagnostics, std::uint64_t & next_id)
{
    auto const time_left = pending[0].tracked.identity.time_ns, time_right = pending[1].tracked.identity.time_ns;
    auto const skew = time_left > time_right ? time_left - time_right : time_right - time_left;
    if (skew > m_options.rig.maximum_skew_ns)
    {
        diagnostics.association = StereoStatus::unsynchronized;
        return VisionStatus::success;
    }
    std::vector<std::vector<cv::DMatch>> forward, reverse;
    auto & extractor = m_trackers[0].m_extractor;
    auto const & a = pending[0].diagnostics.features;
    auto const & b = pending[1].diagnostics.features;
    auto status = extractor.match(a, b, forward);
    if (status == VisionStatus::success)
    {
        status = extractor.match(b, a, reverse);
    }
    if (status != VisionStatus::success)
    {
        diagnostics.association = StereoStatus::backend_failure;
        return VisionStatus::success; // Mono preparation already succeeded; stereo supplies no link.
    }
    auto & left = pending[0].tracked.observations;
    auto & right = pending[1].tracked.observations;
    status = verify_stereo(left, right, forward, reverse, m_options.rig, m_options.ratio, diagnostics.matches);
    if (status != VisionStatus::success)
    {
        return status;
    }
    std::vector<cv::DMatch> accepted;
    for (auto const & match : diagnostics.matches.inliers)
    {
        auto & left_id = left[static_cast<std::size_t>(match.queryIdx)].track_id;
        auto & right_id = right[static_cast<std::size_t>(match.trainIdx)].track_id;
        // Never rename two established tracks or duplicate an ID inside either camera.
        bool const conflict =
            (left_id != 0 && right_id != 0 && left_id != right_id) ||
            (left_id == 0 && right_id != 0 &&
             std::any_of(left.begin(), left.end(), [&](auto const & o) { return o.track_id == right_id; })) ||
            (right_id == 0 && left_id != 0 &&
             std::any_of(right.begin(), right.end(), [&](auto const & o) { return o.track_id == left_id; }));
        if (conflict)
        {
            ++diagnostics.identity_conflicts;
            continue;
        }
        auto shared_id = left_id != 0 ? left_id : right_id;
        if (shared_id == 0)
        {
            if (next_id == std::numeric_limits<std::uint64_t>::max())
            {
                return VisionStatus::invalid_input;
            }
            shared_id = next_id++;
        }
        left_id = shared_id;
        right_id = shared_id;
        accepted.push_back(match);
    }
    diagnostics.matches.inliers = std::move(accepted);
    diagnostics.association = diagnostics.matches.inliers.empty() ? StereoStatus::no_matches : StereoStatus::accepted;
    return VisionStatus::success;
}

VisionStatus StereoOrbTracker::process(std::array<ImageIdentity, 2> const & identity,
                                       std::array<cv::Mat, 2> const & images, std::array<cv::Mat, 2> const & masks,
                                       std::array<TrackedImage, 2> & output, StereoTrackingDiagnostics * diagnostics)
{
    if (!m_valid || identity[0].session_id != identity[1].session_id || identity[0].camera_id == identity[1].camera_id)
    {
        return VisionStatus::invalid_input;
    }
    std::array<OrbTracker::PendingFrame, 2> pending;
    for (std::size_t i = 0; i < 2; ++i)
    {
        auto const status = m_trackers[i].prepare(identity[i], images[i], masks[i], pending[i], diagnostics != nullptr);
        if (status != VisionStatus::success)
        {
            return status;
        }
    }
    auto next_id = m_next_track_id;
    StereoTrackingDiagnostics details;
    auto const association_status = associate(pending, details, next_id);
    if (association_status != VisionStatus::success)
    {
        return association_status;
    }
    for (std::size_t i = 0; i < 2; ++i)
    {
        auto const status = m_trackers[i].finish(pending[i], next_id, diagnostics != nullptr);
        if (status != VisionStatus::success)
        {
            return status;
        }
    }
    // All allocations and fallible operations precede either history update.
    for (std::size_t i = 0; i < 2; ++i)
    {
        m_trackers[i].commit(pending[i], output[i], diagnostics != nullptr ? &details.cameras[i] : nullptr);
    }
    m_next_track_id = next_id;
    if (diagnostics != nullptr)
    {
        *diagnostics = std::move(details);
    }
    return VisionStatus::success;
}

void StereoOrbTracker::reset()
{
    for (auto & tracker : m_trackers)
    {
        tracker.reset();
    }
}
} /* end namespace formal_gvins::vision */
