/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <formal_gvins/vision/orb_tracker.hpp>
#include <array>
#include <span>

namespace formal_gvins::vision
{
/** Fixed calibrated rig: y_R = R_RL * y_L + t_RL. No rectification assumption. */
struct StereoRig
{
    cv::Matx33d R_RL = cv::Matx33d::eye();
    cv::Vec3d t_RL{0, 0, 0};           // Metres, expressed in R; zero baseline is invalid.
    std::uint64_t maximum_skew_ns = 0; // Caller qualifies exposure synchronization; default exact timestamps.
    double epipolar_threshold = 0.003; // Normalized-coordinate Sampson distance.
    double minimum_epipolar_denominator = 1e-12;

    /** Validate a proper rotation/nonzero baseline and build unit-Frobenius E_RL atomically. */
    [[nodiscard]] bool essential(cv::Matx33d & E) const;
};

struct StereoMatchResult
{
    DescriptorMatches descriptors;
    std::vector<cv::DMatch> inliers; // query=left, train=right; observation/descriptor row indices.
    std::size_t geometry_rejected = 0;
};

/** Reciprocal descriptors plus known-rig geometry on calibrated normalized bearings.
 * Rejects undefined epipolar residuals. No triangulation/depth acceptance is implied.
 * On failure preserves output. Track IDs are not modified here.
 */
[[nodiscard]] VisionStatus verify_stereo(std::span<FeatureObservation const> left,
                                         std::span<FeatureObservation const> right,
                                         std::vector<std::vector<cv::DMatch>> const & forward,
                                         std::vector<std::vector<cv::DMatch>> const & reverse, StereoRig const & rig,
                                         float ratio, StereoMatchResult & output);

struct StereoTrackingOptions
{
    std::array<OrbTrackingOptions, 2> cameras;
    StereoRig rig;
    float ratio = 0.75F;
};

enum class StereoStatus
{
    accepted,
    no_matches,
    unsynchronized,
    backend_failure,
};

struct StereoTrackingDiagnostics
{
    std::array<OrbTrackingDiagnostics, 2> cameras;
    StereoMatchResult matches;
    std::size_t identity_conflicts = 0;
    StereoStatus association = StereoStatus::no_matches;
};

/** Two camera streams with one ID allocator and atomic history publication.
 * Invalid camera input/temporal backend failure preserves both histories and outputs.
 * Time-skew/stereo-link rejection or stereo backend failure retains valid mono results.
 * Calibration is fixed at construction. Single owner; no concurrent calls.
 */
class StereoOrbTracker
{
public:
    explicit StereoOrbTracker(StereoTrackingOptions const & options);
    [[nodiscard]] VisionStatus process(std::array<ImageIdentity, 2> const & identity,
                                       std::array<cv::Mat, 2> const & images, std::array<cv::Mat, 2> const & masks,
                                       std::array<TrackedImage, 2> & output,
                                       StereoTrackingDiagnostics * diagnostics = nullptr);
    /** Retire both camera histories without recycling global IDs. */
    void reset();

private:
    [[nodiscard]] VisionStatus associate(std::array<OrbTracker::PendingFrame, 2> & pending,
                                         StereoTrackingDiagnostics & diagnostics, std::uint64_t & next_id);
    StereoTrackingOptions m_options;
    std::array<OrbTracker, 2> m_trackers;
    bool m_valid = false;
    std::uint64_t m_next_track_id = 1;
};
} /* end namespace formal_gvins::vision */
