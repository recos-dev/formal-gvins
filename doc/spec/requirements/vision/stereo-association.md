# V-STEREO: Calibrated stereo association

**ID:** `V-STEREO`

**Status:** Draft

**Applies to:** A configured global-shutter stereo pair; conditional, not enabled by the first mono profile.

**Parent:** [V-ORB](orb-frontend.md).

**Uses:** [V-ACQUISITION](acquisition.md), [V-OBSERVATION](observation-admission.md), [V-ASSOCIATION](track-association.md), [M-UNPROJECT](../msckf/camera-unprojection.md).

## Basis

[OpenVINS descriptor tracking][ov-match] demonstrates reciprocal descriptor
association across cameras. This project instead uses the known calibrated rig
geometry for stereo verification; estimating an unconstrained F from the same
matches would not verify consistency with that rig. See the
[frontend guide](../../../guides/frontend.md) for the
distinction from rectified horizontal-disparity matching.

## Required behavior

The acquisition adapter shall establish compatible calibration, a nonzero
baseline, exposure/clock semantics and a qualified synchronization bound before
stereo association. Pairing shall be one-to-one by identified exposures, not by
arrival time or the nearest image without a time bound. A pair exceeding the
bound shall not use fixed-rig geometry as if simultaneous. Motion compensation
for asynchronous cameras is outside this contract.

After V-LOCALIZE and V-ORB's reciprocal descriptor checks, use normalized
bearings from M-UNPROJECT. With camera-to-body transforms from V-FRAMES, define

$$
R_{RL}=R_{BR}^{\mathsf T}R_{BL},\quad
t_{RL}=R_{BR}^{\mathsf T}(p_{BL}-p_{BR}),\quad
E_{RL}=[t_{RL}]_\times R_{RL},\quad e=d_R^{\mathsf T}E_{RL}d_L.
$$

The point in the right camera is `y_R=R_RL*y_L+t_RL`. Test the epipolar
residual using V-ORB's unit-Frobenius normalization and Sampson metric, with
the stereo threshold in the frontend profile. A vanishing
denominator or invalid bearing shall reject the pair; zero algebraic e alone
shall not pass a degenerate geometry test. Raw-image matching shall not assume
equal image rows, identical intrinsics, parallel optical axes or `depth=f*b/disparity`.

Only geometrically accepted one-to-one pairs may establish a new shared feature identity.
Previously shared IDs may persist through independently valid temporal continuations;
this does not declare a new simultaneous stereo link.
Temporal identity has priority: two existing different track identities shall
not be merged retroactively; reject the conflicting stereo link. A new observation
may inherit the verified partner's identity, and two new observations may receive
one new identity. An ID shall appear at most once in each camera image. Preserve
distinct camera/image records and consumption history. An identity still live in one
camera may be reacquired by the other through a new validated stereo link. Events
and ages describe each camera segment; they shall not allocate a new global ID
merely because that camera segment restarts. Never recycle a globally retired ID.

A malformed camera input or temporal-processing failure shall preserve both
camera histories and both output batches. Prepare both results and their owning
history before committing either camera. A stereo-only backend failure or rejected
link shall publish no new stereo association but retain the valid monocular results;
the diagnostics shall distinguish this from accepted stereo. Reset shall retire both
histories without recycling IDs. Numerical rig calibration remains immutable.

A failed stereo link shall not invalidate an independently verified temporal
continuation. Unmatched observations may remain monocular. Stereo acceptance
shall not imply finite-depth triangulation success. The estimator caller shall
bind each exposure to its modeled clone time and size history/constraint capacity
by observation count, not merely by number of shared clones.

## Acceptance and verification

Test a known 3D scene with nonparallel cameras, unequal intrinsics, lens distortion
and nonzero lever arms. Verify transform direction, reciprocal uniqueness,
occlusions, repeated textures, time skew, zero baseline, epipolar degeneracy,
identity conflicts, duplicate-ID prevention, camera reacquisition, reset, failure
of the second camera and a failed stereo link with a valid temporal track.
Use labeled synthetic correspondences to measure false admission and rejection;
low epipolar error alone is not physical correspondence ground truth.

A stereo profile shall bind synchronization/motion limits, overlap, geometry
thresholds, observation capacities and noise before estimator acceptance. Mono
tests do not establish those bounds or qualify stereo implementation.

[ov-match]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/track/TrackDescriptor.cpp
