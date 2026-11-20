# Local VINS development profile

**Status:** Draft; proposed VINS-01 implementation binding, awaiting owner review.

The FEJ/reset binding below implements the [literature review](../../analysis/msckf-design-review.md)
recommendation as a revised Draft: conventional FEJ, Hamilton Exp injection and
identity reset. Owner review and approximation qualification remain pending.
Weak-geometry/no-update status clarification is separate follow-up work.

**Requirements:** [SW-MSCKF](../requirements/software/local-vio.md) and its children,
[V-TRACK](../requirements/vision/feature-tracks.md),
[V-ACQUISITION](../requirements/vision/acquisition.md),
[C-NUMERICAL-PROFILE](../requirements/common/numerical-profile.md).

This profile selects a bounded first implementation of the
[MSCKF formulation](../architecture/msckf-formulation.md). It does not replace
the mathematical contracts, approve requirements or certify physical accuracy.
Numerical checks and dataset observations support design selection; production
implementation, formal proofs and trajectory acceptance remain later work.

## Execution and acquisition

| Binding | First implementation | Basis / limit |
|---|---|---|
| Execution | Offline Linux, Eigen/double numerical core, one estimator owner; CPU ORB frontend. | Reuses the implemented components; no Orin timing claim. |
| Camera | EuRoC cam0, 752x480 original distorted grayscale images; fixed K, radial-tangential `(k1,k2,p1,p2)` and T_BC. | Raw-pixel residual under [M-VISUAL](../requirements/msckf/visual-geometry.md); pointwise undistortion for geometry. No online calibration. |
| Clock | Recording nanoseconds, fixed camera-to-IMU offset zero for this dataset profile. | Compare integer times before converting interval differences to seconds. No arrival-time substitution. |
| IMU interval | Average both endpoints of each segment, then analytically integrate the held body inputs. | [M-PROPAGATE](../requirements/msckf/propagation.md) owns boundary interpolation and splitting; no extrapolation or duplicated noise. |
| Propagation domain | `1 ns <= h <= 10 ms`, corrected `norm(omega) <= 10 rad/s`, `norm(a) <= 100 m/s^2`. | Selected development domain; the local MH_01 scan fits the raw input/time bounds, but that does not qualify other recordings or corrected estimates. |
| Gravity | `g_W=(0,0,9.81) m/s^2`. | Fixed local gravity, positive world z downward. W has a local origin/yaw; it is not north-aligned NED. |
| IMU noise | Square the four sensor.yaml noise-density amplitudes into the diagonal Qc blocks; use Q_epsilon=Qc/h and the analytical Gd congruence for each segment. | Continuous density, interval-equivalent rate covariance and measured sample variance are distinct. Correlated/anisotropic density remains supported by the reusable contract but is not inferred from EuRoC's scalar metadata. |
| Truth | Evaluation-only; never used to initialize/correct the normal replay. | Synthetic known-state test fixtures are separately identified. |

Missing IMU coverage returns not-ready; an actual gap beyond the admitted interval
domain prevents valid propagation across that gap, even if image boundaries split
it into shorter segments. Interpolation is allowed only within covered intervals;
extrapolation and interval clamping are not. Zero-duration boundaries require no
propagation. Reverse/duplicate image times are rejected before changing state.

This is the selected estimator input domain, not the current frontend behavior.
The [acquisition](euroc.md#local-vins-target) and
[ORB](orb-frontend.md#local-vins-target) profiles identify the required migration
from remapped images. Raw and derived normalized coordinates share one feature
identity; inverse-camera numerical limits and the frontend epipolar threshold
must be qualified before estimator integration.

## Capacity and ownership

| Resource | Selected bound / policy |
|---|---|
| Retained clones | 10 after each image transaction; one additional augmentation slot, so peak N is 11 and peak D is 81. |
| Frontend observations | At most 400 per image, retaining the existing frontend cap. |
| History records | 800 tracks: up to 400 previously active histories plus 400 current IDs during admission. Lost histories are processed or explicitly retired each image. |
| Observations per history | At most 11, one per clone for the mono profile. Current features are temporary, outside the filter state. |
| Constraints per correction | At most 40 feature histories. Prioritize impending clone retirement, then longer histories, then stable track ID. Unselected ending histories are explicitly retired, not reported as fused. |
| Correction scheduling | At most one joint correction per image; no iterative EKF or hidden retry with reused observations. |
| IMU startup storage | 512 samples for two one-second windows plus boundaries at the selected 200 Hz rate; overflow is explicit. |
| Initialization image history | At most 44 frames of timestamp/track/pixel records for both startup windows at 20 Hz; evaluate visual stillness on the older window. No image copies or filter clones are needed. |

The peak covariance contains `81*81*8 = 52,488` bytes. A track observation containing
a uint64 clone ID and two doubles has a 24-byte payload, so the rectangular history
capacity is 211,200 bytes before metadata/alignment. These are sizing calculations,
not measured object sizes. The separate startup history has at most 422,400 bytes
of track-ID/pixel payload. FEJ stores frozen IMU pose/velocity and clone poses;
there are no accumulated attitude maps or map-solve workspaces.

Use incremental QR compression: append at most 19 projected feature rows to at most
81 retained rows. Thus the compression workspace needs at most 100x81 coefficients,
rather than a single 760-row stack or its much larger innovation covariance.
One caller-owned workspace serves the estimator. Reserve a 2 MiB development budget
for numerical state, candidate state, tracks and scratch excluding images/OpenCV;
VINS-02/08 must measure actual peak storage and large automatic temporaries before
claiming the budget met. This is not an embedded-flight resource allocation.

Consumed histories start a new observation segment under the same frontend track
ID if tracking continues. Preserve the last consumed/retired image watermark;
no old observation is used as a new anchor measurement. Before clone removal,
every dependent pending observation is consumed or explicitly retired. A failed
correction preserves its ledger and pre-correction estimate; the first replay
stops on that numerical fault instead of silently discarding/retrying the update.

## Stationary-start initialization

Use two consecutive one-second IMU windows. The older window supplies the initial
estimate; the newer window must show departure from stillness. This follows the
static-then-motion structure of [OpenVINS's initializer][ov-init], with separately
selected admission thresholds. An RMS deviation is not a variance:
`s_x=sqrt(sum(norm(x_i-mean(x))^2)/(n-1))`.

| Admission quantity | Selected criterion | Rationale |
|---|---|---|
| Samples | At least 190 IMU samples per window, no gap above 10 ms. | Allows timestamp jitter while requiring coverage near the declared rate. |
| Older acceleration | `s_a <= 0.15 m/s^2` and `abs(norm(mean(f))-g) <= 0.5 m/s^2`. | Small variation plus a gravity-magnitude screen; neither alone establishes zero acceleration. |
| Older angular rate | `s_omega <= 0.02 rad/s`, `norm(mean(omega_m)) <= 0.2 rad/s`. | Admit a nonzero bias mean while excluding substantial rotation/variation. |
| Older visual motion | At least 20 track IDs spanning the window; P90 of each track's maximum displacement from its first raw pixel <= 2 px. | Uses all available observations of each spanning track; insufficient tracks means not-ready. This working threshold needs qualification on the raw-image frontend. |
| Newer motion | `s_a >= 0.30 m/s^2` or `s_omega >= 0.04 rad/s`. | Two-to-one hysteresis relative to stillness screens; this indicates excitation, not a known velocity. |

These are working development thresholds supported by a local startup inspection,
not fitted guarantees of stationarity. Initial handheld motion must not be accepted
merely because it occurs at the start of a file. Quiet constant acceleration and
unmodelled moving scenes remain failure modes; the stationary-start assumption
requires qualification on each accepted dataset. Do not relax the criteria silently
when no window qualifies. Dynamic initialization and ZUPT are not selected here.

At the last IMU timestamp of the older window, set position and velocity to zero,
gyro bias to the older mean, and align attitude so measured gravity points opposite
g_W. For deterministic local yaw, let `u_b=-mean(f)/norm(mean(f))`; project body x
onto its perpendicular plane and normalize it as `x_b`. If that projection's norm
is below 0.1, use body y instead. Set `y_b=u_b cross x_b` and
`R_WB=[x_b y_b u_b]^T`. This sets a coordinate convention, not observed heading.
The initial
accel bias is the radial residual `mean(f)+R^T*g_W`; transverse bias is unresolved,
not estimated to zero uncertainty. Propagate the buffered newer interval to the
first admitted image time; do not label the older estimate with the newer time.
Startup images support admission only. Start filter observation histories at that
first admitted image; do not fuse startup observations retroactively.

Proposed independent prior standard deviations are position 0.05 m, velocity
0.05 m/s, gyro bias 0.01 rad/s and accel bias 0.2 m/s^2. Position/yaw priors describe
the chosen local gauge, not surveyed position or north. Include tilt/bias coupling:
with `u_b=R^T*g_W/g` and `J=-[u_b]_x/g`, construct initial attitude error as
`delta_theta=J*delta_ba+eta_tilt+u_b*eta_yaw`, where eta_tilt has covariance
`0.02^2*(I-u_b*u_b^T)` and eta_yaw variance is `0.05^2`.
Construct the joint prior as a congruence of these independent latent errors;
retain the resulting attitude/accel-bias cross-block. The floor represents a
development prior, not a measured camera/IMU calibration uncertainty. Test this
construction and qualify its conservatism in VINS-03/12.

## Propagation evaluation and numerical checks

Select the formulation's endpoint-averaged held-input mean and physical Phi, not
`I+F*h`. The corrected input bounds apply to the effective segment inputs. For the
admitted `norm(omega*h) <= 0.1`, evaluate Gamma_1/Gamma_2 by their series through
power 10.
The exact-arithmetic Gamma_1 tail in spectral norm is below `2.2e-20`; the Gamma_2
tail is smaller. Rounding and scalar Exp/J_r remain separate obligations.

Select fixed five-point Gauss-Legendre integration for B1/B2. Build Gd with the
same FEJ references as Psi and evaluate `Qd=Gd*(Qc/h)*Gd^T`, retaining every induced
cross-block. Qd is not evaluated by quadrature of the continuous noise integral.
Exact PSD follows from the congruence, including with approximate coefficients;
this does not bound their accuracy or floating-point error. Native cost and
uniform error bounds remain to be qualified. Five points is a development
accuracy choice, not an established fastest implementation.

The coefficient targets are OpenVINS's ideal analytical Xi1--Xi4 after convention
translation. Its pinned `compute_Xi_sum` instead uses approximate branches below
`norm(omega) < pi/360 rad/s`. This profile retains its series/integral evaluation;
compare both sides of that source threshold and record approximation differences
separately from algebraic correspondence. Do not assert source-execution equality.

VINS-04 reference tests must include interval endpoints, zero/near-zero rate,
nonidentity attitude, anisotropic/correlated PSD noise and split intervals. Proposed
double-precision checks are infinity-norm Phi error <= `1e-11*max(1,norm(Phi_ref))`
and relative infinity-norm Qd error <= `1e-9` for nonzero Qd_ref, with exact zero
for zero Qc. The independent reference uses high-precision coefficient integration,
differentiation of the held-input mean and source-order Gd/noise mapping; it does
not reuse the production quadrature or substitute the continuous-noise integral.
Check the analytical accel-only `h^3/4`, `h^2/2`, `h` blocks and bias-only limits.

Under identical held inputs without correction, require mean/ordinary-transition
agreement under subdivision using the same normalized limits. For noise, require
the segment recurrence, including the formulation's `5*h^3/16` half-step example,
not equality with a single unsplit Qd. Changing endpoint readings must match the
selected piecewise reference. Include image-boundary interpolation, differing
current/FEJ attitudes and recomputation after a bias correction. Statistical
effects of segmentation/interpolated sample correlations remain empirical work.

For matrices that are mathematically symmetric, their owning operation checks
`norm(M-M^T)_inf <= 1e-12*max(1,norm(M)_inf)`, then mirrors the upper triangle
before a symmetric solve/publication. This is an explicit numerical policy, not
permission for the solver to repair arbitrary input. Non-finite results or excess
asymmetry fail. No diagonal jitter is selected. Joint P may be PSD and singular;
only the innovation/measurement systems requiring an SPD solve are tested by
Cholesky. Native eigenvalue/residual diagnostics do not become a proof of PSD.

## FEJ and reset binding

Use [M-FEJ](../requirements/msckf/consistency.md)'s complete Psi, frozen geometric
Jacobians and cache lifetimes. Propagate nominal means from current corrected
values, refresh the current IMU first estimate only at the propagated endpoint,
and retain clone first estimates through corrections. Gd uses those same frozen
start/endpoint attitude factors; its A/B coefficients use the current corrected
inputs. No T maps are stored.

[M-CORRECT](../requirements/msckf/correction.md) selects Joseph update,
Hamilton/right-local Exp injection for every attitude, and `P_after=P_corrected`
(identity reset). Frozen first estimates are not changed by injection. This is
an explicit covariance approximation, not the physical Jr reset.

- Retain the proposed maximum attitude correction of 0.2 rad for every IMU/clone
  attitude. Reject the complete candidate if any exceeds it; no clipping or
  hidden subdivision. This is a development guard, not an OpenVINS default or
  an experimentally qualified consistency bound.
- At that limit, Exp differs from the reference's normalized small-angle
  increment by about `6.627e-4 rad` (0.038 degrees), below `0.2^3/12`.
  First-order agreement is not finite-update equivalence.
- The exact physical reset derivative obeys `norm(J_r(d)-I)_2 <= 0.1` in this
  domain. The derivation's conservative one-step covariance comparison bound
  is `0.21*norm(P_corrected)_2`. This is not an acceptance tolerance or a bound
  on accumulated error; it shows why the 0.2-rad guard alone cannot qualify
  identity reset. Repeated-update consistency and accuracy need measurement.

There are no map-conditioning/solve checks in this baseline. Numerical
normalization/symmetry/solve checks apply to the actual estimator operations. Current/frozen divergence, translation
and feature errors remain part of VINS-07/10/12 acceptance. The frozen gauge
identities do not prove nonlinear/statistical consistency.

## Visual constraints and updates

| Binding | Selected development value / behavior |
|---|---|
| Feature history | 3--11 distinct clones; every selected observation remains unconsumed. |
| Projection domain | Camera z depth in `[0.1,60] m` at current and frozen evaluation points. This is a depth bound, not Euclidean feature distance. |
| Camera inversion | Pointwise radial-tangential inverse with a bounded iteration cap and raw-pixel forward-reprojection check. The valid camera domain, cap and tolerance remain to be qualified before acceptance; no unbounded convergence loop. |
| Parallax | Maximum angle between world-frame observation rays >= 1 degree. |
| Rank / condition | For current triangulation and frozen H_f, use relative QR rank tolerance `1e-10`, absolute zero; require rank three and `cond_1(R_3x3) <= 1e4` using triangular solves. Rank alone is not the condition check. |
| Refinement | Weighted Gauss-Newton, at most 5 iterations; at most 5 trial scales `1,1/2,...,1/16` per iteration. Accept convergence before line search when step norm <= 1e-6 m. Otherwise require valid depths and decreasing cost; improvement divided by `max(1,previous cost)` <= 1e-6 also converges. Exhaustion without convergence rejects the feature. |
| Pixel noise | Initial independent `V=1.0^2*I` in raw pixel coordinates. A working noise model, not calibrated ORB localization accuracy or a transfer of the remapped-image benchmark's noise. |
| Feature gate | `alpha=0.05`, dimension `d=2m-3`; accept equality at the 95% chi-square threshold before compression. |
| Compression | Apply QR to H and residual together; retain at most D rows and undo column permutations. Do not use a numerical-rank threshold alone to discard small nonzero rows. Qualify the information identities below in VINS-08. |
| Correction | One stacked Joseph update, all-pose injection/reset, then atomic state/cache/consumption publication. |

The one-pixel setting and 95% gate have a reference basis in
[OpenVINS's profile][ov-config] and [feature update][ov-msckf], but its calibration,
tracking and geometry settings differ. Depth/refinement scales are informed by its
[triangulation options][ov-geometry]; this profile does not copy that algorithm.
The reference [initializer][ov-feature-init] refines anchored inverse depth using
unweighted normalized-coordinate residuals and damping. This profile refines
global XYZ with raw-pixel covariance weighting and bounded backtracking; identical
measurements need not yield identical finite-iteration feature estimates.
Refinement residuals, Jacobians and covariance all use the raw-pixel camera model;
normalized bearings are used for initialization and parallax geometry.
Condition metrics and units above are local definitions. Outlier selection and
inter-observation correlations limit the nominal chi-square interpretation.

For compression from `(H,r)` to `(H_c,r_c)`, require both
`norm(H_c^T*H_c-H^T*H)_inf / max(1,norm(H^T*H)_inf) <= 1e-10` and
`norm(H_c^T*r_c-H^T*r)_inf / max(1,norm(H^T*r)_inf) <= 1e-10` in reference tests.
The discarded state-independent residual is not used to repeat the feature gate.

## Validity and review gates

Geometry/rank/gate rejection retires an observation candidate explicitly; it does
not imply successful fusion. Numerical solve, normalization or covariance failures
preserve the pre-correction transaction and stop the first offline replay with a fault record.
Empty accepted constraints skip correction. Earlier successful propagation and
augmentation remain separate transactions.

Report acquisition time, initialization state, last successful visual-update time
and whether the output is propagation-only. After initialization, no visual update
for 1 s marks degraded aiding; a valid correction before 5 s can restore aiding.
At 5 s without correction, mark the session lost and require explicit new
initialization. Do not resume corrections in that session, reset covariance or
reuse consumed observations to recover. Irrecoverable time, capacity or numerical
faults also require an explicit new session; no automatic reinitialization is
selected. At 20 Hz these timers allow 20/100 missing correction opportunities;
they are explicit development stop rules, not measured navigation-error bounds
or qualified recovery limits.

Owner review of this profile is separate from implementing its values. VINS-LIT's
literature review has led to the revised consistency binding above; owner review
and the remaining profile clarifications precede dependent VINS-02 implementation. Required
evidence remains:
startup false-admission/rejection tests (VINS-03), native numerical oracle tests
(VINS-04/07/08/09), composed FEJ/consumption sequences (VINS-10), and held-out
trajectory, consistency and timing acceptance (VINS-11/12). Keep local experiment
scripts/reports outside versioned specifications; this profile records decisions
and limits, not a run diary or a completed-proof claim.

[ov-config]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/config/euroc_mav/estimator_config.yaml
[ov-init]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_init/src/static/StaticInitializer.cpp
[ov-update]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/StateHelper.cpp
[ov-msckf]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/update/UpdaterMSCKF.cpp
[ov-geometry]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/feat/FeatureInitializerOptions.h
[ov-feature-init]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/feat/FeatureInitializer.cpp
