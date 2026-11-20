# MSCKF literature and design review

**Status:** Draft recommendations for VINS-LIT; not requirement approval.

**Review cutoff:** 2026-11-21. The reviewed candidates are the
[formulation](../spec/architecture/msckf-formulation.md) and
[local VINS profile](../spec/profiles/local-vins.md). There is no implemented
joint estimator to qualify yet.

## Recommendation

Keep the bounded MSCKF architecture, Hamilton/right-local SO(3) convention,
IMU density inputs with analytical interval-equivalent noise, full joint covariance,
once-only observations and whitened feature elimination. These are a reasonable
basis for the first local VINS. ORB remains a frontend choice, not a reason to
change the estimator family.

The revised Draft selects conventional FEJ with a full frozen-endpoint transition,
Hamilton/right-local Exp injection and an identity-reset covariance approximation.
It excludes accumulated attitude-coordinate maps. Review the coordinated
[M-FEJ](../spec/requirements/msckf/consistency.md),
[M-CORRECT](../spec/requirements/msckf/correction.md), formulation and profile;
these now contain the proposed behavior, while this report explains the choices.

OpenVINS supplies the analytic FEJ/reset and discrete-noise reference. Our Exp
increment, Joseph update, coefficient evaluation, fully frozen distortion
derivative and temporary-feature refinement remain explicit differences.
Neither that reference nor the linear-model gauge identities establish nonlinear
or statistical consistency. The rejected map construction is examined below to
explain its exclusion, not to preserve it as another implementation mode.

The revised definitions are ready for owner review. VINS-LIT's implementation gate
remains open for that review and the profile clarifications below; no estimator
implementation or formal proof is implied.

## Source inventory and scope

Selection covers foundational MSCKF, observability and error coordinates;
implemented filter baselines; recent alternatives affecting those choices; and
representative smoothers/SLAM systems for initialization and future mapping.
Search families included MSCKF consistency/FEJ, invariant/equivariant VIO,
pose-only/immediate MSCKF updates, initialization, and square-root elimination,
with reference-following from the baseline papers. This is a scoped design review,
not an exhaustive bibliography or a reproduced SOTA accuracy ranking.

`Equations` means the identified mathematical sections were inspected, not that
every theorem was independently proved. `Design` means the identified architecture
sections were inspected. `Screened` means relevance/assumptions only; it does not
authorize implementation of the proposed method.

| Ref | Primary source / inspected version | Review scope and relevance |
|---|---|---|
| B1 | [Mourikis and Roumeliotis, MSCKF, ICRA 2007][msckf] | Equations: III-B--E, (11)--(16), (21)--(31). Joint propagation, augmentation, feature elimination, update triggers, QR compression and Joseph update. |
| B2 | [Li, Visual-Inertial Odometry on Resource-Constrained Systems, 2014][li-thesis] | Equations: Chapter 2, especially 2.7, (2.61)--(2.63). First available estimates and observability. This author thesis is the inspected source; the [2013 IJRR paper][li2013] full text was unavailable, so its exact equation numbering is not asserted. |
| B3 | [Hesch et al., Consistency Analysis and Improvement of Vision-Aided Inertial Navigation, TRO 2014][oc] | Equations: V, (60)--(77), and VI-B. Propagated gauge, constrained Jacobians and MSCKF application. |
| B4 | [Solà, Quaternion kinematics for the error-state Kalman filter, 2017, v1][sola] | Equations: local error, injection and 6.3/6.3.1 reset. Convention translation and the distinction between reset derivatives and covariance approximation. |
| B5 | [Sun et al., Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight, 2018, v3][smsckf] | Equations/design: III. Stereo MSCKF propagation, constraints and state management; useful flight baseline, not a mono-ORB equivalence claim. |
| B6 | [Geneva et al., OpenVINS, ICRA 2020][ov-paper] | Equations/design: III--IV. Modular filtering, measurement/calibration models and simulation; read with pinned implementation below. |
| N1 | [Chen et al., FEJ2, ICRA 2022][fej2] | Equations: III-A/B, (14)--(26). Frozen-linearization error and an alternative treatment; no justification for ignoring finite corrections. |
| N2 | [Wu et al., An Invariant-EKF VINS Algorithm for Improving Consistency, 2017, v2][riekf] | Equations: III--IV, particularly retraction (14). Rotation/position/velocity coupling is different from our additive Euclidean errors. |
| N3 | [van Goor and Mahony, EqVIO, arXiv v3, 2026][eqvio] | Design/equations: 4--5, state, dynamics (5), bearing measurements (6), landmark parameterization. A different filter with landmark state, not an interchangeable MSCKF update. |
| N4 | [Fornasier et al., MSCEqF, RA-L 2024; arXiv v1][msceqf] | Equations: III--IV, (1)--(8). Equivariant bias/calibration state and multi-state visual constraints; closer alternative, still a different error model. |
| N5 | [PO-MSCKF, 2024, v1][poseonly] | Equations: III, pose-only observation construction. A candidate alternative residual, not evidence that orthogonal elimination is intrinsically incorrect. |
| N6 | [An Immediate Update Strategy of MSCKF, arXiv v1, 2024][immediate] | Equations: III-A, Fig. 1, (21)--(26). Growing observation prefixes across updates require an explicit correlation/reuse treatment before adoption here. |
| N7 | [Tian et al., Equivariant Filter Transformations, 2026, v2][transform] | Equations: IV-C, V and supplement S7. Coordinate transformation premises and the limits of first-order equivalence. |
| C1 | [Qin et al., VINS-Mono, arXiv v1, 2017][vinsmono] | Design: V--VII. Moving initialization, optimized window and recovery; these do not follow automatically from implementing an MSCKF. |
| C2 | [Campos et al., ORB-SLAM3, 2021, v2][orbslam] | Design: III--V. Atlas, visual-inertial initialization and mapping; descriptors alone do not provide relocalization. |
| C3 | [Leutenegger, OKVIS2, 2022, v2][okvis] | Design: V--VI. Real-time estimator and asynchronous loop optimization/synchronization are separate responsibilities. |
| C4 | [Demmel et al., Square Root Marginalization for Sliding-Window Bundle Adjustment, ICCV 2021, v1][sqrt] | Equations: 3.2, 4.1--4.3. QR elimination and Schur equivalence in a linearized least-squares problem; not covariance clone removal. |
| S1 | [Cerezo et al., Full Visual-Inertial State Initialization, 2026, v3][init] | Screened: small-rotation/constant-velocity assumptions and staged startup. Revisit for moving-start support; not needed for the stationary first profile. |
| S2 | [RDIC-MSCKF, Machines 2026][rdic] | Screened: visual uncertainty calibration and its stated evaluation limits. Relevant to noise qualification, not a reason to add adaptive noise before a fixed baseline exists. |
| S3 | [POPL-KF, 2026, v1][popl] | Screened: point/line pose-only model. Requires line frontend and new measurement contracts; deferred. |

The inventory contains 13 scoped equation reviews, 4 complementary design
reviews, and 3 screened proposals. B2's author thesis substitutes for inaccessible
journal full text; these are not two independently reviewed publications.
Learned features/bias models, NeRF maps and non-rigid-scene estimators are outside
the first calibrated, rigid-scene, ORB local-VINS scope. Their omission is not a
claim that classical MSCKF dominates those methods.

### Implementation reference

OpenVINS is pinned to `69488123ed9362dd44b6f28e7f4680abbff1442b`.

| Inspected entry | Relevant observation |
|---|---|
| [Propagator::compute_F_and_G_analytic][ov-prop] | FEJ selects frozen R, p and v; frozen R also enters bias-sensitivity blocks. The revised Psi includes these factors; the previous three-attitude-block patch did not. |
| [StateHelper::EKFUpdate][ov-update] and [JPLQuat::update][ov-quat] | The inspected update uses covariance subtraction and normalized small-angle quaternion injection; it omits explicit reset transport, which informs the selected identity-reset approximation. |
| [UpdaterHelper::get_feature_jacobian_full][ov-jac], lines 326--392 | Residuals use current raw-pixel predictions. FEJ replaces pose and pinhole factors, but leaves `uv_norm` current for the distortion derivative. Our Draft freezes that derivative too; both common left factors preserve geometric gauge cancellation, without giving identical updates. The implemented remapped-image frontend still needs migration. |
| [FeatureInitializer::single_gaussnewton][ov-feature-init] | Refines anchored inverse depth with unweighted normalized-coordinate residuals and damping. Our profile uses global XYZ, raw-pixel noise weighting and bounded backtracking. It retains the triangulate/refine/eliminate structure, not the same numerical optimizer or noise objective. |

The source uses JPL quaternions and a different rotation/state ordering. Compare
rotation actions and error derivatives, not quaternion coefficient signs or copied
matrix blocks. The project keeps `R_WB`, scalar-first Hamilton multiplication,
right-local attitude error and error order `(p,v,theta,ba,bg)`.

The propagation comparison selects OpenVINS's **analytical path** with fixed
ideal calibration. Its pinned [EuRoC configuration][ov-config] selects RK4 means,
analytical covariance propagation, stereo, KLT, online camera/time calibration
and persistent SLAM features. Those defaults are not our mono/ORB, fixed-calibration,
local-MSCKF profile. Runtime thresholds, feature admission and recovery also remain
local bindings; matching the filter architecture does not import those settings.

## Design decisions

`Keep` means the architecture has a sound basis, not that code/proofs exist.
`Adapt` means the explicit local assumptions are reasonable but need the stated
tests. `Draft revised` identifies a coordinated definition change awaiting owner review. `Qualify` retains
a development choice without claiming operational acceptance.

| Area / owner | Source and comparison | Disposition and required evidence |
|---|---|---|
| 1. State and frames / [M-STATE][state] | B1 (1)--(5) stores IMU plus camera poses. IMU-pose clones with fixed extrinsics encode the same pose information. N2/N4 instead change error/retraction geometry. | **Keep.** Verify the fixed-extrinsic clone/observation transform and right-local derivatives. Full `15+6N` covariance; no persistent features in this baseline. |
| 2. Propagation/noise / [M-PROPAGATE][propagate] | B1 (12)--(13), B5 III use a continuous stochastic model. Endpoint averaging/interpolation and analytical Gd*(Qc/h)*Gd^T follow the pinned OpenVINS path after convention/order translation. | **Adapt.** Check FEJ factors, noise scaling, all cross-blocks and coefficient approximations. Qd is an interval-equivalent approximation, not the exact continuous noise integral. Require sequential noise composition, not split/unsplit equality; only identical-input mean/ordinary-transition composition has that equality. |
| 3. FEJ/observability / [M-FEJ][fej] | B2 (2.61)--(2.63), B3 (60)--(74), N1 III support preserving a common linearized gauge. The revised Psi follows the full analytic FEJ structure; it excludes the rejected three-block-plus-T construction. | **Draft revised.** Full Psi, cache lifetimes and the shared Psi/Gd frozen-attitude binding are specified together. Review every block and the declared source differences; gauge identities alone do not validate bias behavior. |
| 4. Injection/reset / [M-CORRECT][correct] | B4 6.3 explains error reset. N2 (14) and N7 S7 show that finite retraction matters in addition to Jacobian transformations. | **Draft revised.** Hamilton Exp injection and identity reset are selected; physical Jr transport is not claimed. Verify the stated approximation bounds and combined sequences, then qualify consistency empirically. |
| 5. Initialization / [M-STATE][state], profile | C1 V addresses moving start. Our static window instead assumes near-zero motion; static IMU alone does not observe all bias/tilt/velocity components. | **Qualify.** Keep stationary-start scope and coupled tilt/bias prior. Test false admission, unresolved bias, buffered propagation and delayed startup; the selected thresholds/prior magnitudes have no flight qualification. |
| 6. Clone/history lifetime / [M-CLONE][clones], V-CONSUMPTION | B1 (15), III-E supports augmentation and updates before removal; its pose-selection policy differs. N6 reuses growing prefixes. | **Adapt.** Keep once-only segments and bounded FIFO initially. Test cross-covariances, singular PSD augmentation, identity/consumption and retirement. Covariance removal is a principal submatrix, not a Schur complement. |
| 7. Geometry / [M-VISUAL][visual] | B1 III-D, B5 III, B6 III-C support multi-view geometry. Raw-pixel residuals match the reference domain; fully frozen distortion derivatives and weighted raw-pixel XYZ refinement are local choices. Fixed extrinsics and finite-depth rank-three triangulation narrow our domain. | **Draft revised.** Verify distortion/inverse domains, lever-arm signs and current/frozen Jacobians independently, including the reference's current-point distortion factor. Qualify the distinct refinement objective. Migrate the K-only frontend geometry before using raw images. Reject invalid depth/rank; do not invent depth during pure rotation. |
| 8. ORB observation noise / V-ORB, M-VISUAL | B6 III-C models raw image measurements; C2 uses a different ORB/map pipeline. Selecting the same residual domain does not make our detector/scale/localization statistics identical. | **Qualify.** One raw pixel is a working standard deviation, not a sourced accuracy bound. Requalify noise by scale/image region; do not transfer remapped-image benchmark acceptance or claim validated gates/NEES. |
| 9. Whitening/nullspace/gating / [M-CONSTRAINT][constraint] | B1 (21)--(24), N4 (8), C4 4.1 support elimination in the chosen linear model. Generic covariance requires whitening all residual/Jacobian rows together. | **Keep.** Check `A^T Hf=0`, projected covariance and information equivalence. Gate before batch compression, using the actual residual dimension/rank. A selected inlier population does not by itself validate chi-square consistency. |
| 10. Compression/joint update / M-CONSTRAINT, M-CORRECT | B1 (26)--(31) explicitly uses QR and Joseph form. OpenVINS's covariance subtraction is a different numerical implementation of the linear update. | **Keep.** Preserve sufficient statistics after compression; solve S without an explicit inverse. P may be PSD/singular; V SPD makes S SPD in exact arithmetic. Test floating-point failure atomicity separately. |
| 11. Time/calibration/loss / V-ACQUISITION, [M-LIFECYCLE][lifecycle] | B6 III-C/D models calibration/time; C1 recovery and C2 relocalization exceed the first baseline. | **Qualify/revise status semantics.** Fixed calibration/zero offset are dataset bindings. Distinguish missing images, track failure, poor geometry and rejected innovations. No-update timers are development stops, not proven loss-of-navigation thresholds. |
| 12. Resources/evaluation/extensions / SW-MSCKF, VINS-RT/12 | B5 motivates onboard filtering; C3 separates real-time estimation and loop optimization. Paper runtimes use different hardware/workloads. | **Qualify.** Measure bounded workspace and stage latency, then trajectories and observable-subspace consistency. RTX 3060 timings do not qualify Orin. Keep GNSS/map feedback outside this local four-gauge proof. |

The matrix has three Keep, two Adapt, four Qualify, and three Draft revised areas;
area 11 also requires a status-semantics clarification. These counts describe
design dispositions, not proof coverage or a percentage of completed VINS.

## Why accumulated reset-coordinate maps are excluded

The excluded proposal sets `delta_x=T*e`, `Hx=Hx0*T^-1`, `T_next=G*T`, and conjugates
the transition. For an invertible T, these operations preserve the specified
frozen-linear identities. That is useful algebra. A transported gauge differing
from the latest physical gauge is not, by itself, proof that FEJ is wrong.

However, that algebra does not establish that the chosen current residual,
finite mean correction, new endpoint reference and noise model form an estimator
equivalent to a reference FEJ or invariant filter. N7 explicitly separates
first-order transformation equivalence from nonlinear inverse maps. N1 also
identifies additional errors from frozen linearization. Neither paper proves
this project's particular accumulated-reset construction.

The conditioning guard addresses a different problem. For eleven identical
allowed corrections `d=(0,0,0.2)` rad,

$$
T=J_r(d)^{11},\qquad
\kappa_2(T)=\left(\frac{0.1}{\sin 0.1}\right)^{11}\simeq1.01851.
$$

Each right Jacobian's transverse polar rotation is -0.1 rad; their product
rotates that plane by -1.1 rad (about 63 degrees), while `kappa_1(T)` is about
1.80850 and passes that proposal's limit 4. This analytical example, also evaluated
numerically, shows that good conditioning does not bound closeness to identity,
accumulated correction or frozen-linearization error. It is not an observed
trajectory failure, nor a proof that this sequence occurs in normal flight.

There is also a source-level difference independent of T: the excluded Phi0 retained
current-state bias columns, whereas the inspected OpenVINS analytic FEJ path
uses frozen attitude in several such columns. Zero bias components in the gauge
make those columns invisible to `Phi*N=N_next`; that identity cannot validate
their estimation behavior. A modified gauge-preserving transition may be a
legitimate design, but must not inherit the reference implementation's results.

The revised baseline resolves the definition choices as follows:

1. Specify all of Psi with frozen endpoint attitudes, including bias-sensitive
   blocks. Nominal propagation uses current corrected means; Gd uses those same
   frozen attitude factors with current-input A/B coefficients. Qd follows the
   analytical interval-equivalent model. No accumulated T maps remain.
2. Retain Hamilton/right-local Exp injection. The pinned reference instead uses
   a normalized small-angle quaternion; the finite rotation difference is bounded
   by `norm(d)^3/12` in the admitted domain. First derivatives agree, not finite
   trajectories or floating-point outputs.
3. Select identity covariance reset as an explicit conventional-FEJ approximation.
   Keep first estimates fixed through correction. The physical right-log reset
   derivative is still Jr(d); the formulation bounds the one-step discrepancy
   without claiming accumulated/statistical accuracy. A future physical-reset
   alternative requires a complete new consistency review.

Required checks include convention-translated Jacobians, joint cross-blocks,
repeated corrections followed by propagation/clone retirement, large initial
errors and near-static trajectories. Synthetic Monte Carlo should use a known
independent trajectory/noise generator, consistent truth/error coordinates, and
observable-subspace NEES or an explicitly anchored gauge. Compare accuracy,
uncertainty and failures with a reference FEJ under controlled sensor inputs.
Do not use free alignment to hide scale or uncertainty errors.

## Profile findings that need explicit treatment

### Stationarity and weak geometry

The proposed rank-three/one-degree-parallax test can reject all new finite-depth
tracks during pure rotation or long stationary hover after older useful clones
expire. Valid images do not guarantee translational parallax. Consequently the
five-second no-update rule can stop an otherwise functioning acquisition/tracker.

Keep this restriction visible for the first excited-motion offline milestone.
Separate the reason for unavailable aiding from a claim of physical estimator
failure. Before claiming hover-capable quadrotor navigation, evaluate a properly
derived rotation/bearing constraint, qualified stationary update, stereo or
persistent landmarks. Merely lowering a rank threshold and assigning arbitrary
depth is not a valid remedy. No extra model is adopted in this review.

### Visual uncertainty and gating

The extractor retains OpenCV keypoints, but the current core-facing
`FeatureObservation` exports only track ID and u/v; image metadata binds one
noise model. Per-octave uncertainty cannot currently be inferred by the estimator
from those fields. See the existing [frontend comparison](orb-frontend-settings.md)
for preprocessing and subpixel differences.

The revised Draft retains raw pixel observations and derives normalized bearings
only for geometry. It removes full-image undistortion from the target pipeline,
but does not establish a calibrated raw-pixel noise model. The current code still
remaps images; the [frontend profile](../spec/profiles/orb-frontend.md#local-vins-target)
records the pending input/geometry migration.

Start with the declared fixed covariance as an experimental baseline. Measure
localization error on known warps/correspondences and synthetic calibrated scenes,
then inspect real residual distributions. Stratify by scale and image region;
only add octave/covariance metadata if a selected model needs it. Matching
precision, inlier count and post-gate NIS are not interchangeable accuracy or
consistency measurements. The 95% gate and startup/no-update thresholds remain
unqualified profile choices.

### Recent alternatives

| Method | Decision for the first implementation |
|---|---|
| FEJ2 | Defer. It addresses a real frozen-linearization limitation, but adds its own projection/noise treatment and acceptance work. |
| RI-EKF / MSCEqF / EqVIO | Retain as comparison candidates. They change error geometry, bias/calibration coupling or landmark representation; not a quaternion-helper replacement. |
| Pose-only / point-line models | Defer. Derive the complete residual, shared-anchor noise/correlations and gauge before replacing the existing feature-elimination contract. |
| Immediate updates | Defer. N6's Fig. 1 uses overlapping prefixes. The inspected update equations do not provide the correlation bookkeeping needed to satisfy our once-only contract; do not fuse those prefixes as independent new data. This is an adoption gap, not a blanket verdict on the paper's experiments. |
| Adaptive/learned noise and moving initialization | Defer until the fixed-noise, stationary-start baseline has independently scored failures and consistency. |
| Smoother, relocalization and loop closure | Keep as separate extensions. Map corrections require explicit covariance/correlation ownership; ORB descriptors alone do not establish this interface. |

## Follow-up and verification boundary

| Order | Action / existing work owner | Completion |
|---|---|---|
| 1 | Review revised consistency definitions / VINS-01, VINS-LIT | Owner reviews the coordinated requirements/formulation/profile, reference translation and approximation limits before FEJ-dependent VINS-02/04/05/09 code. |
| 2 | Clarify admitted motion and failure reasons / VINS-01, VINS-10 | Distinguish missing data/tracks, poor geometry, gate rejection and numerical failure; state what the mono first milestone cannot support. |
| 3 | Qualify numerical and sensor assumptions / VINS-03/04/07/08/12 | Execute independent propagation/geometry tests, initializer false-admission cases and observation-noise experiments. Pin acceptance thresholds separately. |
| 4 | Continue the existing implementation sequence / VINS-02 onward | Keep the roadmap IDs. Add module evidence as code exists; integrate synthetic and held-out EuRoC trajectories before runtime claims. |

Formal proofs should cover the selected model's propagation, clone covariance,
whitening/elimination, Joseph update, reset and gauge composition. Bounded source
checks should cover indexing, finite domains, failure atomicity and once-only
consumption. Empirical model fidelity, calibrated uncertainty, real-time margins
and flight robustness require different evidence; proving algebra does not settle
them. No Lean, ESBMC, trajectory benchmark or flight validation was completed by
this literature review.

[msckf]: https://www-users.cse.umn.edu/~stergios/papers/ICRA07-MSCKF.pdf
[li-thesis]: https://escholarship.org/uc/item/4nn0j264
[li2013]: https://doi.org/10.1177/0278364913481251
[oc]: https://www-users.cse.umn.edu/~stergios/papers/TRO_2014_OC_VINS_Joel_Dimitris_Sean.pdf
[sola]: https://arxiv.org/abs/1711.02508v1
[smsckf]: https://arxiv.org/abs/1712.00036v3
[ov-paper]: https://yangyulin.net/papers/2020_icra_ov.pdf
[fej2]: https://yangyulin.net/papers/2022_icra_fej2.pdf
[riekf]: https://arxiv.org/abs/1702.07920v2
[eqvio]: https://arxiv.org/abs/2205.01980v3
[msceqf]: https://arxiv.org/abs/2311.11649v1
[poseonly]: https://arxiv.org/abs/2407.01888v1
[immediate]: https://arxiv.org/abs/2411.02028v1
[transform]: https://arxiv.org/abs/2603.24130v2
[vinsmono]: https://arxiv.org/abs/1708.03852v1
[orbslam]: https://arxiv.org/abs/2007.11898v2
[okvis]: https://arxiv.org/abs/2202.09199v2
[sqrt]: https://arxiv.org/abs/2109.02182v1
[init]: https://arxiv.org/abs/2511.18910v3
[rdic]: https://www.mdpi.com/2075-1702/14/9/1004
[popl]: https://arxiv.org/abs/2602.06425v1
[ov-prop]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp#L726-L775
[ov-update]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/StateHelper.cpp#L116-L196
[ov-quat]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/types/JPLQuat.h#L104-L125
[ov-jac]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/update/UpdaterHelper.cpp
[ov-feature-init]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/feat/FeatureInitializer.cpp
[ov-config]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/config/euroc_mav/estimator_config.yaml
[state]: ../spec/requirements/msckf/state.md
[propagate]: ../spec/requirements/msckf/propagation.md
[fej]: ../spec/requirements/msckf/consistency.md
[correct]: ../spec/requirements/msckf/correction.md
[clones]: ../spec/requirements/msckf/clones.md
[visual]: ../spec/requirements/msckf/visual-geometry.md
[constraint]: ../spec/requirements/msckf/constraints.md
[lifecycle]: ../spec/requirements/msckf/lifecycle.md
