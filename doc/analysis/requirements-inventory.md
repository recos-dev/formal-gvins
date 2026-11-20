# Requirements implementation and verification inventory

**Status:** Draft assessment; mappings are candidates, not acceptance results.

The [index](../spec/README.md) owns requirement discovery and Draft status;
[verification policy](../spec/verification.md) owns evidence acceptance. This
inventory maps each requirement once to current production symbols/test sources
and remaining work. It covers bounded linalg, the frontend and the EuRoC adapter;
each execution record must independently bind actual source content/revision.
No test or proof execution is claimed by this document.

## Scope

| Requirement group | Count |
|---|---:|
| Common contracts, including frames | 6 |
| Reusable mathematics | 24 |
| Acquisition, frontend and track ownership | 7 |
| Local MSCKF parent and children | 9 |
| Total / Draft / Approved | 46 / 46 / 0 |

All original IDs are retained. V-TRACK has three children and SW-MSCKF has eight;
these links allocate responsibility, not additional completed proofs. Architecture
and the two development profiles are not counted as requirements.

**Policy** means a cross-cutting obligation. **Present** means its numerical
operation exists, not that every clause/profile has accepted evidence. **Partial**
means only part of the boundary is implemented. **Planned** means the responsible
component/composition is absent. The corresponding counts are 6 policy, 24 present,
5 partial and 11 planned. Do not interpret these as a completion percentage.

No local Lean/ESBMC suite or requirement-to-execution registry is present. Imported
native tests do not transfer formal-eskf's proofs. Current mathematical acceptance
still needs exact identities, scoped source checks and justified numerical domains;
frontend tests do not establish physical association or navigation accuracy.

## Common contracts

| Requirement | Implementation scope | Source / test pointers | Review or evidence gap |
|---|---|---|---|
| [C-CPP-SUBSET](../spec/requirements/common/cpp-subset.md) | Policy | [Build targets](../../CMakeLists.txt), [bounded storage](../../include/formal_gvins/linalg/bounded.hpp), [backend test setup](../../tests/test_backend.hpp) | Review core/host boundary and capacity/stack obligations. Allocation-guard tests do not qualify full-capacity deployment resources. |
| [C-NUMERICAL-PROFILE](../spec/requirements/common/numerical-profile.md) | Policy | [Numeric test flags](../../CMakeLists.txt), [scalar IEEE cases](../../tests/test_scalar_standard.cpp) | Bind actual compiler/backend/libm, rounding and subnormal settings. Private numeric-test flags do not enforce those settings on downstream consumers. CPU/GPU image processing needs its own profile, not an assumed core profile. |
| [C-INPUT-DOMAIN](../spec/requirements/common/input-domain.md) | Policy | [Fixed operations](../../include/formal_gvins/linalg/operations.hpp), [active operations](../../include/formal_gvins/linalg/bounded.hpp), [Log](../../include/formal_gvins/so3/rotation_vector.hpp) | Discharge the new operations' active-index, rank and conditioning premises at their callers. Do not interpret this shared policy as repeated validation at every internal call. |
| [C-FAILURE](../spec/requirements/common/failure.md) | Policy | [Status](../../include/formal_gvins/status.hpp); [scalar](../../tests/test_scalar_standard.cpp), [SPD](../../tests/test_linalg_spd.cpp) and [bounded](../../tests/test_linalg_bounded.cpp) failure cases | Map each checked operation's precedence and aliases. Numerical atomicity does not establish frontend exception handling or a complete estimator transaction. |
| [C-NUMERICAL-ACCEPTANCE](../spec/requirements/common/numerical-acceptance.md) | Policy | Residual checks in [SPD](../../tests/test_linalg_spd.cpp), [bounded](../../tests/test_linalg_bounded.cpp) and [right-Jacobian tests](../../tests/test_so3_jacobian.cpp) | Review domains, residual definitions and tolerance rationale per property/profile. Constants in tests are not automatically accepted deployment error bounds. |
| [V-FRAMES](../spec/requirements/common/frames.md) | Policy | [Observation metadata](../../include/formal_gvins/vision/observation.hpp), [EuRoC reader](../../src/dataset/euroc.cpp), [extrinsic/time fixtures](../../tests/test_euroc.cpp) | Bind exposure time, clock alignment, image/noise coordinates and dataset-reference-to-W conversion in acquisition/scoring profiles. Reading raw ground truth does not align it to W or establish physical calibration accuracy. |

## Mathematics

| Requirement | Implementation scope | Source / test pointers | Review or evidence gap |
|---|---|---|---|
| [F-SCALAR](../spec/requirements/math/scalar/operations.md) | Present | [Scalar API](../../include/formal_gvins/scalar/math.hpp); [tests](../../tests/test_scalar_standard.cpp): domains, signed zero, aliases, injected non-finite backend results | Register checked-control-flow obligations separately from StandardMath accuracy and target libm evidence. |
| [F-LINALG](../spec/requirements/math/linalg/matrix-operations.md) | Present | [Matrix](../../include/formal_gvins/linalg/matrix.hpp), [operations](../../include/formal_gvins/linalg/operations.hpp); [fixed tests](../../tests/test_linalg.cpp) | Review owning/eager values, raw versus checked boundaries, aliases and backend residual domains. |
| [F-NORMALIZE](../spec/requirements/math/linalg/vector-normalization.md) | Present | `linalg::normalize` in [operations](../../include/formal_gvins/linalg/operations.hpp); [fixed tests](../../tests/test_linalg.cpp) | Complete threshold/error-order, alias and numerical residual evidence; quaternion remapping is separate. |
| [F-BOUNDED](../spec/requirements/math/linalg/bounded-matrices.md) | Present | [bounded.hpp](../../include/formal_gvins/linalg/bounded.hpp); [bounded tests](../../tests/test_linalg_bounded.cpp) | Capacity/index proofs, full-capacity stack/resource qualification and numerical domains remain open. |
| [F-CHOLESKY](../spec/requirements/math/linalg/cholesky.md) | Present | `cholesky` in [cholesky.hpp](../../include/formal_gvins/linalg/cholesky.hpp); [bounded tests](../../tests/test_linalg_bounded.cpp) | Exact factor theorem, failure paths and qualified reconstruction/conditioning bounds. |
| [F-TRIANGULAR](../spec/requirements/math/linalg/triangular-solve.md) | Present | `solve_lower` in [cholesky.hpp](../../include/formal_gvins/linalg/cholesky.hpp); [bounded tests](../../tests/test_linalg_bounded.cpp) | Structural/zero-divisor precedence, substitution and finite-precision residual/alias evidence. |
| [F-WHITEN](../spec/requirements/math/linalg/whitening.md) | Present | `whiten` in [cholesky.hpp](../../include/formal_gvins/linalg/cholesky.hpp); [bounded tests](../../tests/test_linalg_bounded.cpp) | Exact covariance/norm identities and caller row/noise binding; native factor tests do not qualify physical covariance. |
| [F-SOLVE](../spec/requirements/math/linalg/spd-solve.md) | Present | [Fixed solve](../../include/formal_gvins/linalg/solve.hpp), [active solve](../../include/formal_gvins/linalg/cholesky.hpp); [fixed tests](../../tests/test_linalg_spd.cpp), [active tests](../../tests/test_linalg_bounded.cpp) | Review added active-shape precedence alongside fixed behavior; conditioning/error domains and local proof evidence remain open. |
| [F-QR](../spec/requirements/math/linalg/qr-reduction.md) | Present | `qr` in [qr.hpp](../../include/formal_gvins/linalg/qr.hpp); [bounded tests](../../tests/test_linalg_bounded.cpp) | Orthogonal/permutation proofs, threshold semantics and numerical residual bounds; no condition-number claim. |
| [F-LEAST-SQUARES](../spec/requirements/math/linalg/least-squares.md) | Present | `least_squares` in [qr.hpp](../../include/formal_gvins/linalg/qr.hpp); [bounded tests](../../tests/test_linalg_bounded.cpp) | Minimizer/column-order theorem and conditioning-dependent error bounds; no pseudoinverse fallback. |
| [F-NULLSPACE](../spec/requirements/math/linalg/nullspace-projection.md) | Present | `project_left_nullspace` in [qr.hpp](../../include/formal_gvins/linalg/qr.hpp); [bounded tests](../../tests/test_linalg_bounded.cpp) | Annihilation/information theorem and numerical residuals; triangulation and statistical acceptance remain callers. |
| [Q-IDENTITY](../spec/requirements/math/quaternion/identity.md) | Present | [UnitQuaternion](../../include/formal_gvins/so3/unit_quaternion.hpp); [SO(3) tests](../../tests/test_so3.cpp): `test_construction` | Add local evidence for both identity laws over the stated domain; default-value checks alone are narrower. |
| [Q-MUL](../spec/requirements/math/quaternion/multiplication.md) | Present | [Hamilton coefficients](../../include/formal_gvins/so3/unit_quaternion.hpp); [SO(3) tests](../../tests/test_so3.cpp): `test_multiplication_and_inverse` | Prove coefficient correspondence and unit closure separately; document raw arithmetic's finite-precision domain. |
| [Q-INVERSE](../spec/requirements/math/quaternion/inverse.md) | Present | [Conjugation](../../include/formal_gvins/so3/unit_quaternion.hpp); [SO(3) tests](../../tests/test_so3.cpp): `test_multiplication_and_inverse` | Establish two-sided inverse and matrix-transpose identities under the unit premise. |
| [Q-NORMALIZE](../spec/requirements/math/quaternion/normalization.md) | Present | [Checked construction](../../include/formal_gvins/so3/unit_quaternion.hpp); [SO(3) tests](../../tests/test_so3.cpp): `test_construction` | Complete threshold equality/overlap, in-place and overflow cases; establish exact normalization and profile-specific residual bounds. Current tests exercise only part of this contract. |
| [Q-ROT-MATRIX](../spec/requirements/math/quaternion/rotation-matrix.md) | Present | [SO(3) operations](../../include/formal_gvins/so3/operations.hpp); [SO(3) tests](../../tests/test_so3.cpp): `test_reference_rotation_matrix` | Register coefficient and rotation properties under unit premises; a reference fixture is not their universal proof. |
| [Q-ROT-COMPOSE](../spec/requirements/math/quaternion/composition.md) | Present | [compose_normalized](../../include/formal_gvins/so3/operations.hpp); [SO(3) tests](../../tests/test_so3.cpp): `test_multiplication_and_inverse` | Add explicit failed-composition/status and both output-alias cases; prove order and connect normalization to its own contract. |
| [Q-ROTATE](../spec/requirements/math/quaternion/rotation-action.md) | Present | [Rotation actions](../../include/formal_gvins/so3/operations.hpp); [SO(3) tests](../../tests/test_so3.cpp): `test_rotation_action` | Establish action, recovery and norm preservation; qualify numerical residuals separately. |
| [Q-SIGN](../spec/requirements/math/quaternion/sign-equivalence.md) | Present | [Comparison functions](../../include/formal_gvins/so3/operations.hpp); [SO(3) tests](../../tests/test_so3.cpp): sign/construction and principal-Log cases | Review the new coefficient-tolerance clause; add threshold/invalid-tolerance cases and exact sign-equivalence evidence. |
| [SO3-HAT](../spec/requirements/math/so3/hat.md) | Present | [hat](../../include/formal_gvins/so3/operations.hpp); [SO(3) tests](../../tests/test_so3.cpp): `test_hat` | Prove arbitrary-vector cross-product/skew identities and bind unchecked coefficient evaluation. |
| [SO3-EXP](../spec/requirements/math/so3/exp.md) | Present | [Maps](../../include/formal_gvins/so3/rotation_vector.hpp); [SO(3) tests](../../tests/test_so3.cpp): `test_exp_and_log`, `test_exp_rotation_matrix`, `test_exp_failure_atomicity` | Exact map/unit/zero proofs, scalar-failure propagation and accepted numerical domains remain separate obligations. |
| [SO3-LOG](../spec/requirements/math/so3/log.md) | Present | [Maps](../../include/formal_gvins/so3/rotation_vector.hpp); [SO(3) tests](../../tests/test_so3.cpp): `test_exp_and_log` | Add explicit pi-boundary and checked-failure evidence; prove principal range/local inverse only under the unit premise. |
| [SO3-SMALL-ANGLE](../spec/requirements/math/so3/small-angle.md) | Present | [Map branches](../../include/formal_gvins/so3/rotation_vector.hpp); [SO(3) tests](../../tests/test_so3.cpp): small-vector cases | Add exact threshold/adjacent-representable-value cases for Exp and Log, remainder bounds and branch-transition analysis. Keep this shared branch contract referenced by both maps. |
| [F-JR](../spec/requirements/math/so3/right-jacobian.md) | Present | [right_jacobian](../../include/formal_gvins/so3/right_jacobian.hpp); [tests](../../tests/test_so3_jacobian.cpp): series reference, differential, cutoff and failures | Local exact differential/Taylor proofs and rounding/libm bounds are absent; finite differences are supporting tests. |

## Acquisition and visual observations

| Requirement | Implementation scope | Source / test pointers | Review or evidence gap |
|---|---|---|---|
| [V-ACQUISITION](../spec/requirements/vision/acquisition.md) | Partial | [EuRoC reader](../../src/dataset/euroc.cpp); [fixtures](../../tests/test_euroc.cpp) | Parsing/undistortion exist; exposure/clock/noise qualification, interval binding and truth scoring do not. |
| [V-EXECUTION](../spec/requirements/vision/frontend-execution.md) | Partial | [OrbExtractor](../../src/vision/orb.cpp), tracker; [ORB](../../tests/test_orb.cpp) and [tracker tests](../../tests/test_tracker.cpp) | Blocking ownership/availability exist; device-error, allocation recovery and deployment timing/overload qualification remain gaps. |
| [V-ORB](../spec/requirements/vision/orb-frontend.md) | Partial | [Extractor](../../src/vision/orb.cpp), [tracker](../../src/vision/tracker.cpp); [extractor tests](../../tests/test_orb.cpp), [tracker tests](../../tests/test_tracker.cpp) | Review publication-boundary and caller-owned descriptor compatibility clauses; resolve CPU/CUDA recipe difference and quality/resource bounds. |
| [V-TRACK](../spec/requirements/vision/feature-tracks.md) | Partial | [Image/feature identities](../../include/formal_gvins/vision/observation.hpp), `OrbTracker::process/reset`; [tracker tests](../../tests/test_tracker.cpp) | Review all three children plus composed admission/association/consumption sequences. Frontend continuity is only part of this parent. |
| [V-OBSERVATION](../spec/requirements/vision/observation-admission.md) | Partial | [ImageIdentity/FeatureObservation](../../include/formal_gvins/vision/observation.hpp), tracker; [tests](../../tests/test_tracker.cpp) | Frontend portion only; observation-store conflict handling and cross-camera admission have no implementation. |
| [V-ASSOCIATION](../spec/requirements/vision/track-association.md) | Planned | No local track-history/clone component. | Implement stable time/clone links, capacity and cross-owner lifetime after runtime profile review. |
| [V-CONSUMPTION](../spec/requirements/vision/observation-consumption.md) | Planned | No estimator consumption ledger/transaction. | Implement once-only records and complete correction/cache/ledger publication; exercise failed/retried updates. |

## Local MSCKF

| Requirement | Implementation scope | Source / test pointers | Review or evidence gap |
|---|---|---|---|
| [SW-MSCKF](../spec/requirements/software/local-vio.md) | Planned | [Formulation](../spec/architecture/msckf-formulation.md); no joint estimator. | Review eight child allocations, dependencies and composition acceptance before implementation. |
| [M-STATE](../spec/requirements/msckf/state.md) | Planned | No joint-state/initializer implementation. | Choose initialization/prior validity, capacities and numerical profile; verify layout and invalid-candidate preservation. |
| [M-PROPAGATE](../spec/requirements/msckf/propagation.md) | Planned | Formulation only; numerical primitives are available. | Choose bounded evaluation and interval/noise profile; establish held-input, derivative and matched-noise evidence. |
| [M-CLONE](../spec/requirements/msckf/clones.md) | Planned | No clone manager; bounded gather is only a primitive. | Implement identity/covariance/cache augmentation and removal; verify PSD and all cross-blocks. |
| [M-VISUAL](../spec/requirements/msckf/visual-geometry.md) | Planned | No estimator triangulation/Jacobian module; frontend essential-matrix screening is separate. | Review geometry/noise/refinement profile; prove derivatives and test geometric degeneracy. |
| [M-CONSTRAINT](../spec/requirements/msckf/constraints.md) | Planned | No assembled feature-update pipeline; whitening/QR/nullspace primitives exist. | Bind rank/conditioning/gate/compression choices; verify joint transforms and statistical premises. |
| [M-CORRECT](../spec/requirements/msckf/correction.md) | Planned | No joint correction; SPD/SO(3) primitives exist. | Implement Joseph/all-pose reset and atomic publication; prove composition and qualify numerical residuals. |
| [M-FEJ](../spec/requirements/msckf/consistency.md) | Planned | Project derivation only; no FEJ cache/transport implementation. | Owner review of proposal, correction/conditioning limits, exact gauge/transport proofs and statistical studies. |
| [M-LIFECYCLE](../spec/requirements/msckf/lifecycle.md) | Planned | No local-estimator runtime. | Bind ordering/capacity/loss/recovery policies; implement and verify full transaction sequences. |

## Review order and unresolved decisions

1. Review numerical contracts and the fixed/active distinction. In particular,
   vector/quaternion normalization have different validation order/statuses,
   numerical rank is not conditioning, and whitening is not an SPD inverse action.
2. Review acquisition/frontend and V-TRACK's three children. Resolve the documented
   [CPU/CUDA descriptor recipe difference](orb-frontend-settings.md) before claiming
   a matched-settings speedup. Qualify exposure/clock, noise and resource profiles
   before estimator integration; current development defaults are not flight limits.
3. Review SW-MSCKF's eight children against the formulation. Initialization,
   propagation evaluation/error bounds, geometry/gate/FEJ limits, capacities and
   lifecycle policies remain explicit open decisions, not implicit implementation
   defaults. The FEJ design is a project proposal, not an imported OpenVINS guarantee.
4. Bind reviewed claims to individual test/proof cases and identified executions
   under the verification policy. Complete the planned [module benchmarks](../roadmap.md#b-01-per-module-unit-benchmarks)
   with defined metrics/references; do not use frontend inlier counts as trajectory
   ground truth. New code/profile changes require impact review of these mappings.

[GNSS and mapping](../spec/architecture/msckf-map-gnss.md) remain extension work,
including frame/alignment uncertainty, timing, map lifetime and correlated feedback.
Their unsettled models are not invented here or made prerequisites for accepting
an independent numerical primitive. Requirements and profiles remain Draft until
explicit owner review; closing documentation gaps does not close evidence gaps.
