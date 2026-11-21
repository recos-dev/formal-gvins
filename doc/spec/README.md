# Specification Index

[The writing template](TEMPLATE.md) defines requirement identity, decomposition,
approval and evidence ownership. Each independently reviewable requirement has
one canonical file. Parent links express derivation; Uses links express contract
dependencies. Directory placement alone expresses neither.

This Draft set allocates numerical foundations, acquisition/frontend boundaries,
feature ownership and the proposed local MSCKF. It is not an implemented or
approved estimator. Profile values and open acceptance decisions remain explicit.
GNSS, mapping, relocalization and loop correction remain extension proposals.

For source/test pointers and remaining evidence gaps, use the
[requirements inventory](../analysis/requirements-inventory.md). The
[verification policy](verification.md) separates approval, implementation,
proofs, native tests and identified executions.

## Common contracts

| ID | Requirement | Status |
|---|---|---|
| `C-CPP-SUBSET` | [C++ language and portability subset](requirements/common/cpp-subset.md) | Draft |
| `C-NUMERICAL-PROFILE` | [Numerical execution profile](requirements/common/numerical-profile.md) | Draft |
| `C-INPUT-DOMAIN` | [Input-domain and caller obligations](requirements/common/input-domain.md) | Draft |
| `C-FAILURE` | [Numerical failure reporting and atomicity](requirements/common/failure.md) | Draft |
| `C-NUMERICAL-ACCEPTANCE` | [Numerical residual acceptance](requirements/common/numerical-acceptance.md) | Draft |
| `V-FRAMES` | [Frames, units and attitude-error convention](requirements/common/frames.md) | Draft |

## Reusable mathematics

| ID | Requirement | Status |
|---|---|---|
| `F-SCALAR` | [Checked scalar operations](requirements/math/scalar/operations.md) | Draft |
| `F-SOLVE` | [SPD linear solve](requirements/math/linalg/spd-solve.md) | Draft |
| `F-LINALG` | [Fixed matrix/vector operations](requirements/math/linalg/matrix-operations.md) | Draft |
| `F-NORMALIZE` | [Checked vector normalization](requirements/math/linalg/vector-normalization.md) | Draft |
| `F-BOUNDED` | [Bounded active matrices](requirements/math/linalg/bounded-matrices.md) | Draft |
| `F-CHOLESKY` | [Lower Cholesky factor](requirements/math/linalg/cholesky.md) | Draft |
| `F-TRIANGULAR` | [Lower-triangular solve](requirements/math/linalg/triangular-solve.md) | Draft |
| `F-WHITEN` | [Covariance whitening](requirements/math/linalg/whitening.md) | Draft |
| `F-QR` | [Orthogonal reduction and rank](requirements/math/linalg/qr-reduction.md) | Draft |
| `F-LEAST-SQUARES` | [Full-column-rank least squares](requirements/math/linalg/least-squares.md) | Draft |
| `F-NULLSPACE` | [Left-nullspace action](requirements/math/linalg/nullspace-projection.md) | Draft |
| `Q-IDENTITY` | [Quaternion identity](requirements/math/quaternion/identity.md) | Draft |
| `Q-MUL` | [Hamilton multiplication](requirements/math/quaternion/multiplication.md) | Draft |
| `Q-INVERSE` | [Quaternion inverse](requirements/math/quaternion/inverse.md) | Draft |
| `Q-NORMALIZE` | [Quaternion normalization](requirements/math/quaternion/normalization.md) | Draft |
| `Q-ROT-MATRIX` | [Quaternion rotation matrix](requirements/math/quaternion/rotation-matrix.md) | Draft |
| `Q-ROT-COMPOSE` | [Rotation composition](requirements/math/quaternion/composition.md) | Draft |
| `Q-ROTATE` | [Rotation action](requirements/math/quaternion/rotation-action.md) | Draft |
| `Q-SIGN` | [Quaternion sign equivalence](requirements/math/quaternion/sign-equivalence.md) | Draft |
| `SO3-HAT` | [Hat operator](requirements/math/so3/hat.md) | Draft |
| `SO3-EXP` | [Exponential map](requirements/math/so3/exp.md) | Draft |
| `SO3-LOG` | [Logarithmic map](requirements/math/so3/log.md) | Draft |
| `SO3-SMALL-ANGLE` | [Small-angle behavior](requirements/math/so3/small-angle.md) | Draft |
| `F-JR` | [SO(3) right Jacobian](requirements/math/so3/right-jacobian.md) | Draft |

## Acquisition and visual observations

| ID | Requirement | Status |
|---|---|---|
| `V-ORB` | [ORB extraction and association](requirements/vision/orb-frontend.md) | Draft |
| `V-STEREO` | [Calibrated stereo association](requirements/vision/stereo-association.md) | Draft |
| `V-LOCALIZE` | [Subpixel feature localization](requirements/vision/feature-localization.md) | Draft |
| `V-TRACK` | [Feature observation identity and ownership](requirements/vision/feature-tracks.md) | Draft |
| `V-OBSERVATION` | [Observation identity and admission](requirements/vision/observation-admission.md) | Draft |
| `V-ASSOCIATION` | [Track/clone association and lifetime](requirements/vision/track-association.md) | Draft |
| `V-CONSUMPTION` | [Once-only consumption](requirements/vision/observation-consumption.md) | Draft |
| `V-ACQUISITION` | [Sensor time and calibration binding](requirements/vision/acquisition.md) | Draft |
| `V-EXECUTION` | [Host frontend execution](requirements/vision/frontend-execution.md) | Draft |

V-TRACK retains its composition scope and allocates identity, association and
consumption to three children. V-ORB uses that boundary; it is not the parent of
the estimator's observation store.

## Local MSCKF capability

| ID | Requirement | Status |
|---|---|---|
| `SW-MSCKF` | [Local visual-inertial estimation](requirements/software/local-vio.md) | Draft |
| `M-STATE` | [Joint state and initial validity](requirements/msckf/state.md) | Draft |
| `M-PROPAGATE` | [Held-input joint propagation](requirements/msckf/propagation.md) | Draft |
| `M-CLONE` | [Pose cloning and covariance selection](requirements/msckf/clones.md) | Draft |
| `M-VISUAL` | [Feature geometry and Jacobians](requirements/msckf/visual-geometry.md) | Draft |
| `M-UNPROJECT` | [Calibrated pixel-to-bearing conversion](requirements/msckf/camera-unprojection.md) | Draft |
| `M-CONSTRAINT` | [Constraint elimination and acceptance](requirements/msckf/constraints.md) | Draft |
| `M-CORRECT` | [Joint correction and reset](requirements/msckf/correction.md) | Draft |
| `M-FEJ` | [First estimates and coordinate consistency](requirements/msckf/consistency.md) | Draft |
| `M-LIFECYCLE` | [Execution and transactions](requirements/msckf/lifecycle.md) | Draft |

SW-MSCKF owns composition and has eight direct children. Reusable mathematical
and common contracts remain roots referenced through Uses. Parent links allocate
obligations; calling an operation does not make the caller its requirement child.

## Development profiles

- [ORB frontend](profiles/orb-frontend.md): current CPU/CUDA settings, validation
  ranges, raw-image/stereo bindings, descriptor compatibility and unresolved quality/resource acceptance.
- [EuRoC acquisition](profiles/euroc.md): dataset conversion, coordinates/time,
  raw-image behavior, optional exact-timestamp stereo and estimator/scoring qualifications.
- [Local VINS](profiles/local-vins.md): proposed mono/CPU estimator bindings,
  raw-pixel camera model, initialization, capacities, propagation evaluation,
  FEJ/reset limits and updates.

These Draft profiles describe current development choices. They neither approve
flight settings nor waive the linked requirements' unresolved bounds.

## Reuse and evidence

The original reusable mathematical requirements retain IDs and operation semantics from
[formal-eskf revision 6e4ffb5](https://github.com/recos-dev/formal-eskf/tree/6e4ffb5bc5932804c177cbacfd5a10e3126f981a/docs/spec/requirements/math).
Project names, links and application scope are adapted. Active-size and MSCKF
allocations are local additions; F-SOLVE also covers active solves. The four common
numerical contracts are tailored from the
[same revision](https://github.com/recos-dev/formal-eskf/tree/6e4ffb5bc5932804c177cbacfd5a10e3126f981a/docs/spec/requirements/common):
ESKF state checks, configuration macros and historical evidence are not inherited.
The project code and documentation use [Apache-2.0](../../LICENSE).

Reuse does not approve these Draft requirements or import proof coverage. Evidence
must identify the local source revision, supported dimensions, scalar/backend
profile and target. Open decisions and unestablished numerical bounds remain
acceptance gaps. Existing conformance tests do not prove every profile or a
complete estimator.

## Architecture proposals

- [Application and Component Composition](architecture/application-composition.md)
  (Draft): application ownership, reusable VINS assembly, template boundaries and
  `.hpp`/`.cpp` responsibilities under C-CPP-SUBSET.
- [MSCKF Formulation](architecture/msckf-formulation.md) (Draft): state and frame
  conventions, IMU-pose clones, visual constraints, joint updates and proof scope.

Architecture and mathematical formulation proposals belong under `architecture/`.
They require explicit review before their decisions become baseline requirements.
Approval and verification remain separate: neither a commit nor an index entry
establishes either one.
