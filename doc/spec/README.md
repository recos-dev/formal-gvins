# Specification Index

[The writing template](TEMPLATE.md) defines requirement identity, decomposition,
approval and evidence ownership. Each independently reviewable requirement has
one canonical file. Parent links express derivation; Uses links express contract
dependencies. Directory placement alone expresses neither.

This initial set covers numerical foundations, coordinate conventions and the
ORB/feature-track boundary. It is not a complete MSCKF or SLAM specification.
Joint-state transitions, QR/nullspace operations, consistency policy, map
management, relocalization and loop correction still require allocated contracts.

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

## Visual observations

| ID | Requirement | Status |
|---|---|---|
| `V-ORB` | [ORB extraction and temporal association](requirements/vision/orb-frontend.md) | Draft |
| `V-TRACK` | [Feature observation identity and ownership](requirements/vision/feature-tracks.md) | Draft |

Reusable primitives and boundary contracts are independent roots here. Uses links
identify dependencies; they do not claim a completed software-capability
decomposition. New capability parents must allocate their own composition behavior
when that scope is defined.

## Reuse and evidence

The mathematical requirements retain IDs and operation semantics from
[formal-eskf revision 6e4ffb5](https://github.com/recos-dev/formal-eskf/tree/6e4ffb5bc5932804c177cbacfd5a10e3126f981a/docs/spec/requirements/math).
Project names, links and application scope are adapted. The four common numerical
contracts are tailored from the
[same revision](https://github.com/recos-dev/formal-eskf/tree/6e4ffb5bc5932804c177cbacfd5a10e3126f981a/docs/spec/requirements/common):
ESKF state checks, configuration macros and historical evidence are not inherited.
The project code and documentation use [Apache-2.0](../../LICENSE).

Reuse does not approve these Draft requirements or import proof coverage. Evidence
must identify the local source revision, supported dimensions, scalar/backend
profile and target. Open decisions and unestablished numerical bounds remain
acceptance gaps. Existing conformance tests do not prove every profile or a
complete estimator.

## Architecture proposals

- [MSCKF Formulation](architecture/msckf-formulation.md) (Draft): state and frame
  conventions, IMU-pose clones, visual constraints, joint updates and proof scope.

Architecture and mathematical formulation proposals belong under `architecture/`.
They require explicit review before their decisions become baseline requirements.
Approval and verification remain separate: neither a commit nor an index entry
establishes either one.
