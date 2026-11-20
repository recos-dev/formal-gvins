# SW-MSCKF: Local visual-inertial estimation

**ID:** `SW-MSCKF`

**Status:** Draft

**Applies to:** Bounded local MSCKF with fixed calibration, known constant gravity, static point features and qualified global-shutter mono or synchronized-stereo input.

**Uses:** [C-CPP-SUBSET](../common/cpp-subset.md), [C-NUMERICAL-PROFILE](../common/numerical-profile.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md), [V-FRAMES](../common/frames.md), [V-ACQUISITION](../vision/acquisition.md), [V-TRACK](../vision/feature-tracks.md).

## Basis

The [project formulation](../../architecture/msckf-formulation.md) derives this
local-error model and its conventional FEJ/identity-reset approximation. This
allocation remains Draft and does not claim an implemented estimator or approved FEJ design.

## Required behavior

Given an accepted initial joint estimate and qualified time/calibration/noise
profile, the estimator shall propagate IMU inputs and incorporate eligible visual
constraints into one joint state/covariance. It shall preserve clone identity,
full cross-covariance, linearization-coordinate binding and once-only observation
consumption across the operations below. Invalid or failed updates shall not
publish partial estimate/cache/consumption state.
Transaction boundaries follow M-LIFECYCLE: a failed correction does not roll back
earlier successful propagation or augmentation.

Output shall identify its modeled time, local frame, calibration/configuration
binding and validity. No output before valid initialization shall be presented as
an accepted navigation estimate. Lack of accepted visual constraints shall not
fabricate corrections. Capacity, delay and loss/recovery policies must be explicit.
No flight accuracy or real-time guarantee follows from the mathematical model.

## Decomposition

| Child | Allocated obligation |
|---|---|
| [M-STATE](../msckf/state.md) | Joint coordinates, initial validity and bounded storage. |
| [M-PROPAGATE](../msckf/propagation.md) | Held-input nominal propagation, FEJ covariance propagation and analytical interval-equivalent process noise. |
| [M-CLONE](../msckf/clones.md) | Pose augmentation/removal with full covariance and identity. |
| [M-VISUAL](../msckf/visual-geometry.md) | Projection, triangulation and geometric Jacobians. |
| [M-CONSTRAINT](../msckf/constraints.md) | Whitening, feature elimination, gate and compression. |
| [M-CORRECT](../msckf/correction.md) | Joseph correction, all-pose injection/reset and publication. |
| [M-FEJ](../msckf/consistency.md) | First-estimate lifetime, full FEJ transition and linear-model gauge compatibility. |
| [M-LIFECYCLE](../msckf/lifecycle.md) | Time/order/capacity/recovery and observation transaction composition. |

## Acceptance and verification

Require applicable child and Uses evidence plus composed sequences: propagation
split at image times, clone growth/removal, feature loss, failed/retried corrections
and capacity boundaries. Exact algebra, checked source behavior, numerical residuals
and trajectory/consistency benchmarks are separate evidence. Frame/time/state/cache
identities must match across every boundary; child PASS counts alone are insufficient.

## Open decisions

Initialization/recovery algorithm, capacities, clock/noise qualification,
geometry/gate/FEJ limits and target resources remain explicit profile decisions.
Online calibration, GNSS, map feedback, relocalization and loop closure are outside
this capability and require separate contracts before integration.
