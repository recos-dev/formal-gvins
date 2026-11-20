# M-STATE: Joint state and initial validity

**ID:** `M-STATE`

**Status:** Draft

**Applies to:** The calibrated local MSCKF profile and its initialization boundary.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [V-FRAMES](../common/frames.md), [F-BOUNDED](../math/linalg/bounded-matrices.md), [C-INPUT-DOMAIN](../common/input-domain.md).

## Basis

[State and measurement ownership](../../architecture/msckf-formulation.md#state-and-measurement-ownership)
defines IMU-pose clones with a right-local attitude error.

## Required behavior

The current nominal state shall be `(p_WB,v_WB,q_WB,b_a,b_g)` with error order
`(delta p_W,delta v_W,delta theta_B,delta b_a,delta b_g)`. Each clone shall store
`(p_WB_j,q_WB_j)` with error order `(delta p_j_W,delta theta_B_j)`.
For N clones the nominal coefficient count is `16+7N` and the error/covariance
dimension is `D=15+6N`. Clones shall be identified by measurement time and stable
identity; their active block order shall be explicit. Gravity and camera calibration
are fixed profile inputs, not estimated state components in this baseline.

One joint covariance shall retain IMU/clone and clone/clone cross-correlations.
Its exact premise is symmetric positive semidefinite, not strictly positive
definite. Temporary feature points, descriptors, track histories and frozen first
estimates shall not add independent physical states or covariance dimensions. P follows
M-FEJ/M-CORRECT's first-order covariance and identity-reset approximation; it is
not claimed to be an exactly transported nonlinear physical-error covariance.

Initialization shall establish finite state, qualified quaternion norm residuals,
matching covariance/layout and declared frame/time/calibration/bias priors before
marking the estimate valid. The initializer owns PSD/physical prior justification;
finiteness or positive diagonal checks alone do not establish it. Invalid initial
candidates shall not replace a valid estimate. Global yaw need not be known or
north-aligned; the declared gauge and uncertainty shall not be hidden by naming W
NED. Insufficient motion for an initialization policy shall not produce a falsely
accepted result.

State, covariance and workspaces shall have declared maximum sizes and valid active
counts. Index/layout mappings shall change consistently with clone operations.

## Acceptance and verification

Check zero/max clones, block correspondence and initialization rejection atomicity.
Establish the coordinate dimensions and PSD premises separately from numerical
norm/symmetry residuals. Test nonidentity initial orientation and nonzero biases,
and verify that clone indexing cannot alias unrelated state blocks.

## Open decisions

Initialization algorithm, prior values, unit/symmetry tolerances, capacities and
loss/reinitialization policy require review. This contract states the validity
boundary, not an implemented automatic initializer.
