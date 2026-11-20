# M-VISUAL: Feature geometry and visual Jacobians

**ID:** `M-VISUAL`

**Status:** Draft

**Applies to:** Static point features, fixed calibrated pinhole cameras and undistorted pixel observations.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [V-FRAMES](../common/frames.md), [V-ASSOCIATION](../vision/track-association.md), [F-LEAST-SQUARES](../math/linalg/least-squares.md), [M-FEJ](consistency.md).

## Basis

[Visual observation and Jacobians](../../architecture/msckf-formulation.md#visual-observation-and-jacobians)
derives the following equations in the project's Hamilton/right-local convention.

## Required behavior

For clone j and camera c, let `s=R_WB_j^T*(p_f^W-p_WB_j)` and
`y=R_BC^T*(s-p_BC)`. Prediction shall be
`pi(y)=[fx*X/Z+cx, fy*Y/Z+cy]^T`, residual `z-pi(y)`, in the observation's
undistorted-pixel/noise domain. Require valid positive depth under a declared
bound before division. Camera lever arm and rotation shall both participate.

For ordinary geometric derivatives at one common point, use

$$
J_\pi=\begin{bmatrix}f_x/Z&0&-f_xX/Z^2\\0&f_y/Z&-f_yY/Z^2\end{bmatrix},
\quad H_{c_j}=J_\pi R_{BC}^\mathsf T[-R_{WB_j}^\mathsf T\ \ [s]_\times],
\quad H_f=J_\pi R_{BC}^\mathsf T R_{WB_j}^\mathsf T.
$$

Place clone columns at their declared indices; other direct state columns are
zero with fixed calibration. This does not prohibit updates through cross-covariance.
For the consistency-aware filter, M-FEJ selects frozen Jacobian points/transport;
residuals and triangulation refinement still use current estimates.

Triangulation shall use the linked bearing-collinearity initializer with rank-aware
least squares. Reject insufficient rank/parallax, invalid depth or unacceptable
conditioning; a frontend inlier is not a triangulation certificate. A configured
refinement shall have bounded iterations, declared step/convergence acceptance
and reprojection-noise weighting. Failed geometry/refinement shall not publish a
usable feature or partially assembled constraint. Temporary points shall remain
outside the filter state and carry no invented independent measurement prior.

## Acceptance and verification

Establish projection/Jacobian derivatives, including the positive right-local
attitude-skew sign. Test finite differences with nonidentity extrinsics/lever arms,
multiple cameras, pure rotation, low parallax, behind-camera points and bad
calibration. Verify refinement's objective/acceptance, without claiming global
convergence. Numerical depth/parallax/conditioning, noise and iteration settings
remain profile decisions before this module can be accepted.
