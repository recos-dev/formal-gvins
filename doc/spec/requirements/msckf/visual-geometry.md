# M-VISUAL: Feature geometry and visual Jacobians

**ID:** `M-VISUAL`

**Status:** Draft

**Applies to:** Static point features, fixed calibrated cameras and raw distorted pixel observations; the first profile uses pinhole projection with radial-tangential distortion.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [V-FRAMES](../common/frames.md), [V-ASSOCIATION](../vision/track-association.md), [F-LEAST-SQUARES](../math/linalg/least-squares.md), [M-FEJ](consistency.md).

## Basis

[Visual observation and Jacobians](../../architecture/msckf-formulation.md#visual-observation-and-jacobians)
derives the following equations in the project's Hamilton/right-local convention.

## Required behavior

For clone j and camera c, let `s=R_WB_j^T*(p_f^W-p_WB_j)` and
`y=R_BC^T*(s-p_BC)`. Prediction shall be `pi(y)=D_c([X/Z,Y/Z]^T)`,
where D_c is the declared camera's normalized-to-raw distortion/intrinsic map.
Residual `z-pi(y)` and its covariance shall use raw pixel coordinates. Require
valid positive depth and a qualified camera-model domain before evaluation.
Camera lever arm and rotation shall both participate. Fixed calibration includes
the lens model, coefficient ordering and raw image dimensions.

For ordinary geometric derivatives at one common point, use

$$
J_\pi=J_D\begin{bmatrix}1/Z&0&-X/Z^2\\0&1/Z&-Y/Z^2\end{bmatrix},\quad
J_D=\left.\frac{\partial D_c}{\partial(x,y)}\right|_{(x,y)=(X/Z,Y/Z)}.
$$

$$
H_{c_j}=J_\pi R_{BC}^\mathsf T[-R_{WB_j}^\mathsf T\ \ [s]_\times],\qquad
H_f=J_\pi R_{BC}^\mathsf T R_{WB_j}^\mathsf T.
$$

Place clone columns at their declared indices; other direct state columns are
zero with fixed calibration. This does not prohibit updates through cross-covariance.
For the consistency-aware filter, M-FEJ selects frozen points for both the
normalized projection and distortion derivatives; residuals and triangulation
refinement still use current estimates.

Triangulation shall invert D_c pointwise to obtain normalized bearings and use
the linked bearing-collinearity initializer with rank-aware least squares. The
inverse shall have a qualified domain, bounded work and a forward-reprojection
residual acceptance limit. An invalid or nonconvergent inverse shall not produce
a usable bearing. Raw pixels and derived bearings shall retain one observation
identity and shall not be fused as independent measurements.

Reject insufficient rank/parallax, invalid depth or unacceptable conditioning;
a frontend inlier is not a triangulation certificate. A configured
refinement shall have bounded iterations, declared step/convergence acceptance
and raw-pixel reprojection-noise weighting. Failed geometry/refinement shall not
publish a usable feature or partially assembled constraint. Temporary points shall remain
outside the filter state and carry no invented independent measurement prior.

## Acceptance and verification

Establish the complete projection/distortion Jacobian, including the positive
right-local attitude-skew sign and the zero-distortion pinhole limit. Check
pointwise inverse/forward round trips, inverse failures, nonzero radial/tangential
coefficients and off-axis points. Test finite differences with nonidentity
extrinsics/lever arms, multiple cameras, pure rotation, low parallax, behind-camera
points and bad calibration/domain bindings. Verify current/frozen evaluation
and refinement's raw-pixel objective, without claiming global convergence.
Numerical depth/parallax/conditioning, noise, inverse and iteration settings
remain profile decisions before this module can be accepted.

## Decomposition

[M-UNPROJECT](camera-unprojection.md) owns the calibrated inverse used by frontend geometry.
