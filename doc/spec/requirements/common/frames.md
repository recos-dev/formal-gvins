# V-FRAMES: Frames, units and attitude-error convention

**ID:** `V-FRAMES`

**Status:** Draft

**Applies to:** Local visual-inertial models and their sensor/frontend interfaces.

**Uses:** [Q-MUL](../math/quaternion/multiplication.md), [Q-ROT-COMPOSE](../math/quaternion/composition.md), [SO3-EXP](../math/so3/exp.md).

## Basis

The project uses the Hamilton convention and local attitude error in
[Sola, Quaternion kinematics for the error-state Kalman filter, v1](https://arxiv.org/abs/1711.02508v1),
sections 5--6. This requirement owns the conventions used by the
[MSCKF derivation](../../architecture/msckf-formulation.md); it does not select
a camera model or a GNSS reference frame.

## Required behavior

Interfaces and equations shall use the following meanings consistently:

| Symbol | Meaning |
|---|---|
| $W$ | Right-handed local odometry frame, gravity along positive $z$; yaw need not be north-aligned. |
| $B$ | Calibrated IMU frame. A different vehicle body frame requires an explicit extrinsic transform. |
| $C_c$ | Camera $c$ optical frame: $x$ right, $y$ down, $z$ forward. |
| $R_{AB}$ | Rotation mapping vector coordinates in $B$ to coordinates in $A$. |
| $p_{AB}$ | Position of the origin of $B$, expressed in $A$. |

A point and a free vector shall transform differently:

$$
p_f^A=R_{AB}p_f^B+p_{AB},\qquad v^A=R_{AB}v^B.
$$

Rotations shall use scalar-first Hamilton quaternions
$q=[q_0,q_1,q_2,q_3]^\mathsf T$. Composition shall obey
$q_{AC}=q_{AB}\otimes q_{BC}$ and $R_{AC}=R_{AB}R_{BC}$.
The local attitude error shall be right-multiplied:

$$
R_{WB}^{\mathrm{true}}=\hat R_{WB}\operatorname{Exp}([\delta\theta_B]_\times),
\qquad
q_{WB}^{\mathrm{true}}=\hat q_{WB}\otimes\operatorname{Exp}_q(\delta\theta_B).
$$

For extrinsics $(R_{BC_c},p_{BC_c})$, a camera pose derived from an IMU pose shall
include both rotation and translation:

$$
R_{WC_c}=R_{WB}R_{BC_c},\qquad
p_{WC_c}=p_{WB}+R_{WB}p_{BC_c}.
$$

An observation shall identify the calibration revision used to interpret it.
Extrinsics shall not be assumed to be identity, nor silently inverted. Stored
observations shall not be reinterpreted under a new calibration revision without
an explicit conversion or invalidation policy.

Estimator positions, velocities, specific force and angular rates shall use
metres, metres/second, metres/second squared and radians/second respectively.
Rotation errors shall use radians. Integration intervals shall use seconds;
timestamp representations shall declare their clock, epoch and unit before
conversion. A sensor measurement time shall not be replaced by callback arrival
time. Camera timestamps shall identify a declared exposure reference instant.

Every image-coordinate interface shall identify whether it carries raw pixels,
rectified/undistorted pixels or normalized coordinates, with the applicable
calibration and image geometry. Residuals and measurement covariance shall use
the same domain. Cropping, resizing or undistortion shall preserve that binding
through an explicit coordinate/calibration transformation. Internal ORB metadata
in library-specific units shall not silently enter estimator equations.

A future map or geodetic frame shall be distinct from $W$ and connected by an
explicit transform. The local frame shall not be called NED without establishing
north/east alignment. This requirement does not authorize map corrections to
overwrite the local estimator state.

## Acceptance and verification

Check point/vector transformation, composition and inverse consistency using
nonidentity rotations and nonzero camera lever arms. Verify the right-perturbation
sign in each derived Jacobian. Test coordinate/noise transformations and reject
or explicitly convert incompatible calibration/frame bindings. Native geometric
tests and exact identities do not establish physical calibration accuracy.

## Open decisions

Camera/distortion model, camera count, exposure reference and synchronization
accuracy require an acquisition profile. Map/GNSS frame alignment and calibration
change recovery require separate contracts. Their absence prevents claiming
those integrations complete, but does not change the conventions above.
