# MSCKF Formulation

**Status:** Draft

## Scope and requirements

This is a proposed mathematical baseline for the local MSCKF in formal-gvins,
awaiting owner review. It does not describe an implemented MSCKF or establish
proof coverage. The [requirement index](../README.md) owns the allocated contracts;
remaining equations here are derivations/proposals, not approved requirements.

The proposed first profile uses a calibrated IMU, fixed camera intrinsics and
extrinsics, known camera/IMU clock alignment, global-shutter images, a constant
known gravity vector and static point features. It supports monocular or
synchronized stereo observations. This local-filter derivation excludes online
calibration, persistent landmark states inside the filter, loop-correction feedback
and raw-GNSS clock/ambiguity states. A separate sparse map for relocalization and
loop closure is a project direction; its ownership and coupling require separate
contracts. These exclusions do not prohibit a long-term map outside the MSCKF.

Covariance relations use a local first-order error model. They do not guarantee
accuracy for arbitrary initialization errors or unmodelled motion/noise.

The numerical core follows [C-CPP-SUBSET](../requirements/common/cpp-subset.md).
The deployment under consideration is an ARM Linux companion computer such as
Orin Nano, with an independent flight controller. The proposed first frontend is
[ORB](../requirements/vision/orb-frontend.md), with observation ownership defined
by [V-TRACK](../requirements/vision/feature-tracks.md). Middleware and
flight-controller integration remain unspecified.

## Frames and rotation convention

[V-FRAMES](../requirements/common/frames.md) owns frame symbols, units,
extrinsic direction, scalar-first Hamilton quaternions and right-local attitude
error. This derivation uses those conventions throughout. $[a]_\times$ is the
[hat operator](../requirements/math/so3/hat.md). $\operatorname{Exp}$ denotes the
matrix exponential; $\operatorname{Exp}_q$ and its zero limit follow
[SO3-EXP](../requirements/math/so3/exp.md).

These are Sola's local-error conventions [1]; OpenVINS uses different quaternion
conventions [3], so its Jacobian signs and multiplication order cannot be copied
unchanged.

## State and measurement ownership

The evolving IMU nominal state and error coordinates are

$$
x_I=(p_{WB},v_{WB},q_{WB},b_a,b_g),\qquad
\delta x_I=
\begin{bmatrix}\delta p_W\\\delta v_W\\\delta\theta_B\\\delta b_a\\\delta b_g\end{bmatrix}
\in\mathbb R^{15}.
$$

At each retained image time $t_j$, store an IMU pose clone:

$$
x_{c_j}=(p_{WB_j},q_{WB_j}),\qquad
\delta x_{c_j}=\begin{bmatrix}\delta p_j^W\\\delta\theta_{B_j}\end{bmatrix}.
$$

The joint state contains the current IMU state followed by the clones in time
order. Its nominal storage has $16+7N$ coefficients, including unit-quaternion
constraints; its error dimension is $D=15+6N$:

$$
\delta x=\begin{bmatrix}\delta x_I\\\delta x_{c_1}\\\vdots\\\delta x_{c_N}\end{bmatrix},
\qquad
P=\begin{bmatrix}P_{II}&P_{IC}\\P_{CI}&P_{CC}\end{bmatrix}\in\mathbb R^{D\times D}.
$$

Cross-covariances belong to this single joint estimate. Clones do not carry
their own velocity or bias states. The assumed known gravity is
$g_W=[0,0,g]^\mathsf T$, $g>0$; unlike Solà's gravity-estimating example,
there is no $\delta g$ in this profile.

IMU-pose cloning follows the representation used by OpenVINS [4]. With known
fixed extrinsics, the corresponding historical camera pose is

$$
R_{WC_{cj}}=R_{WB_j}R_{BC_c},\qquad
p_{WC_{cj}}=p_{WB_j}+R_{WB_j}p_{BC_c}.
$$

Thus an IMU pose in the world is the stored variable; the camera pose in the
world is derived. The camera pose relative to the IMU is the fixed extrinsic,
not a trajectory. Choosing camera-pose clones instead would require transforming
augmentation, observation and covariance coordinates consistently. Synchronized
stereo observations share one IMU clone and use different camera extrinsics.

Feature tracks are measurement records outside the EKF state. A record associates
a feature ID, camera ID, exposure timestamp/clone identity, observed image
coordinates and their noise model. Frontend tracking data has separate ownership.
Triangulated 3D points are temporary estimates used to build constraints, not
persistent landmark states with their own EKF covariance.

## IMU propagation

Let $f_m$ and $\omega_m$ be calibrated specific-force and angular-rate
measurements in $B$. Define $a=f_m-\hat b_a$ and
$\omega=\omega_m-\hat b_g$. The nominal model is

$$
\dot{\hat p}_{WB}=\hat v_{WB},\quad
\dot{\hat v}_{WB}=\hat R_{WB}a+g_W,\quad
\dot{\hat q}_{WB}=\tfrac12\hat q_{WB}\otimes[0,\omega],\quad
\dot{\hat b}_a=\dot{\hat b}_g=0.
$$

Under the stated error coordinates, the first-order continuous error model is
$\delta\dot x_I=F_I\delta x_I+G_In$, with $3\times3$ blocks:

$$
F_I=\begin{bmatrix}
0&I&0&0&0\\
0&0&-\hat R_{WB}[a]_\times&-\hat R_{WB}&0\\
0&0&-[\omega]_\times&0&-I\\
0&0&0&0&0\\
0&0&0&0&0
\end{bmatrix},\qquad
G_I=\begin{bmatrix}
0&0&0&0\\
-\hat R_{WB}&0&0&0\\
0&-I&0&0\\
0&0&I&0\\
0&0&0&I
\end{bmatrix}.
$$

The noise order is $n=(n_a,n_g,n_{ba},n_{bg})$, with continuous spectral density
defined by $E[n(t)n(\tau)^\mathsf T]=Q_c\delta(t-\tau)$.
Bias driving noises represent random walks. Sensor-noise density and per-sample
variance are distinct inputs; formal-eskf's sampled-variance parameters cannot
be substituted for $Q_c$ without a sampling-model conversion.

The ideal transition and discrete noise covariance satisfy

$$
\dot\Phi(t,t_k)=F_I(t)\Phi(t,t_k),\qquad \Phi(t_k,t_k)=I,
$$

$$
Q_d=\int_{t_k}^{t_{k+1}}
\Phi(t_{k+1},\tau)G_I(\tau)Q_cG_I(\tau)^\mathsf T
\Phi(t_{k+1},\tau)^\mathsf T\,d\tau.
$$

For $\Phi=\Phi(t_{k+1},t_k)$, the complete propagation is

$$
P^-=\begin{bmatrix}
\Phi P_{II}^+\Phi^\mathsf T+Q_d&\Phi P_{IC}^+\\
P_{CI}^+\Phi^\mathsf T&P_{CC}^+
\end{bmatrix}.
$$

Historical nominal poses remain unchanged during propagation. Their correlation
with the current IMU state does not remain unchanged. The discrete integrator
and its corresponding transition/noise approximations remain an open decision;
the integral above is a mathematical definition, not an implemented algorithm.

## Clone augmentation

Propagate to the corrected image timestamp before cloning the current IMU pose.
With fixed time alignment, let $J_c\in\mathbb R^{6\times D}$ select the current
position and local attitude error:

$$
J_c=\begin{bmatrix}
I&0&0&0&0&0_{3\times6N}\\
0&0&I&0&0&0_{3\times6N}
\end{bmatrix},\qquad
P_{\mathrm{aug}}=
\begin{bmatrix}
P&PJ_c^\mathsf T\\
J_cP&J_cPJ_c^\mathsf T
\end{bmatrix}.
$$

This deterministic duplication adds no independent process noise. It can make
the covariance singular: the required exact property is positive semidefiniteness,
not strict positive definiteness. An uncertain time offset would require an
extended state and augmentation Jacobian; it is excluded here.

## Visual observation and Jacobians

For a static feature at $p_f^W$, camera $c$, and clone $j$, define

$$
s_j=R_{WB_j}^\mathsf T(p_f^W-p_{WB_j}),\qquad
y_{fcj}=R_{BC_c}^\mathsf T(s_j-p_{BC_c}).
$$

The provisional observation uses undistorted pixel coordinates:

$$
z_{fcj}=\pi_c(y_{fcj})+n_{fcj},\qquad
\pi_c([X,Y,Z]^\mathsf T)=
\begin{bmatrix}f_xX/Z+c_x\\f_yY/Z+c_y\end{bmatrix}.
$$

Use camera-specific calibrated intrinsics. A raw-pixel distortion model is an
alternative requiring its own Jacobian; do not compare raw distorted pixels to
an undistorted prediction. Noise covariance must be expressed in the same
coordinates as the residual, including any undistortion transformation.

At a valid linearization point with positive depth,

$$
J_\pi=\begin{bmatrix}
f_x/Z&0&-f_xX/Z^2\\
0&f_y/Z&-f_yY/Z^2
\end{bmatrix}.
$$

For clone error order $(\delta p_j^W,\delta\theta_{B_j})$, the Jacobians are

$$
H_{c_j}=J_\pi R_{BC_c}^\mathsf T
\begin{bmatrix}-\hat R_{WB_j}^\mathsf T&[\hat s_j]_\times\end{bmatrix},
\qquad
H_{f_j}=J_\pi R_{BC_c}^\mathsf T\hat R_{WB_j}^\mathsf T.
$$

The positive attitude-skew sign follows from the right-local perturbation.
Insert $H_{c_j}$ in the corresponding clone columns of the complete Jacobian;
the other direct state columns are zero for fixed calibration. Cross-covariance
still allows the measurement to correct current velocity, attitude and biases.

Estimate the feature temporarily by multi-view triangulation, with optional
bounded refinement of

$$
\hat p_f^W\approx\arg\min_p
\sum_{c,j}\|z_{fcj}-\pi_c(y_{fcj}(p))\|_{V_{fcj}^{-1}}^2.
$$

This equation defines an objective, not guaranteed convergence or a globally
optimal solution. Positive depth, adequate parallax and conditioning require
explicit acceptance bounds. At chosen nominal/feature linearization points,
stacking the feature's $m$ two-dimensional observations gives

$$
r_f=z_f-h_f(\hat x,\hat p_f)
\simeq H_x\delta x+H_f\delta p_f+n_f,
\quad H_x\in\mathbb R^{2m\times D},\quad H_f\in\mathbb R^{2m\times3}.
$$

These are ordinary differential Jacobians. An FEJ implementation must specify
which estimates evaluate each factor; that policy is not fixed by this equation.

## Feature elimination, gating and compression

Let $V_f=\operatorname{Cov}(n_f)\succ0$, with Cholesky factor $V_f=LL^\mathsf T$.
Whiten by triangular solves:

$$
\tilde r=L^{-1}r_f,\qquad
\tilde H_x=L^{-1}H_x,\qquad
\tilde H_f=L^{-1}H_f.
$$

Compute an orthonormal left-nullspace basis $U$ such that

$$
U^\mathsf T\tilde H_f=0,\qquad U^\mathsf TU=I.
$$

Then the feature-position error disappears from the first-order residual:

$$
r_o=U^\mathsf T\tilde r,\qquad H_o=U^\mathsf T\tilde H_x,
\qquad r_o\simeq H_o\delta x+n_o,\quad \operatorname{Cov}(n_o)=I.
$$

The retained dimension is $d=2m-\operatorname{rank}(\tilde H_f)$. This profile
proposes accepting only sufficiently conditioned rank-three features, giving
$d=2m-3$; rank-deficient features are rejected rather than silently assigned
rank three. Rank tolerance and conditioning limits remain open. In finite
precision, orthogonality and annihilation require residual bounds rather than
exact coefficient equality. QR can apply orthogonal transforms implicitly;
an explicit dense nullspace matrix is not required.

The proposed joint innovation gate for each feature is

$$
S_o=H_oP^-H_o^\mathsf T+I,\qquad
\gamma=r_o^\mathsf TS_o^{-1}r_o,
\qquad \gamma\le\chi^2_{d,1-\alpha}.
$$

The probability interpretation assumes the linearized Gaussian model. Choose
$\alpha$ and numerical acceptance separately; this is not formal-eskf's
componentwise gate. Use a solve to evaluate the quadratic form.

Stack accepted constraints into $(r,H)$. The default working model treats
distinct accepted measurement noises as independent after whitening; temporal
or cross-feature tracking correlations require a different covariance model.
Optional orthogonal row compression retains the informative rows of $Q^\mathsf TH$
and the matching rows of $Q^\mathsf Tr$, with identity noise covariance. Perform
the feature gate before this compression; discarded state-insensitive residual
components still contain goodness-of-fit information.

## Joint correction, injection and reset

For the accepted, optionally compressed observation with covariance $V$,

$$
S=HP^-H^\mathsf T+V,\qquad KS=P^-H^\mathsf T,\qquad
\widehat{\delta x}=Kr,
$$

$$
P_{\mathrm{corr}}=(I-KH)P^-(I-KH)^\mathsf T+KVK^\mathsf T.
$$

The proposed baseline uses Joseph form and an SPD solve for $S$, without forming
an explicit inverse. In exact arithmetic $P^-\succeq0$, $V\succ0$ imply
$S\succ0$; this does not guarantee success of every floating-point factorization.

Inject additive corrections into current position, velocity, biases and every
clone position. For the current IMU and every cloned attitude $a$, inject

$$
\hat q_a^+=\hat q_a^-\otimes\operatorname{Exp}_q(\widehat{\delta\theta_a}).
$$

Define the right Jacobian, with its continuous limit $J_r(0)=I$, as

$$
J_r(\phi)=I-\frac{1-\cos\theta}{\theta^2}[\phi]_\times
+\frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2,
\qquad \theta=\|\phi\|.
$$

The complete reset is

$$
G=\operatorname{diag}\left(
I_3,I_3,J_r(\widehat{\delta\theta_I}),I_3,I_3,
I_3,J_r(\widehat{\delta\theta_1}),\ldots,
I_3,J_r(\widehat{\delta\theta_N})\right),
$$

$$
P^+=GP_{\mathrm{corr}}G^\mathsf T,\qquad \widehat{\delta x}\leftarrow0.
$$

The closed-form Jacobian is the derivative of the coordinate reset at the
injected mean. Covariance transport remains first-order, not an exact nonlinear
distribution transformation. Floating-point quaternion normalization and its
residual bounds require a numerical contract. A failed correction or injection
must not publish a partially updated joint state/covariance.

## Clone removal and execution order

After processing the relevant constraints, select retained error coordinates
with $E$:

$$
P_{\mathrm{keep}}=EP^+E^\mathsf T.
$$

This is marginalization in covariance form: retain the appropriate principal
submatrix, not a covariance Schur complement. Remove corresponding nominal poses
and linearization caches consistently. Cross-correlations among retained states
remain present.

The proposed image-update sequence is:

1. Validate time/association and propagate the joint state to the image timestamp.
2. Augment its IMU pose clone and append frontend observations to feature tracks.
3. Select eligible tracks, including those affected by impending clone removal.
4. Triangulate, check geometry, build/whiten residuals, eliminate feature error,
   gate and optionally compress accepted constraints.
5. Correct, inject and reset the complete joint estimate.
6. Mark consumed observations, retire obsolete records and remove eligible clones.

Observation reuse and failure/retry behavior need an explicit transaction policy.
The same pixel measurement must not be fused again as independent evidence.
Feature IDs, camera IDs and timestamps must resolve to the intended live clone;
slot reuse alone is not a valid association. Capacity overflow must not silently
overwrite poses or IMU samples still required by pending observations.

## Consistency and linearization

Pure VIO with known gravity has global-translation and global-yaw gauge freedoms
under ordinary nondegenerate motion. Additional degeneracies may occur. A
consistency-aware linearization must preserve these freedoms rather than create
spurious information. For compatible gauge bases $N_k$, the relevant conditions
include

$$
H_kN_k=0,\qquad \bar\Phi_{k+1,k}N_k=N_{k+1},
$$

where $H_k$ is the feature-eliminated visual Jacobian and
$\bar\Phi=\operatorname{diag}(\Phi,I_{6N})$ is the joint transition between state
changes. Augmentation/removal/reset transform the basis as the state coordinates
change. FEJ is the proposed approach [7], but its exact first-estimate retention
and evaluation rules must be derived for this right-local formulation. Freezing
arbitrary Jacobian terms or copying OpenVINS signs does not establish consistency.
GNSS aiding changes observability and requires a separate analysis.

## Implementation and verification boundary

The imported [linalg](../../../include/formal_gvins/linalg/linalg.hpp),
[scalar](../../../include/formal_gvins/scalar/math.hpp) and
[SO(3)](../../../include/formal_gvins/so3/so3.hpp) layers provide numerical
primitives. They do not yet provide active-size joint matrices, QR/nullspace
operations, an MSCKF state or a feature frontend. Eigen and CMSIS-DSP are backend
choices; their external internals are not proved by shared mathematical theorems.

| Boundary | Intended evidence, not completed proof |
|---|---|
| Frames, IMU model and joint propagation | Exact derivation; discretization/noise consistency; covariance symmetry/PSD under stated premises. |
| Augmentation and marginalization | Index/state correspondence, full cross-covariances and preservation of exact PSD. |
| Camera geometry and Jacobians | Derivative proofs, finite-difference cross-checks, depth/domain and calibration-coordinate checks. |
| Whitening, feature elimination and compression | Exact orthogonality/information properties; scoped rank, conditioning and floating-point residual bounds. |
| Correction and all-pose reset | Kalman/Joseph identities, right-local reset derivative, covariance transport and failure atomicity. |
| FEJ and observability | Preservation of the appropriate gauge subspace across propagation, update and state changes. |
| Track/clone lifecycle and resources | Bounded indexing/storage, valid time/identity association, once-only fusion and target timing/workspace measurements. |

Lean can address exact algebra and geometric properties. Scoped C++ verification
can address control flow, indexing and supported floating-point properties.
Backend tests and target experiments address separate numerical/resource claims.
An imported theorem, harness or passing native test is not an automatic proof
of the assembled estimator or of visual-feature correctness.

## Open decisions before implementation

| Decision | What must be fixed |
|---|---|
| Discrete propagation | Integration scheme, IMU interval semantics and matched $\Phi,Q_d$ approximation. |
| Vision profile | Actual camera/distortion model, mono/stereo configuration, synchronization accuracy and track-noise assumptions. |
| Geometry and innovation acceptance | Depth/parallax/conditioning limits, rank tolerance, triangulation iteration cap and gate confidence. |
| Consistency | FEJ variables, cache lifetime and propagation/update/reset linearization rules for these conventions. |
| Capacity and failure | Maximum clones/tracks/observations, workspace ownership, retirement policy and complete failure/retry semantics. |
| Initialization and recovery | Gravity/yaw initialization, bias priors, insufficient-motion handling and loss/reacquisition behavior. |
| GNSS extension | PVT versus raw observations, world/geodetic alignment, antenna lever arm, clocks, timing and correlation handling. |

## References

1. Joan Solà, [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508v1), 2017, sections 5--6; quaternion Exp equations (97)--(101), right Jacobian (183), injection/reset (283)--(294). The profile here fixes gravity and adds pose clones.
2. A. I. Mourikis and S. I. Roumeliotis, [A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation](https://doi.org/10.1109/ROBOT.2007.364024), ICRA 2007. The multi-view constraint construction is adapted to the conventions above.
3. OpenVINS, [JPL quaternion convention](https://docs.openvins.com/classov__type_1_1JPLQuat.html) and [IMU propagation](https://docs.openvins.com/propagation.html).
4. OpenVINS, [StateHelper clone augmentation and marginalization](https://docs.openvins.com/classov__msckf_1_1StateHelper.html).
5. OpenVINS, [camera measurement model](https://docs.openvins.com/update-feat.html) and [feature triangulation](https://docs.openvins.com/update-featinit.html).
6. OpenVINS, [nullspace projection](https://docs.openvins.com/update-null.html) and [MSCKF update](https://docs.openvins.com/classov__msckf_1_1UpdaterMSCKF.html).
7. OpenVINS, [First-Estimate Jacobians](https://docs.openvins.com/fej.html).

OpenVINS documentation was consulted on 2026-11-20. Its URLs are explanatory
references, not a pinned implementation baseline or a claim of behavioral identity.
Any future source import or code-level equivalence claim must identify the exact
upstream revision and recheck the affected conventions and assumptions.
