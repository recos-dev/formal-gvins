# MSCKF Formulation

**Status:** Draft

## Scope and requirements

This is a proposed mathematical baseline for the local MSCKF in formal-gvins,
awaiting owner review. It does not describe an implemented MSCKF or establish
proof coverage. The [requirement index](../README.md) owns the allocated contracts;
remaining equations here are derivations/proposals, not approved requirements.

[SW-MSCKF](../requirements/software/local-vio.md) allocates the proposed local
behavior to eight Draft children. Their Basis links identify the derivation
sections below. Requirements own obligations/acceptance; this document owns the
supporting model and derivation. The selected Draft uses conventional FEJ,
Hamilton right-local Exp injection and an explicitly approximate identity reset.
The [literature/design review](../../analysis/msckf-design-review.md) explains
why accumulated reset-coordinate maps are excluded. This revision still needs
owner review and numerical/statistical acceptance; it is not a proof of
OpenVINS equivalence or of an implemented estimator.

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

### Continuous model

Let $f_m$ and $\omega_m$ be calibrated specific-force and angular-rate
measurements in $B$. Define $a=f_m-\hat b_a$ and
$\omega=\omega_m-\hat b_g$. The nominal model is

$$
\dot{\hat p}_{WB}=\hat v_{WB},\quad
\dot{\hat v}_{WB}=\hat R_{WB}a+g_W,\quad
\dot{\hat q}_{WB}=\tfrac12\hat q_{WB}\otimes[0,\omega],\quad
\dot{\hat b}_a=\dot{\hat b}_g=0.
$$

With gravity fixed, this is Sola's nominal model (237a--e) [1]. Under the stated
error coordinates, his error equations (238a--e), with delta_g removed, give
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

The physical transition and exact continuous-white-noise covariance satisfy

$$
\dot\Phi(t,t_k)=F_I(t)\Phi(t,t_k),\qquad \Phi(t_k,t_k)=I,
$$

$$
Q_{d,\mathrm{ct}}=\int_{t_k}^{t_{k+1}}
\Phi(t_{k+1},\tau)G_I(\tau)Q_cG_I(\tau)^\mathsf T
\Phi(t_{k+1},\tau)^\mathsf T\,d\tau.
$$

This integral is a continuous-model reference [9], not the selected discrete Qd.
The filter uses the complete FEJ transition Psi and the interval-equivalent
noise construction below. Historical nominal poses remain unchanged during
propagation; their correlation with the current IMU state changes.

### Discrete nominal model

The baseline uses midpoint input approximation: average the calibrated body-frame
measurements at both endpoints of each propagation segment, subtract the current
bias means, then hold the result constant for analytical integration [10]. For
$h=t_r-t_l>0$, the effective inputs used throughout the following discrete formulas
are

$$
a=\tfrac12\bigl(f_m(t_l)+f_m(t_r)\bigr)-\hat b_a,\qquad
\omega=\tfrac12\bigl(\omega_m(t_l)+\omega_m(t_r)\bigr)-\hat b_g.
$$

Bias means remain constant within that segment. This is a zero-order hold (ZOH)
of the endpoint average, not a midpoint Runge-Kutta solver. It does not assume
constant world-frame acceleration: the body rotates during the segment.

The acquisition adapter divides propagation at every real IMU sample and image
boundary. For a boundary $t_i<t<t_{i+1}$, linearly interpolate either calibrated
measurement $y_m\in\{f_m,\omega_m\}$:

$$
\lambda=\frac{t-t_i}{t_{i+1}-t_i},\qquad
y_m(t)=(1-\lambda)y_m(t_i)+\lambda y_m(t_{i+1}).
$$

An exact sample timestamp reuses that sample. Each resulting segment uses its own
endpoint average; the full interval's average is not reused for both parts.
After a correction at a boundary, recompute the following segment's effective
inputs with the updated biases, retaining bias-independent measured endpoints.
Missing bracketing samples mean not-ready, without extrapolation. Subdivision
does not make an excessive original sample gap admissible. Interpolated endpoints
are input approximations, not independent sensor observations.

Write $R=R_{WB}(t_k)$, $\phi=\omega h$, $K=[\phi]_\times$,
$\theta=\|\phi\|$, and $E(s)=\operatorname{Exp}([\omega]_\times s)$.
Define

$$
A_1(h)=\int_0^h E(s)\,ds=h\Gamma_1(\phi),\qquad
A_2(h)=\int_0^h(h-s)E(s)\,ds=h^2\Gamma_2(\phi),
$$

$$
\Gamma_1(\phi)=I+\frac{1-\cos\theta}{\theta^2}K
+\frac{\theta-\sin\theta}{\theta^3}K^2,
$$

$$
\Gamma_2(\phi)=\frac12I+\frac{\theta-\sin\theta}{\theta^3}K
+\frac{\theta^2/2+\cos\theta-1}{\theta^4}K^2.
$$

The continuous extensions are
$\Gamma_\ell(\phi)=\sum_{i=0}^{\infty}K^i/(i+\ell)!$ for $\ell=1,2$.
Use these limits near zero, rather than evaluating cancelling quotients.
The exact solution of the held-input nominal ODE is

$$
\begin{aligned}
\hat p_{k+1}^-&=\hat p_k^++\hat v_k^+h+R A_2(h)a+\tfrac12g_Wh^2,\\
\hat v_{k+1}^-&=\hat v_k^++R A_1(h)a+g_Wh,\\
\hat q_{k+1}^-&=\hat q_k^+\otimes\operatorname{Exp}_q(\omega h),\\
\hat b_{a,k+1}^-&=\hat b_{a,k}^+,\qquad
\hat b_{g,k+1}^-=\hat b_{g,k}^+ .
\end{aligned}
$$

Position uses the interval's initial velocity. At $\omega=0$,
$A_1=hI$ and $A_2=h^2I/2$, recovering constant-acceleration integration.
"Exact" here is conditional on the held endpoint average, not a solution for
arbitrary time-varying input or a claim of error-free floating-point evaluation.
This is a project derivation under the declared conventions; analytical IMU
integration is also discussed in [8].

### Transition of the held-input model

The angular-rate bias affects translation during rotation. The SO(3) right
Jacobian [1], with its continuous limit $J_r(0)=I$, is

$$
J_r(\phi)=I-\frac{1-\cos\theta}{\theta^2}[\phi]_\times
+\frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2,
\qquad \theta=\|\phi\|.
$$

Define the translation sensitivities:

$$
\begin{aligned}
B_1(h)&=\int_0^h E(s)[a]_\times J_r(\omega s)s\,ds,\\
B_2(h)&=\int_0^h(h-s)E(s)[a]_\times J_r(\omega s)s\,ds.
\end{aligned}
$$

Indeed,
$D_{b_g}(E(s)a)=E(s)[a]_\times J_r(\omega s)s$ and
$D_{b_a}(E(s)a)=-E(s)$. The ordinary transition, before the FEJ modification
defined later, is

$$
\Phi^{\mathrm{ZOH}}=
\begin{bmatrix}
I&hI&-R[A_2a]_\times&-RA_2&RB_2\\
0&I&-R[A_1a]_\times&-RA_1&RB_1\\
0&0&E(h)^\mathsf T&0&-hJ_r(\phi)\\
0&0&0&I&0\\
0&0&0&0&I
\end{bmatrix}.
$$

This is both the derivative of the discrete nominal map and the transition of
its continuous first-order error ODE. It is not $I+F_I(t_k)h$.
At zero angular rate,
$B_1=h^2[a]_\times/2$ and $B_2=h^3[a]_\times/6$; the gyro-bias translation
blocks generally remain nonzero. The development profile specifies bounded
evaluation of these coefficients and independent numerical checks.

### Interval-equivalent process noise

The selected model follows the analytical OpenVINS construction [10], translated
to our error order $(\delta p,\delta v,\delta\theta,\delta b_a,\delta b_g)$.
It uses one interval-equivalent noise vector
$\epsilon=(\epsilon_a,\epsilon_g,\epsilon_{ba},\epsilon_{bg})$, whose components
are noise rates, including the bias drivers. For $h>0$ and constant continuous
density $Q_c\succeq0$ in that same order,

$$
Q_\epsilon=\operatorname{Cov}(\epsilon)=\frac{Q_c}{h},\qquad
Q_c=\operatorname{diag}(S_a,S_g,S_{ba},S_{bg})
\quad\text{for independent drivers}.
$$

The first profile uses $S_i=\sigma_i^2I$. A supplied anisotropic/correlated PSD
$Q_c$ uses the same congruence without discarding its blocks; this is a local
generalization beyond the reference's scalar density settings. Density coordinates
must match the calibrated body measurements and bias drivers. The upstream
variable named `Qc` in `predict_and_compute` already contains the division by h;
it corresponds to our $Q_\epsilon$, not our continuous $Q_c$.

Let $R_k^0$ be the cached start attitude and
$D_k=(R_{k+1}^0)^\mathsf T R_k^0$, where the new first estimate equals the
propagated nominal endpoint. These are the same FEJ references used by Psi below.
With $A_1,A_2,B_1,B_2$ computed from current bias-corrected inputs,

$$
G_d=\begin{bmatrix}
-R_k^0 A_2&R_k^0 B_2&0&0\\
-R_k^0 A_1&R_k^0 B_1&0&0\\
0&-hD_kJ_r(-\phi)&0&0\\
0&0&hI&0\\
0&0&0&hI
\end{bmatrix},\qquad
Q_d=G_dQ_\epsilon G_d^\mathsf T.
$$

$G_d$ is $15\times12$ and differs from the instantaneous $G_I$. The first two
column groups are the held-input sensor-noise sensitivities. The final groups
inject bias increments of covariance $S_{ba}h$ and $S_{bg}h$ at the endpoint.
For independent drivers this adds no within-segment bias-noise coupling into
pose/velocity. Existing bias uncertainty still propagates through Psi's bias
columns. This is an intentional discrete approximation, not exact integration
of continuous bias random walks. Exact symmetry/PSD follows from the congruence;
floating-point evaluation and model fidelity need separate qualification.

The reference correspondence assumes fixed ideal IMU calibration (calibrated
inputs already in B, with no residual gyroscope acceleration-sensitivity term):

| Quantity | OpenVINS analytical path | This formulation |
|---|---|---|
| Error order | `(theta,p,v,bg,ba)` | `(p,v,theta,ba,bg)` |
| Noise order | `(n_g,n_a,n_bg,n_ba)` | `(n_a,n_g,n_ba,n_bg)` |
| Stored rotation | World to IMU | IMU to world; transpose the reference rotation |
| Analytical coefficients | Xi1, Xi2, Xi3, Xi4 | A1, A2, B1, B2 |

After the convention conversion, state/noise permutations give the same Gd and
Qd when coefficients and FEJ references agree. The pinned source has approximate
small-rate coefficient branches; the profile retains its own bounded evaluation
of the ideal integrals. Matching the model does not imply bitwise equality or
identical source execution. Calibration states and their extra columns are not
part of the first profile.

At zero angular rate, with current and first-estimate attitudes equal and only
accelerometer density $S_a$, put $S_W=RS_aR^\mathsf T$. The selected model gives

$$
Q_{pp}=\tfrac14S_Wh^3,\qquad
Q_{pv}=Q_{vp}^\mathsf T=\tfrac12S_Wh^2,\qquad Q_{vv}=S_Wh.
$$

The continuous-white-noise integral instead gives $Q_{pp}=S_Wh^3/3$.
Consequently that integral, including its Van Loan evaluation [9], cannot serve
as an equality oracle for the selected Qd.

For each actual positive segment duration, recompute Gd and $Q_\epsilon=Q_c/h$.
With segment disturbances modeled as independent, accumulate in time order:

$$
Q_{20}=\Psi_{21}Q_{10}\Psi_{21}^\mathsf T+Q_{21}.
$$

Do not require equality with a single unsplit Qd. Even for identical held inputs,
the zero-rate accelerometer-only example gives $5S_Wh^3/16$ in Qpp after two
equal half-steps, instead of $S_Wh^3/4$ for one step. Mean and ordinary-transition
composition still agree under identical inputs without correction; re-averaging
varying readings changes even that input approximation. FEJ composition has the
scope stated below.

Averaging/interpolating endpoints does not halve Qc or create a separate endpoint
disturbance. Do not also inject the original full-interval Qd. Independent segment
noise is a modeling approximation; it does not establish the exact covariance
of shared, interpolated finite-sample errors. Correlated sample errors or an IMU
providing integrated increments require their own acquisition/noise model.

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

The observation is the feature's position in the original distorted image.
ORB detection/description operates on that image without geometric remapping.
Pointwise undistortion supplies normalized bearings for geometric operations;
it does not replace the stored raw measurement. With fixed calibration, define

$$
z_{fcj}=\pi_c(y_{fcj})+n_{fcj},\qquad
\pi_c(y)=D_c(\pi_n(y)),\qquad
\pi_n([X,Y,Z]^\mathsf T)=\begin{bmatrix}X/Z\\Y/Z\end{bmatrix}.
$$

$D_c$ maps normalized coordinates to raw pixels, including lens distortion and
intrinsics. Residual $z_{fcj}-\pi_c(y_{fcj})$ and noise covariance $V_{fcj}$ are
both in raw pixel coordinates. This follows the OpenVINS measurement domain
[5,12], retaining our Hamilton/right-local pose convention and fixed calibration.

The first EuRoC profile uses zero-skew radial-tangential calibration
$(f_x,f_y,c_x,c_y,k_1,k_2,p_1,p_2)$. For normalized coordinates $(x,y)$, let
$\rho=x^2+y^2$ and $\ell=1+k_1\rho+k_2\rho^2$. Then

$$
x_d=x\ell+2p_1xy+p_2(\rho+2x^2),\qquad
y_d=y\ell+p_1(\rho+2y^2)+2p_2xy,
$$

$$
D_c([x,y]^\mathsf T)=\begin{bmatrix}f_xx_d+c_x\\f_yy_d+c_y\end{bmatrix}.
$$

Other lens models require a matched forward map, inverse-domain policy and
derivatives; they are not silently approximated by this model. Zero distortion
recovers ordinary pinhole pixels. Raw and normalized coordinates of a feature
are representations of the same observation, not two independent measurements.

At a valid linearization point with positive depth,

$$
J_\pi=J_D J_n,\qquad
J_n=\begin{bmatrix}1/Z&0&-X/Z^2\\0&1/Z&-Y/Z^2\end{bmatrix},\qquad
J_D=\left.\frac{\partial D_c}{\partial(x,y)}\right|_{(x,y)=\pi_n(y_{fcj})}.
$$

For the radial-tangential model, put $\eta=k_1+2k_2\rho$. Its derivative is

$$
J_D=\begin{bmatrix}f_x&0\\0&f_y\end{bmatrix}
\begin{bmatrix}
\ell+2x^2\eta+2p_1y+6p_2x&2xy\eta+2p_1x+2p_2y\\
2xy\eta+2p_1x+2p_2y&\ell+2y^2\eta+6p_1y+2p_2x
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

To initialize the temporary feature, invert $D_c$ pointwise on the qualified
camera domain: $\xi_{cj}=D_c^{-1}(z_{cj})=[x_n,y_n]^\mathsf T$. This is a
locally valid inverse, not a claim of global invertibility for arbitrary lens
coefficients. An iterative inverse needs a bounded iteration policy and a
forward-reprojection residual check; failure rejects that geometric input.
Retain the raw z and its calibration identity even if the derived bearing is
cached. Write the bearing and world-frame camera center as

$$
d_{cj}=\begin{bmatrix}x_n\\y_n\\1\end{bmatrix},
\qquad c_{cj}=\hat p_{WB_j}+\hat R_{WB_j}p_{BC_c},
\qquad R_{C_cW_j}=R_{BC_c}^\mathsf T\hat R_{WB_j}^\mathsf T.
$$

Collinearity gives

$$
A_{cj}p_f^W=b_{cj},\qquad
A_{cj}=[d_{cj}]_\times R_{C_cW_j},\qquad b_{cj}=A_{cj}c_{cj}.
$$

Each three-row block has only two independent rows. Stack views and solve the
overdetermined system using rank-revealing QR or SVD. This algebraic solution is
an initializer, not a pixel-noise maximum-likelihood estimate; pure rotation
or insufficient baseline can leave depth unobservable.

Optionally refine the point with a bounded number of iterations on

$$
\hat p_f^W\approx\arg\min_p
\sum_{c,j}\|z_{fcj}-\pi_c(y_{fcj}(p))\|_{V_{fcj}^{-1}}^2.
$$

This equation defines an objective, not guaranteed convergence or a globally
optimal solution. Positive depth, adequate parallax and conditioning require
explicit acceptance bounds. For fixed poses, a Gauss--Newton step solves
$\min_{\Delta p}\|L^{-1}(r_f-H_f\Delta p)\|^2$ and proposes
$p_f\leftarrow p_f+\Delta p$, with $V_f=LL^\mathsf T$.
Here $H_f$ is the current geometric derivative, not the later FEJ factor.
Invalid steps or a failed convergence/geometry criterion do not produce a usable
feature. Damping, step acceptance and iteration limits belong to the numerical
profile. Refinement uses the complete raw-pixel projection and its derivative;
the normalized bearings are only the geometric initializer. At chosen
nominal/feature linearization points,
stacking the feature's $m$ two-dimensional observations gives

$$
r_f=z_f-h_f(\hat x,\hat p_f)
\simeq H_x\delta x+H_f\delta p_f+n_f,
\quad H_x\in\mathbb R^{2m\times D},\quad H_f\in\mathbb R^{2m\times3}.
$$

These are ordinary differential Jacobians at a common linearization point.
The FEJ section below specifies the frozen points and linearization policy used
instead in the proposed consistency-aware update. Residuals still use current
estimates; the resulting FEJ model is an intentional linearization approximation.

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

For full-column-rank $\tilde H_f$, this elimination has a precise least-squares
meaning. For any fixed $\delta x$, set $b=\tilde r-\tilde H_x\delta x$. Then

$$
\min_{\delta p_f}\|b-\tilde H_f\delta p_f\|^2
=b^\mathsf T\Pi b=\|U^\mathsf Tb\|^2,\qquad
\Pi=I-\tilde H_f(\tilde H_f^\mathsf T\tilde H_f)^{-1}\tilde H_f^\mathsf T
=UU^\mathsf T.
$$

The inverse expresses the identity; a production implementation uses orthogonal
factorization. This removes a nuisance parameter with no independent landmark
prior. Do not add the triangulated point as another independent measurement:
it was estimated from these same image observations.

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

More explicitly, if an orthogonal $Q=[Q_1,Q_2]$ gives $Q_2^\mathsf TH=0$, use
$H_c=Q_1^\mathsf TH$ and $r_c=Q_1^\mathsf Tr$. Then
$H_c^\mathsf TH_c=H^\mathsf TH$ and
$H_c^\mathsf Tr_c=H^\mathsf Tr$. Compression preserves the state-dependent
linear Gaussian likelihood. Dropping merely small rows is an additional
approximation requiring a rank/error criterion, not this exact identity.

## Joint correction, injection and reset

For the accepted, optionally compressed observation with covariance $V$,

$$
S=HP^-H^\mathsf T+V,\qquad KS=P^-H^\mathsf T,\qquad
\widehat{\delta x}=Kr,
$$

$$
P_c=(I-KH)P^-(I-KH)^\mathsf T+KVK^\mathsf T.
$$

Use the FEJ measurement Jacobian defined below, Joseph form and an SPD solve
without an explicit inverse. Exact $P^-\succeq0$, $V\succ0$ imply $S\succ0$;
finite-precision factorization still needs its own acceptance.

### Hamilton injection and reference translation

Inject additive corrections into current position, velocity, biases and every
clone position. For every current/cloned attitude $a$, with correction $d_a$,

$$
\hat q_a^+=\hat q_a^-\otimes\operatorname{Exp}_q(d_a).
$$

The selected baseline keeps the project's Exp retraction. First estimates stay
unchanged during correction. All updated poses and the complete covariance form
one transaction; do not update only the current IMU or diagonal blocks.

For the pinned OpenVINS reference [10,11], identify global G with W and IMU I
with B, after expressing both states in the same physical axes. The reference
uses `-gravity_ref` in its mean; identify `-gravity_ref=g_W` in that common world
basis (rotate all world quantities when comparing different up/down frames):

| Quantity | OpenVINS | This formulation |
|---|---|---|
| Stored quaternion coefficients | JPL $(q_v,q_s)$ | Hamilton $(q_s,q_v)$, up to common quaternion sign |
| Rotation action | $C(q):W\to B$ | $R(q):B\to W=C(q)^\mathsf T$ |
| Local attitude error | Left JPL increment | Right Hamilton increment; same infinitesimal $d$ |
| IMU error order | $(\theta,p,v,b_g,b_a)$ | $(p,v,\theta,b_a,b_g)$ |
| Clone error order | $(\theta,p)$ | $(p,\theta)$ |

This translation follows $C(q)=R(q)^\mathsf T$ and reversal of product order
between these JPL/Hamilton representations. It is not a rule to conjugate every
quaternion called JPL without checking its rotation action. Pose/Jacobian
comparisons must also permute both covariance axes and the matching columns.

OpenVINS injects a normalized small-angle increment; in our notation it is

$$
c(d)=\frac{[1,d/2]}{\sqrt{1+\|d\|^2/4}}
=\operatorname{Exp}_q\!\left(\frac{2\arctan(\theta/2)}{\theta}d\right),
\qquad \theta=\|d\|,
$$

with the zero limit understood. We instead use $\operatorname{Exp}_q(d)$.
Their first derivatives at zero agree. In the admitted small-angle domain,
the finite rotation difference satisfies

$$
\theta-2\arctan(\theta/2)=\int_0^\theta\frac{t^2}{4+t^2}\,dt
\le\theta^3/12.
$$

This is an explicit adaptation, not finite-update equivalence.
No small-angle replacement of Exp is selected for IMU propagation either.

### Conventional FEJ identity reset

After injection, reset the estimated error mean to zero and use

$$
G_{\mathrm{FEJ}}=I_D,\qquad P^+=P_c,\qquad \widehat{\delta x}\leftarrow0.
$$

This is the **identity-reset approximation**, following the absence of explicit
reset transport in the inspected OpenVINS update [11]. Sola also discusses the
identity approximation after (288) [1]; his nontrivial reset remains distinct.
P is the FEJ filter's
first-order covariance approximation in the declared local block layout. It is
not asserted to be the exact covariance of the nonlinear physical error after
finite injection. No $T_a$, inverse-map solves, $H^0T^{-1}$, or conjugated
transition are part of this baseline. Keep all cached first estimates fixed at
the update time; the FEJ gauge basis therefore stays fixed through correction.

For comparison, the physical right-log error defined by V-FRAMES obeys

$$
\delta\theta^+
=\operatorname{Log}\!\left(
\operatorname{Exp}(-[d]_\times)\operatorname{Exp}([\delta\theta]_\times)\right),
\qquad
\left.\frac{\partial\delta\theta^+}{\partial\delta\theta}
\right|_{\delta\theta=d}=J_r(d).
$$

Its derivative is generally not identity. The baseline neglects its
$-\tfrac12[d]_\times+O(\|d\|^2)$ departure from identity; it does not prove
that term zero. Inserting a Jr reset only into P while keeping the selected
frozen H/Psi unchanged would define a different filter and invalidate the
composition argument below.

A useful one-step comparison bound follows from
$J_r(d)=\int_0^1\operatorname{Exp}(-s[d]_\times)\,ds$:
$\|J_r(d)-I\|_2\le\|d\|/2$.
For the block reset $G_{\mathrm{phys}}$ containing all $J_r(d_a)$, let
$\epsilon=\max_a\|d_a\|/2$. Then

$$
\|G_{\mathrm{phys}}P_cG_{\mathrm{phys}}^\mathsf T-P_c\|_2
\le(2\epsilon+\epsilon^2)\|P_c\|_2.
$$

This compares two local covariance formulas at the same P_c; it does not bound
trajectory error, accumulated FEJ error or the exact nonlinear distribution.
A profile must bound all attitude corrections and qualify the approximation
empirically. Failed solves, excessive corrections, non-finite candidates or
failed quaternion normalization preserve the entire pre-correction transaction.

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

1. Validate time/association and propagate the joint state to the image timestamp,
   applying the FEJ transition and establishing the new IMU first estimate.
2. Augment its IMU pose clone and matching first estimate; append frontend
   observations to feature tracks.
3. Select eligible tracks, including those affected by impending clone removal.
4. Triangulate, check geometry, build/whiten residuals, eliminate feature error,
   gate and optionally compress accepted constraints.
5. Correct/inject the complete joint estimate and apply identity reset, retaining
   first estimates.
6. Mark consumed observations, retire obsolete records and remove eligible clones.

Following [V-TRACK](../requirements/vision/feature-tracks.md), commit the corrected
joint estimate, covariance, frozen first estimates and observation-consumption
records together. A numerical failure leaves that correction transaction's
inputs unchanged; earlier successful propagation is a separate operation.
An explicitly rejected/retired observation is not a successfully fused one.
If no track is accepted, skip correction/reset and apply only the declared
retirement/removal policy.

The same pixel must not be fused again as independent evidence, even if its
track or descriptor is retained for a separate map. Feature, camera and image
identities must resolve to the intended live clone; slot reuse alone is not an
association. Capacity overflow must not silently overwrite poses or IMU samples
still required by pending observations. Retry, overflow and clone-selection
policies remain runtime decisions.

## Consistency and linearization

### Four gauge directions

Let $u=g_W/\|g_W\|$. A common translation $t$ and rotation
$S=\operatorname{Exp}(\psi[u]_\times)$ act on the trajectory and features as
$p'=Sp+t$, $v'=Sv$, $R'=SR$, $p_f'=Sp_f+t$. Gravity and body-frame biases
are unchanged. These transformations leave the IMU/visual model unchanged.
Thus pure VIO has three global-translation and one global-yaw gauge directions.
Degenerate motion can introduce additional unobservable directions.

$$
N_I(x)=
\begin{bmatrix}
I&[u]_\times p\\
0&[u]_\times v\\
0&R^\mathsf Tu\\
0&0\\
0&0
\end{bmatrix},\qquad
N_{c_j}(x)=
\begin{bmatrix}I&[u]_\times p_j\\0&R_j^\mathsf Tu\end{bmatrix},\qquad
N_f(p_f)=\begin{bmatrix}I&[u]_\times p_f\end{bmatrix}.
$$

Stack $N_I,N_{c_1},\ldots,N_{c_N}$ vertically to obtain $N\in\mathbb R^{D\times4}$.
At a single common nominal point the ordinary derivatives satisfy

$$
H_xN+H_fN_f=0,\qquad
\Phi^{\mathrm{ZOH}}N_I(x_k)=N_I(x_{k+1}^-).
$$

The first identity follows already before projection:
$-R_j^\mathsf T[u]_\times p_j+
[s_j]_\times R_j^\mathsf Tu+
R_j^\mathsf T[u]_\times p_f=0$.
Premultiplying by the same $J_DJ_n$ in both geometric factors preserves this
cancellation. Whitening and eliminating the feature therefore give $H_oN=0$.
These single-step identities alone do not establish multi-update consistency:
replacing old linearization points after every correction changes the basis
used by successive constraints [7].

### First estimates and their lifetime

Use a superscript $0$ for first-estimate values, distinct from the corrected
nominal estimate denoted by a hat. The proposed FEJ policy is:

| Quantity | First estimate and lifetime | Use |
|---|---|---|
| Current IMU $p^0,v^0,R^0$ | Initialize from the starting nominal state; set to its newly propagated nominal value at each new time, before correction there; hold through corrections at that same time. | Previous endpoint of the next FEJ transition. |
| Clone $p_j^0,R_j^0$ | Copy the current IMU's first estimate when cloned; keep until clone removal. | All visual Jacobians involving that clone. |
| Temporary feature $p_f^0$ | Use one triangulated estimate when assembling its candidate constraint; hold for every row of that assembly. | All factors of $H_x^0,H_f^0$ for that constraint. |
| Bias means and held IMU inputs | Use current corrected bias means and the declared interval input. | Nominal propagation and A/B integrals; both Psi and Gd use the specified frozen-attitude factors. |
| Calibration and gravity | Fixed profile constants. | Nominal model, residuals and Jacobians. |

Compute $r_f=z_f-h_f(\hat x,\hat p_f)$ at the current nominal poses and temporary
feature estimate. Compute every geometric factor of $H_x^0,H_f^0$, including
$J_\pi=J_DJ_n$, at $(p_j^0,R_j^0,p_f^0)$; mixing a current feature with different
per-row frozen features breaks the cancellation. Both current and frozen
projections need valid depth/domain. A rebuilt candidate rebuilds all its rows
at a common feature point; it cannot mix old factors with a new point.
These temporary features are not persistent EKF landmarks.

This fully frozen distortion derivative is a local choice. In the pinned
`UpdaterHelper::get_feature_jacobian_full` [12], `uv_norm` is computed at the
current nominal point and is not recomputed in the FEJ branch: that path uses
$J_D(\hat\xi)J_n(y^0)$, whereas we use $J_D(\xi^0)J_n(y^0)$, with
$\hat\xi=\pi_n(\hat y)$ and $\xi^0=\pi_n(y^0)$. Our choice keeps the complete
visual Jacobian at one declared frozen point. Either common left factor preserves
the geometric gauge cancellation in Hx/Hf. Their numerical Jacobians and
corrections can differ when current/frozen points differ and distortion is
nonzero; neither gauge cancellation nor this comparison proves equal accuracy.
The zero-distortion case has constant JD and removes this particular difference.

FEJ deliberately uses a Jacobian different from the derivative at the current
residual evaluation point. The equalities below concern that chosen linear
system; they do not remove its nonlinear linearization error.

### FEJ transition

The nominal endpoint is propagated from the **current corrected** state using
the declared ZOH mean. Set its first estimate to that propagated endpoint:
$(R_{k+1}^0,p_{k+1}^0,v_{k+1}^0)=(\hat R_{k+1}^-,\hat p_{k+1}^-,\hat v_{k+1}^-)$.
Retain the previous endpoint's cached $(R_k^0,p_k^0,v_k^0)$ even if corrections
have changed its nominal state. Set

$$
D_k=(R_{k+1}^0)^\mathsf T R_k^0,\qquad
\Delta v^0=v_{k+1}^0-v_k^0-g_Wh,\qquad
\Delta p^0=p_{k+1}^0-p_k^0-v_k^0h-\tfrac12g_Wh^2.
$$

With $a,\omega$ evaluated using current corrected biases, $\phi=\omega h$,
and the previously defined $A_1,A_2,B_1,B_2$, the entire selected IMU transition is

$$
\Psi=\begin{bmatrix}
I&hI&-[\Delta p^0]_\times R_k^0&-R_k^0 A_2&R_k^0 B_2\\
0&I&-[\Delta v^0]_\times R_k^0&-R_k^0 A_1&R_k^0 B_1\\
0&0&D_k&0&-hD_kJ_r(-\phi)\\
0&0&0&I&0\\
0&0&0&0&I
\end{bmatrix}.
$$

This specifies **all** blocks, including frozen-attitude bias sensitivities;
it is not a three-block patch of the current physical transition. It translates
the ideal analytic FEJ structure of the pinned OpenVINS path [10] with fixed
ideal IMU calibration: its $\Xi_1,\Xi_2,\Xi_3,\Xi_4$ correspond to
$A_1,A_2,B_1,B_2$. Its gyro-bias block uses $D_kJ_r(-\phi)$, not simply
$J_r(\phi)$ after an intervening correction. Our integral definitions and
profile evaluation own small-rate accuracy; source branch approximations are
not silently adopted.

When the previous first estimate equals the corrected state, $D_k=E(h)^\mathsf T$
and $E(h)^\mathsf TJ_r(-\phi)=J_r(\phi)$; every block reduces to
$\Phi^{\mathrm{ZOH}}$. Otherwise Psi is a FEJ linearization approximation,
not the derivative of the current nominal map.

For the joint state use $\bar\Psi=\operatorname{diag}(\Psi,I_{6N})$:

$$
P_{II}^-=\Psi P_{II}^+\Psi^\mathsf T+Q_d,\qquad
P_{IC}^-=\Psi P_{IC}^+,\qquad P_{CC}^-=P_{CC}^+.
$$

Qd uses the interval-equivalent construction above, including the same frozen
start attitude and endpoint rotation Dk as Psi. Using current attitude factors
only for Gd would change the selected reference binding. This FEJ/discrete-noise
pair does not claim to exactly integrate the original continuous stochastic ODE.
No extra coordinate map or reference-noise covariance is maintained.

### Composition and scope of the consistency claim

Use $N^0=N(x^0)$, assembled from the cached IMU/clone first estimates, and
$N_f^0=N_f(p_f^0)$ for the common temporary feature point. Set
$H_x=H_x^0$ and $H_f=H_f^0$ directly; no coordinate transport follows correction.
The selected linear model satisfies

$$
H_x^0N^0+H_f^0N_f^0=0,\qquad H_oN^0=0,\qquad
\bar\Psi N_k^0=N_{k+1}^0,\qquad (I-KH_o)N^0=N^0.
$$

For example, the velocity yaw column is
$[u]_\times v_k^0-[\Delta v^0]_\times u=[u]_\times v_{k+1}^0$;
the position column also includes $h[u]_\times v_k^0$. The attitude column
uses $D_k(R_k^0)^\mathsf Tu=(R_{k+1}^0)^\mathsf Tu$.
Bias gauge rows are zero: the gauge identity alone does not validate bias blocks.
Their acceptance also requires comparison with the specified full transition.

Whitening, feature elimination and compression preserve the nullspace identity.
With $A_c=[I;J_c]$, cloning maps it to $A_cN^0$ and selection maps it to $EN^0$.
Correction changes nominal means but not cached first estimates; its selected
reset is identity, so the frozen gauge is unchanged. New IMU first estimates
are established only by successful propagation; clone first estimates remain
fixed until removal. This argument concerns the selected FEJ linear model,
not an exact physical-gauge transformation under finite injection.

These identities prevent direct visual information along that model's four
gauge directions. They do not imply four zero eigenvalues of P, constant yaw
variance, accuracy, or statistical consistency under arbitrary initialization
and nonlinear errors. No complete semigroup claim across arbitrary corrections
follows for every bias/noise block from a gauge identity. Physical interval
composition without correction remains a separate propagation obligation.
GNSS, absolute heading and map feedback change the information/gauge model and
require separate derivations.

## Implementation and verification boundary

The imported [linalg](../../../include/formal_gvins/linalg/linalg.hpp),
[scalar](../../../include/formal_gvins/scalar/math.hpp) and
[SO(3)](../../../include/formal_gvins/so3/so3.hpp) layers provide numerical
primitives. The [linear algebra inventory](../../analysis/msckf-linear-algebra.md)
tracks bounded operations and their acceptance boundary; the
[frontend guide](../../guides/frontend.md) describes the separate image pipeline.
These components do not implement the joint MSCKF transaction derived here.
Eigen and CMSIS-DSP are backend choices; their external internals are not proved
by shared mathematical theorems.

| Boundary | Intended evidence, not completed proof |
|---|---|
| Frames, IMU model and joint propagation | Endpoint averaging/interpolation; held-input nominal/transition derivative; Gd, density/time scaling and source-order translation; segment noise recurrence; identical-input mean/ordinary-transition composition; exact covariance symmetry/PSD and separate numerical qualification. |
| Augmentation and marginalization | Index/state correspondence, full cross-covariances and preservation of exact PSD. |
| Camera geometry and Jacobians | Distortion/projection derivative proofs, zero-distortion limit, inverse-domain/round-trip checks, current/frozen finite-difference cross-checks and calibration-coordinate binding. |
| Whitening, feature elimination and compression | Exact orthogonality/information properties; scoped rank, conditioning and floating-point residual bounds. |
| Correction and all-pose reset | Joseph PSD, Hamilton Exp injection, explicit identity-reset behavior and failure atomicity; bound its difference from the physical reset separately. |
| FEJ and observability | Full Psi block binding, frozen visual-factor cancellation, and gauge composition through identity reset, propagation, augmentation and removal. |
| Track/clone lifecycle and resources | Bounded indexing/storage, valid time/identity association, once-only fusion and target timing/workspace measurements. |

Lean can address exact algebra and geometric properties. Scoped C++ verification
can address control flow, indexing and supported floating-point properties.
Backend tests and target experiments address separate numerical/resource claims.
An imported theorem, harness or passing native test is not an automatic proof
of the assembled estimator or of visual-feature correctness.

## Open decisions before implementation

The [local VINS development profile](../profiles/local-vins.md) proposes the
first mono/CPU bindings below. It remains Draft: owner review and the specified
implementation/trajectory evidence are still required. It does not select GNSS
or deployment settings.

| Decision | What must be fixed |
|---|---|
| Propagation implementation | Implement endpoint averaging/interpolation and analytical Gd Q_epsilon Gd^T; qualify coefficient evaluation, small-rate differences, error limits, noise discretization and timing/workspace budgets. |
| Vision profile | Implement the selected raw-image, mono/radial-tangential binding; qualify inverse-camera domain/iterations/residual limits, frontend geometric-threshold units, synchronization accuracy and raw-pixel noise. Other cameras/configurations require their own profiles. |
| Geometry and innovation acceptance | Depth/parallax/conditioning limits, rank tolerance, triangulation iteration cap and gate confidence. |
| Consistency acceptance | Review the complete conventional-FEJ/Exp/identity-reset binding; qualify correction limits and statistical consistency separately from its linear-model gauge identities. |
| Capacity and failure | Maximum clones/tracks/observations, workspace ownership, retirement policy and complete failure/retry semantics. |
| Initialization and recovery | Gravity/yaw initialization, bias priors, insufficient-motion handling and loss/reacquisition behavior. |
| GNSS extension | PVT versus raw observations, world/geodetic alignment, antenna lever arm, clocks, timing and correlation handling. |

## References

1. Joan Solà, [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508v1), 2017, sections 5--6; quaternion Exp (97)--(101), right Jacobian (183), continuous nominal/error dynamics (237)--(238), injection/reset (283)--(294). The profile fixes gravity and adds pose clones; it adopts OpenVINS's analytical discretization rather than Sola's simplified (260)--(271).
2. A. I. Mourikis and S. I. Roumeliotis, [A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation](https://doi.org/10.1109/ROBOT.2007.364024), ICRA 2007. The multi-view constraint construction is adapted to the conventions above.
3. OpenVINS, [JPL quaternion convention](https://docs.openvins.com/classov__type_1_1JPLQuat.html) and [IMU propagation](https://docs.openvins.com/propagation.html).
4. OpenVINS, [StateHelper clone augmentation and marginalization](https://docs.openvins.com/classov__msckf_1_1StateHelper.html).
5. OpenVINS, [camera measurement model](https://docs.openvins.com/update-feat.html) and [feature triangulation](https://docs.openvins.com/update-featinit.html). The raw-pixel distortion/projection chain is retained; pose derivatives follow our Hamilton/right-local convention with fixed calibration.
6. OpenVINS, [nullspace projection](https://docs.openvins.com/update-null.html) and [MSCKF update](https://docs.openvins.com/classov__msckf_1_1UpdaterMSCKF.html).
7. OpenVINS, [First-Estimate Jacobians](https://docs.openvins.com/fej.html).
8. OpenVINS, [analytical propagation](https://docs.openvins.com/propagation_analytical.html), especially mean integration, error-state evolution and the FEJ endpoint discussion. The adaptation above translates state order, rotation convention and gravity sign; reference [10] fixes the source-level noise construction.
9. Charles F. Van Loan, [Computing Integrals Involving the Matrix Exponential](https://doi.org/10.1109/TAC.1978.1101743), IEEE Transactions on Automatic Control, 23(3):395--404, 1978. Background for exact continuous-white-noise integration, not the selected interval-equivalent Qd.

10. OpenVINS revision `69488123ed9362dd44b6f28e7f4680abbff1442b`, [Propagator.cpp](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp): endpoint averaging, density/time scaling, G Q G^T and first-estimate refresh in `predict_and_compute`; `compute_Xi_sum`; `compute_F_and_G_analytic`; covered-interval selection in `select_imu_readings`. [Propagator.h](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.h): `interpolate_data`. Calibration is fixed/ideal here; coefficient evaluation and rejection of missing coverage remain local choices.
11. The same revision, [StateHelper::EKFUpdate](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/StateHelper.cpp), [JPLQuat::update](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/types/JPLQuat.h) and [quaternion operations](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/utils/quat_ops.h): covariance update/reset policy, increment, product and rotation-action translation.

12. The same revision, [CamRadtan](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/cam/CamRadtan.h): radial-tangential forward map and derivative; [TrackDescriptor](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/track/TrackDescriptor.cpp): separate raw and normalized point coordinates; [UpdaterHelper](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/update/UpdaterHelper.cpp): raw-pixel residual, frozen pose/projection factors and current-point distortion derivative. The fully frozen distortion derivative, inverse iteration settings and frontend noise acceptance are local choices.

The pinned source is a design reference, not a claim of implementation identity.
In particular, this project uses Hamilton/right-local Exp injection, Joseph form,
its own coefficient evaluation, fully frozen visual derivatives, bounded storage
and atomic failures. Temporary-feature refinement follows the local profile.
OpenVINS's normalized increment, covariance subtraction, numerical branches,
runtime settings and online calibration behavior are not inherited.
