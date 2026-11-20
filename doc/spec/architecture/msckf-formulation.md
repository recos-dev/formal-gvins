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
with the current IMU state does not remain unchanged.

### Discrete nominal model

The proposed mathematical baseline is zero-order hold (ZOH) of the calibrated
body-frame IMU inputs over each interval of length $h>0$. Bias means remain
constant within that interval. This does not assume constant acceleration in
the world: the rotating body changes the direction of the specific force.
The acquisition adapter must declare which sample represents each interval.
Splitting an interval at an image timestamp uses the same held input on its
two parts; it does not invent an independent IMU measurement.

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
"Exact" here is conditional on ZOH, not a claim that real IMU inputs are constant
or that floating-point evaluation has no error. This is a project derivation
under the declared conventions; analytical IMU integration is also discussed
in [8].

### Transition of the held-input model

The angular-rate bias affects translation during rotation. Define its two
translation sensitivities using the right Jacobian from the reset section:

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
blocks generally remain nonzero. The following constant-matrix construction
also evaluates the complete transition, without leaving these integrals as
undefined operations.

### Matched process-noise covariance

Assume $Q_c$ is constant over the interval. Rotate only position and velocity
errors into the instantaneous nominal body frame:

$$
\eta=D(s)\delta x_I,\qquad
D(s)=\operatorname{diag}(R(s)^\mathsf T,R(s)^\mathsf T,I,I,I),
\quad R(s)=RE(s).
$$

With $W=[\omega]_\times$, the transformed system has constant matrices:

$$
\dot\eta=A\eta+Bn,\qquad
A=\begin{bmatrix}
-W&I&0&0&0\\
0&-W&-[a]_\times&-I&0\\
0&0&-W&0&-I\\
0&0&0&0&0\\
0&0&0&0&0
\end{bmatrix},\qquad
B=\begin{bmatrix}
0&0&0&0\\
-I&0&0&0\\
0&-I&0&0\\
0&0&I&0\\
0&0&0&I
\end{bmatrix}.
$$

Consequently, one finite matrix exponential gives an exact reference evaluation
for this linearized ZOH/noise model [9]:

$$
\exp\!\left(h
\begin{bmatrix}A&BQ_cB^\mathsf T\\0&-A^\mathsf T\end{bmatrix}\right)
=\begin{bmatrix}U&V\\0&U^{-\mathsf T}\end{bmatrix},
$$

$$
\Phi^{\mathrm{ZOH}}=D(h)^{-1}UD(0),\qquad
Q_d=D(h)^{-1}VU^\mathsf T D(h)^{-\mathsf T}.
$$

$D^{-1}=D^\mathsf T$. The construction supports anisotropic densities and
specified cross-correlations; it does not assume that every noise block is
$\sigma^2I$. The original integral establishes $Q_d\succeq0$ when
$Q_c\succeq0$. A computed matrix exponential still needs numerical error checks.

For example, with $\omega=0$, only accelerometer white noise of body-frame
density $S_a$, and all other driving densities zero, put $S_W=RS_aR^\mathsf T$.
Then

$$
Q_{pp}=\tfrac13S_Wh^3,\qquad
Q_{pv}=Q_{vp}^\mathsf T=\tfrac12S_Wh^2,\qquad Q_{vv}=S_Wh.
$$

Replacing this by a single constant random acceleration of variance $S_a/h$
would instead give $Q_{pp}=S_Wh^3/4$. Those are different stochastic models,
even though their velocity variances agree. Bias random walks also create
cross-covariances during the interval; adding only bias diagonal terms omits them.

The $30\times30$ exponential is a reference/oracle, not a required flight-time
implementation. A bounded positive-weight quadrature is a possible production
evaluation of the same integral:

$$
Q_d^{\mathrm{quad}}=h\sum_i w_i\,C(s_i)Q_cC(s_i)^\mathsf T,\qquad
C(s)=\Phi^{\mathrm{ZOH}}(h,s)G_I(s),\quad w_i>0.
$$

For three-point Gauss--Legendre quadrature, $s_i/h$ are
$1/2-\sqrt{15}/10,\ 1/2,\ 1/2+\sqrt{15}/10$, with weights
$5/18,\ 4/9,\ 5/18$. It preserves exact PSD and has local quadrature error
$O(h^7)$ for a smooth integrand with bounded derivatives. This is not a
uniform floating-point bound or a selection of production order/tolerances.
The accepted interval/rate domain and numerical implementation need separate
review against the reference.

Across independent continuous-white-noise intervals, compose in time order:

$$
\Phi_{20}=\Phi_{21}\Phi_{10},\qquad
Q_{20}=\Phi_{21}Q_{10}\Phi_{21}^\mathsf T+Q_{21}.
$$

Do not independently inject a full interval's noise twice when subdividing at
an exposure time. Correlated sample errors or an IMU providing integrated
increments require their own acquisition/noise model.

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

To initialize the temporary feature, convert each undistorted pixel
$z=[z_x,z_y]^\mathsf T$ to a camera bearing and write its world-frame
camera center:

$$
d_{cj}=\begin{bmatrix}(z_x-c_x)/f_x\\(z_y-c_y)/f_y\\1\end{bmatrix},
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
profile. At chosen nominal/feature linearization points,
stacking the feature's $m$ two-dimensional observations gives

$$
r_f=z_f-h_f(\hat x,\hat p_f)
\simeq H_x\delta x+H_f\delta p_f+n_f,
\quad H_x\in\mathbb R^{2m\times D},\quad H_f\in\mathbb R^{2m\times3}.
$$

These are ordinary differential Jacobians at a common linearization point.
The FEJ section below specifies the frozen points and coordinate transport used
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

For one attitude, the derivative follows directly from

$$
\delta\theta^+
=\operatorname{Log}\!\left(
\operatorname{Exp}(-[\widehat{\delta\theta}]_\times)
\operatorname{Exp}([\delta\theta]_\times)\right),\qquad
\left.\frac{\partial\delta\theta^+}{\partial\delta\theta}
\right|_{\delta\theta=\widehat{\delta\theta}}=J_r(\widehat{\delta\theta}).
$$

Every block touching a corrected attitude must be transported, including
IMU/clone and clone/clone cross-covariances. Resetting only attitude diagonal
blocks is incorrect. The FEJ linearization coordinates must also be transported,
as specified below.

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
2. Augment its IMU pose clone, first estimate and coordinate map; append frontend
   observations to feature tracks.
3. Select eligible tracks, including those affected by impending clone removal.
4. Triangulate, check geometry, build/whiten residuals, eliminate feature error,
   gate and optionally compress accepted constraints.
5. Correct, inject and reset the complete joint estimate and FEJ coordinate maps.
6. Mark consumed observations, retire obsolete records and remove eligible clones.

Following [V-TRACK](../requirements/vision/feature-tracks.md), commit the corrected
joint estimate, covariance, FEJ coordinate maps and observation-consumption
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
Whitening and eliminating the feature therefore give $H_oN=0$.
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
| Bias means and held IMU inputs | Use current corrected bias means and the declared interval input. | Nominal propagation and physical noise/other bias sensitivity blocks. |
| Calibration and gravity | Fixed profile constants. | Nominal model, residuals and Jacobians. |

Compute $r_f=z_f-h_f(\hat x,\hat p_f)$ at the current nominal poses and temporary
feature estimate. Compute every geometric factor of $H_x^0,H_f^0$, including
$J_\pi$, at $(p_j^0,R_j^0,p_f^0)$; mixing a current feature with different
per-row frozen features breaks the cancellation. Both current and frozen
projections need valid depth/domain. A rebuilt candidate rebuilds all its rows
at a common feature point; it cannot mix old factors with a new point.
These temporary features are not persistent EKF landmarks.

FEJ deliberately uses a Jacobian different from the derivative at the current
residual evaluation point. The equalities below concern that chosen linear
system; they do not remove its nonlinear linearization error.

### FEJ transition

Starting from $\Phi^{\mathrm{ZOH}}$ evaluated at the current corrected IMU state,
define $\Phi^0$ by replacing exactly three attitude-column blocks:

$$
\begin{aligned}
\Phi^0_{\theta\theta}&=(R_{k+1}^0)^\mathsf T R_k^0,\\
\Phi^0_{v\theta}&=-[v_{k+1}^0-v_k^0-g_Wh]_\times R_k^0,\\
\Phi^0_{p\theta}&=-[p_{k+1}^0-p_k^0-v_k^0h-\tfrac12g_Wh^2]_\times R_k^0.
\end{aligned}
$$

Keep the remaining blocks as specified by the physical ZOH transition.
Bias columns do not enter the gauge identities because the gauge has zero bias
components. Substitution gives

$$
\Phi^0N_I(x_k^0)=N_I(x_{k+1}^0).
$$

For example, the velocity yaw column becomes
$[u]_\times v_k^0-[v_{k+1}^0-v_k^0-g_Wh]_\times u
=[u]_\times v_{k+1}^0$, since $g_W$ is parallel to $u$.
The position identity follows in the same way, including the $hI$ velocity
block. Without intervening corrections these three blocks reduce to the
ordinary ZOH derivatives.

This is a first-estimate endpoint construction adapted to our convention from
the FEJ principle [7,8]. When first and current estimates differ, $\Phi^0$ is
not the derivative of the current nominal map. The process-noise covariance
still uses the physical ZOH integral, evaluated along the propagated nominal
trajectory. The resulting FEJ pair is a specified approximation: do not claim
that the modified $\Phi^0$ and unchanged $Q_d$ are jointly the exact transition
and noise of the original continuous error ODE.

### Transport through right-local resets

Frozen points alone are insufficient when retaining the explicit $J_r$ reset
above. A frozen Jacobian has columns in its reference linear-error coordinates,
whereas $P$ after reset describes new current-local coordinates. The proposed
adaptation keeps their change of basis explicit:

$$
\delta x=T e,\qquad
T=\operatorname{diag}(I,I,T_I,I,I,I,T_1,\ldots,I,T_N).
$$

Here $e$ is the FEJ linear model's reference error, not a finite difference
$x-x^0$. Each $T_a$ is a $3\times3$ attitude-coordinate map, initially $I$.
Only one joint covariance is needed: $P$ remains in current-local coordinates.
No additional physical state or independent uncertainty is introduced.
Only the $3\times3$ maps need storage, not a dense $D\times D$ matrix.
The gauge basis $N$ is an analysis object, not a required runtime state.

For an observation use

$$
H_x=H_x^0T^{-1},\qquad H_f=H_f^0,\qquad N=T N^0,\quad N^0=N(x^0).
$$

Implement the inverse action by small solves. After injection/reset, leave
the first-estimate values unchanged and update

$$
T^+=GT^-,\qquad N^+=GN^- .
$$

This is the coordinate transport of a fixed linear model:
$H^{0}(T^+)^{-1}N^+=H^{0}(T^-)^{-1}N^-$.
It is not permission to replace $T_a$ by
$R(\hat q_a)^\mathsf T R(q_a^0)$; a finite rotation and a reset derivative
are different maps.

At the next IMU propagation, the new endpoint is its first estimate, so set
$T_{I,k+1}^-=I$ there. Retain each clone's $T_j$. The actual joint transition
in current-local coordinates is

$$
\bar\Phi_{\mathrm{use}}
=T_{k+1}^-\operatorname{diag}(\Phi^0,I_{6N})(T_k^+)^{-1}.
$$

Its clone block is identity. Use its IMU block and the physical $Q_d$ in the
joint covariance propagation above; the noise is already in the new endpoint's
local coordinates. If representing it in another basis, transform the noise
by the same congruence as the covariance.

At augmentation, copy the selected current pose's first estimates and $T_I$
into the new clone; retain the existing $J_c$ covariance formula.
At removal, delete the corresponding first estimates and coordinate-map block.
These rules make propagation, augmentation and removal agree with the same
linearized gauge representation.

This reset-aware transport is a project design proposal derived here, not a
claim that OpenVINS implements these extra maps. Its nonsingular, adequately
conditioned domain, correction-size limit and response to an ill-conditioned
map need acceptance bounds. Silently discarding $T$ or refreshing frozen clone
points would change the estimator. The algebra establishes coordinate/gauge
compatibility, not statistical consistency under arbitrary corrections.

### Composition and scope of the consistency claim

For the feature-eliminated Jacobian in current-local coordinates and the
declared propagation, the algebraic obligations are

$$
H_oN=0,\qquad
\bar\Phi_{\mathrm{use}}N_k=N_{k+1},\qquad
(I-KH_o)N=N .
$$

Whitening and row compression preserve the first identity. With
$A_c=[I;J_c]$, augmentation maps $N$ to $A_cN$; reset maps it to $GN$;
removal maps it to $EN$. Chaining these identities prevents these visual
constraints from adding direct information in the selected four gauge
directions of the linear model.

It does not require four zero eigenvalues in $P$, promise constant marginal
yaw variance, or prove estimator accuracy. Initialization priors, correlations,
nonlinear errors, numerical errors, data-dependent outlier selection and
additional degeneracies still matter. GNSS, absolute-heading sensors or map
feedback change the information/gauge model and need separate derivations.

## Implementation and verification boundary

The imported [linalg](../../../include/formal_gvins/linalg/linalg.hpp),
[scalar](../../../include/formal_gvins/scalar/math.hpp) and
[SO(3)](../../../include/formal_gvins/so3/so3.hpp) layers provide numerical
primitives. They do not yet provide active-size joint matrices, QR/nullspace
operations, general coordinate-map solves, an MSCKF state or a feature frontend.
The existing SPD solve cannot directly solve a nonsymmetric $T_a$.
Eigen and CMSIS-DSP are backend
choices; their external internals are not proved by shared mathematical theorems.

| Boundary | Intended evidence, not completed proof |
|---|---|
| Frames, IMU model and joint propagation | ZOH nominal/transition derivative; matched white-noise integral; interval composition; covariance symmetry/PSD under stated premises. |
| Augmentation and marginalization | Index/state correspondence, full cross-covariances and preservation of exact PSD. |
| Camera geometry and Jacobians | Derivative proofs, finite-difference cross-checks, depth/domain and calibration-coordinate checks. |
| Whitening, feature elimination and compression | Exact orthogonality/information properties; scoped rank, conditioning and floating-point residual bounds. |
| Correction and all-pose reset | Kalman/Joseph identities, right-local reset derivative, covariance transport and failure atomicity. |
| FEJ and observability | Endpoint gauge identities; frozen visual-factor cancellation; coordinate transport through propagation, correction/reset, augmentation and removal. |
| Track/clone lifecycle and resources | Bounded indexing/storage, valid time/identity association, once-only fusion and target timing/workspace measurements. |

Lean can address exact algebra and geometric properties. Scoped C++ verification
can address control flow, indexing and supported floating-point properties.
Backend tests and target experiments address separate numerical/resource claims.
An imported theorem, harness or passing native test is not an automatic proof
of the assembled estimator or of visual-feature correctness.

## Open decisions before implementation

| Decision | What must be fixed |
|---|---|
| Propagation implementation | Bind acquisition/interval semantics to the ZOH proposal; select bounded evaluation of $B_1,B_2,Q_d$, small-angle branches, error limits and timing/workspace budgets. |
| Vision profile | Actual camera/distortion model, mono/stereo configuration, synchronization accuracy and track-noise assumptions. |
| Geometry and innovation acceptance | Depth/parallax/conditioning limits, rank tolerance, triangulation iteration cap and gate confidence. |
| Consistency acceptance | Review the proposed FEJ endpoint/coordinate-transport design; set correction and conditioning limits, then evaluate statistical consistency as well as the algebraic gauge identities. |
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
8. OpenVINS, [analytical propagation](https://docs.openvins.com/propagation_analytical.html), especially mean integration, error-state evolution and the FEJ endpoint discussion. The state order, rotation convention, gravity sign and noise discretization here are defined locally, not copied from that implementation.
9. Charles F. Van Loan, [Computing Integrals Involving the Matrix Exponential](https://doi.org/10.1109/TAC.1978.1101743), IEEE Transactions on Automatic Control, 23(3):395--404, 1978. The block-exponential construction is applied here to the transformed constant-coefficient error system.

OpenVINS documentation was consulted on 2026-11-20. Its URLs are explanatory
references, not a pinned implementation baseline or a claim of behavioral identity.
Any future source import or code-level equivalence claim must identify the exact
upstream revision and recheck the affected conventions and assumptions.
