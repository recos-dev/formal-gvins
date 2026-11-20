# M-PROPAGATE: Held-input joint propagation

**ID:** `M-PROPAGATE`

**Status:** Draft

**Applies to:** Positive qualified IMU segments with endpoint-averaged body inputs held constant, and constant bias means and continuous noise density within each segment.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [M-STATE](state.md), [M-FEJ](consistency.md), [V-ACQUISITION](../vision/acquisition.md), [SO3-EXP](../math/so3/exp.md), [C-FAILURE](../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

[IMU propagation](../../architecture/msckf-formulation.md#imu-propagation)
defines endpoint averaging, the resulting held-input nominal map, its ordinary
derivative and the interval-equivalent noise construction adapted from OpenVINS's
analytical path. The body-input hold differs from constant acceleration held in
the world frame. The selected Qd approximates continuous stochastic dynamics;
it is not the exact continuous-white-noise integral.

## Required behavior

The acquisition adapter shall provide calibrated measurements in B at both segment
endpoints. An endpoint between real samples shall use linear interpolation between
those samples; an exact sample timestamp shall use that sample. Segments shall
not span an interior real sample. Missing coverage shall return not-ready without
extrapolation. Subdivision shall not bypass the profile's original sample-gap limit.

For a segment `[t_l,t_r]`, define `h=t_r-t_l` and

$$
a=\tfrac12(f_m(t_l)+f_m(t_r))-\hat b_a,\qquad
\omega=\tfrac12(\omega_m(t_l)+\omega_m(t_r))-\hat b_g.
$$

Propagation shall hold these inputs constant over the segment, using the current
bias means for both endpoints. This is midpoint input approximation followed by
analytical integration, not a midpoint Runge-Kutta solver. With initial attitude R
and `E(s)=Exp([omega]_x*s)`, define
`A1=integral_0^h E(s) ds` and `A2=integral_0^h (h-s)E(s) ds`.
Propagation shall target

$$
p'=p+vh+RA_2a+\tfrac12g_Wh^2,\quad
v'=v+RA_1a+g_Wh,\quad q'=q\otimes\operatorname{Exp}_q(\omega h).
$$

Bias means shall remain unchanged. Position shall use initial velocity.
At zero angular rate use continuous limits `A1=hI`, `A2=h^2 I/2`.
Historical nominal poses shall remain unchanged.

The ordinary physical transition shall be the derivative of this held-input map,
including the gyro-bias translation blocks in the linked derivation; it shall not
silently become `I+F*h`.

For continuous density Qc in noise order `(n_a,n_g,n_ba,n_bg)`, define the
interval-equivalent rate covariance `Q_epsilon=Qc/h`. Let `R0` be the cached
start attitude and `D_R=(R_next0)^T*R0` as in M-FEJ. With the linked derivation's
translation sensitivities B1/B2 and `phi=omega*h`, process noise shall target

$$
G_d=\begin{bmatrix}
-R^0 A_2&R^0 B_2&0&0\\
-R^0 A_1&R^0 B_1&0&0\\
0&-hD_RJ_r(-\phi)&0&0\\
0&0&hI&0\\
0&0&0&hI
\end{bmatrix},\qquad Q_d=G_d\frac{Q_c}{h}G_d^\mathsf T.
$$

The coefficient inputs shall use current corrected biases; the attitude factors
shall use the specified FEJ endpoints. Qc shall be symmetric PSD in its declared
noise coordinates. The first profile uses independent isotropic blocks;
anisotropic/correlated PSD densities shall retain their declared frame and blocks
when supplied. This generalization does not inherit OpenVINS's scalar-noise
configuration. Measured sample variance shall not be substituted for density
without a sampling-model conversion.

The actual covariance transition shall use M-FEJ's complete
IMU block Psi: `P_II'=Psi*P_II*Psi^T+Qd`, `P_IC'=Psi*P_IC`, `P_CC'=P_CC`.
Retain the transpose cross-block consistently. With independent drivers, new bias
random-walk noise enters only the bias blocks; existing bias uncertainty still
propagates through Psi. Do not add within-segment bias-noise cross-blocks from a
different discretization.

Splitting at an exposure time shall use the interpolated endpoint and a separate
endpoint average for each part. After a correction, the following segment shall
recompute its effective inputs using the updated biases; interpolated measurements
shall remain bias-independent. Compute Gd and Qc/h separately for each positive
segment duration and compose `Q_total=Psi*Q_total*Psi^T+Qd` in time order.
The model treats segment disturbances as independent; averaging shall neither
halve Qc nor inject a separate endpoint disturbance. The original full-interval
noise shall not be injected twice. Subdivision generally changes the resulting Qd,
even with identical held inputs; split/unsplit noise equality is not required.

Candidate state/covariance/time/cache updates shall publish together
only after required numerical checks. Invalid intervals or failed calculations
shall leave that propagation transaction unchanged.

## Acceptance and verification

Establish the held-input solution/derivative, zero-rate limits, noise/state-order
translation and exact PSD of the Gd congruence. With zero rate, aligned current/FEJ
estimates and only accelerometer density Sa, require `R*Sa*R^T` times `h^3/4`,
`h^2/2`, `h` in the position/velocity noise blocks. Independent bias-only drivers
shall give their density times h in the corresponding bias block and zero
elsewhere. Retain all cross-blocks induced by Gd and a supplied correlated Qc.

Verify split/unsplit mean and ordinary-transition composition for identical held
inputs without correction. Verify noise against the selected segment recurrence,
not the continuous-white-noise integral or split/unsplit equality. Vary endpoint
readings, frozen/current attitudes and biases; test exact/interpolated boundaries,
missing coverage and excessive raw gaps. Compare coefficient evaluation and the
mapped analytical reference independently, including small-rate branch differences.
Qualify approximation/rounding, then test FEJ composition and failure atomicity.

## Open decisions

The [development profile](../../profiles/local-vins.md) proposes evaluation order
and h/rate/input domains; numerical, statistical and timing qualification remain open.
