# M-PROPAGATE: Held-input joint propagation

**ID:** `M-PROPAGATE`

**Status:** Draft

**Applies to:** Positive qualified IMU intervals with constant held body inputs, bias means and continuous noise density within each interval.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [M-STATE](state.md), [M-FEJ](consistency.md), [V-ACQUISITION](../vision/acquisition.md), [SO3-EXP](../math/so3/exp.md), [C-FAILURE](../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

[IMU propagation](../../architecture/msckf-formulation.md#imu-propagation)
defines the ZOH nominal map, ordinary derivative and continuous-white-noise
integral. This differs from constant acceleration held in the world frame.

## Required behavior

Let `a=f_m-b_a`, `omega=omega_m-b_g`, interval h, initial attitude R and
`E(s)=Exp([omega]_x*s)`. Define
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
silently become `I+F*h`. For continuous density Qc, process noise shall target

$$
Q_d=\int_0^h\Phi^{ZOH}(h,s)G(s)Q_cG(s)^\mathsf T
                 \Phi^{ZOH}(h,s)^\mathsf T\,ds.
$$

Anisotropic/correlated densities shall retain their declared frame and blocks.
Sample variance shall not be substituted for density without a sampling-model
conversion. The actual covariance transition shall use M-FEJ's coordinate-bound
IMU block Psi: `P_II'=Psi*P_II*Psi^T+Qd`, `P_IC'=Psi*P_IC`, `P_CC'=P_CC`.
Retain the transpose cross-block consistently. The FEJ pair is an intentional
linearization approximation, not the physical transition/noise pair of the ODE.

Splitting at an exposure time shall retain the held sample and compose independent
white-noise increments in time order; it shall not inject the original full-interval
noise twice. Candidate state/covariance/time/cache updates shall publish together
only after required numerical checks. Invalid intervals or failed calculations
shall leave that propagation transaction unchanged.

## Acceptance and verification

Establish the held-input solution/derivative, zero-rate limits, interval composition
without intervening correction, and PSD of the exact noise integral.
Compare physical Phi/Qd against the derivation's
matrix-exponential oracle, including anisotropic noise and cross-covariances.
For zero rate and accelerometer white noise alone, require the `h^3/3`, `h^2/2`,
`h` position/velocity noise blocks, not a constant-random-acceleration substitute.
Qualify approximation/rounding, then test FEJ composition and failure atomicity.

## Open decisions

Production evaluation/quadrature order, accepted h/rate/input domains, residual
bounds and workspace/timing budgets remain unselected. The 30 by 30 exponential
is an oracle, not a required runtime algorithm.
