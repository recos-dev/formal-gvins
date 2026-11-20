# M-CORRECT: Joint correction, injection and reset

**ID:** `M-CORRECT`

**Status:** Draft

**Applies to:** Accepted local constraints on a valid joint state/covariance,
using M-FEJ's conventional FEJ baseline and a qualified correction domain.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [M-STATE](state.md), [M-CONSTRAINT](constraints.md), [M-FEJ](consistency.md), [F-SOLVE](../math/linalg/spd-solve.md), [SO3-EXP](../math/so3/exp.md), [F-JR](../math/so3/right-jacobian.md), [V-CONSUMPTION](../vision/observation-consumption.md).

## Basis

[Joint correction, injection and reset](../../architecture/msckf-formulation.md#joint-correction-injection-and-reset)
specifies Joseph form, Hamilton/right-local Exp injection and an identity-reset
approximation. The inspected [OpenVINS update](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/StateHelper.cpp)
also omits explicit reset transport. Its normalized small-angle injection differs
from the selected Exp injection at finite corrections; the derivation bounds that
difference. Neither reference usage nor the name FEJ proves statistical accuracy.

## Required behavior

For residual r, M-FEJ's Jacobian H and SPD measurement covariance V, target

$$
S=HP^-H^\mathsf T+V,\quad KS=P^-H^\mathsf T,\quad d=Kr,
\qquad P_c=(I-KH)P^-(I-KH)^\mathsf T+KVK^\mathsf T.
$$

Use an SPD solve without an explicit inverse. Exact $P^-\succeq0$ and $V\succ0$
give SPD S, but do not guarantee floating-point success. Numerical asymmetry
handling shall be explicit and qualified; the solve shall not silently repair P.

Inject additive errors into current position/velocity/biases and every clone
position. For every attitude a, use `q_a'=q_a*Exp_q(d_theta_a)` under the quaternion
normalization/profile contract. Validate every current/clone attitude correction
against the profile bound before publication; an excessive correction rejects
the whole candidate, without clipping, splitting it into smaller updates or
silently refreshing first estimates.

Reset the estimated error mean to zero and retain `P'=P_c`, equivalent to the
selected `G_FEJ=I_D`. Leave all first estimates unchanged. No covariance reset
multiplication or FEJ coordinate-map update is performed. The full Joseph matrix,
including all IMU/clone and clone/clone cross-blocks, shall be retained.

Identity reset is an approximation: the physical right-log reset derivative for
Exp injection is `J_r(d_theta)`, generally not I. Covariance here is the FEJ
linear model's approximation, not an exactly transported physical distribution.
Do not combine a partial Jr reset with the frozen H/Psi selected by M-FEJ.

All state/covariance candidates and consumed-observation records shall succeed
together before publication. Failure preserves the pre-correction joint estimate,
first estimates and records. Explicit rejection/retirement is separate from
successful consumption. Failed solves, excessive/non-finite corrections and failed
quaternion normalization shall not publish a partial update.

## Acceptance and verification

Establish Kalman/Joseph identities, exact PSD of P_c, Hamilton injection and the
selected identity-reset behavior. Test all current/clone means and cross-blocks,
zero/nonzero corrections, unchanged first estimates, correction-bound boundaries
and failure atomicity. Compose these checks with M-FEJ's gauge obligations.

Separately establish the physical reset derivative and its distinction from the
selected identity approximation. For maximum attitude correction d_max, the
one-step covariance comparison bound is
`norm(G_phys*P_c*G_phys^T-P_c)_2 <= (2*epsilon+epsilon^2)*norm(P_c)_2`,
where `epsilon=d_max/2`. It does not qualify accumulated or statistical error.
Also distinguish Exp from the reference's normalized increment: their finite
rotation difference is bounded by `norm(d_theta)^3/12` in the admitted small-angle
domain. Floating-point residuals and repeated-update trajectory/NEES acceptance
remain separate evidence; no exact physical-covariance claim follows from these
linear-model proofs.
