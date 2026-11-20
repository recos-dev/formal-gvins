# M-CORRECT: Joint correction, injection and reset

**ID:** `M-CORRECT`

**Status:** Draft

**Applies to:** Accepted local constraints on a valid joint state/covariance.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [M-STATE](state.md), [M-CONSTRAINT](constraints.md), [M-FEJ](consistency.md), [F-SOLVE](../math/linalg/spd-solve.md), [SO3-EXP](../math/so3/exp.md), [F-JR](../math/so3/right-jacobian.md), [V-CONSUMPTION](../vision/observation-consumption.md).

## Basis

[Joint correction, injection and reset](../../architecture/msckf-formulation.md#joint-correction-injection-and-reset)
specifies Joseph form and right-local coordinate transport for every attitude.

## Required behavior

For residual r, H and SPD measurement covariance V, the exact target is

$$
S=HP^-H^\mathsf T+V,\quad KS=P^-H^\mathsf T,\quad d=Kr,
\qquad P_c=(I-KH)P^-(I-KH)^\mathsf T+KVK^\mathsf T.
$$

Use an SPD solve without an explicit inverse. Exact $P^-\succeq0$ and $V\succ0$
give SPD S, but do not guarantee floating-point success. Any treatment of numerical
asymmetry shall be explicit and qualified before the exactly symmetric solve;
the solve shall not silently repair covariance.

Inject additive errors into current position/velocity/biases and every clone
position. For each attitude a, use `q_a'=q_a*Exp_q(d_theta_a)` under the quaternion
normalization/profile contract. Reset with block-diagonal G containing `J_r` for
each injected attitude and identity for additive states: `P'=G*P_c*G^T`.
Transport all cross-blocks and FEJ coordinate maps, then reset the error mean to
zero. Updating only the current IMU or attitude diagonal blocks is insufficient.

All numerical candidates, FEJ maps and consumed-observation records shall succeed
together before publication. Failure shall preserve the pre-correction joint
estimate and records. Explicit rejection/retirement is separate from consumption.
Approximation selections shall not silently replace this reset or Joseph model.

## Acceptance and verification

Establish Kalman/Joseph identities, PSD preservation in exact arithmetic and the
reset derivative. Check all clone/IMU cross-blocks, zero/nonzero corrections and
failure at solve/normalization/reset/cache stages. Bound numerical residuals and
coordinate conditioning under an accepted correction domain; nonlinear covariance
transport remains first-order despite the closed-form reset Jacobian.
