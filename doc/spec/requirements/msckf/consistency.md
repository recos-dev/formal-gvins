# M-FEJ: First estimates and coordinate consistency

**ID:** `M-FEJ`

**Status:** Draft

**Applies to:** Proposed reset-aware FEJ local-VIO profile; no absolute GNSS/map constraints.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [M-STATE](state.md), [F-JR](../math/so3/right-jacobian.md), [F-LEAST-SQUARES](../math/linalg/least-squares.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

[Consistency and linearization](../../architecture/msckf-formulation.md#consistency-and-linearization)
derives this project's endpoint and coordinate-map proposal. It is not a claim
that OpenVINS uses these extra maps or that gauge identities prove statistical
consistency in all conditions.

## Required behavior

First estimates shall have the following lifetimes:

| Quantity | Binding |
|---|---|
| Current IMU pose/velocity | Newly propagated nominal at each new time, held through corrections at that time. |
| Clone pose | Current first estimate at clone creation, frozen until removal. |
| Temporary feature | One triangulated point per candidate assembly, common to all rows; a rebuild rebuilds every row consistently. |
| Biases/held input | Current corrected bias means and declared interval input for physical propagation/noise. |

Residuals shall use current estimates; every geometric Jacobian factor, including
projection derivatives, shall use the common frozen points. Both current and
frozen projections require valid domains. Do not mix per-row feature estimates.

Starting from the physical ZOH transition, replace exactly its three attitude
column blocks with

$$
\Phi^0_{\theta\theta}=(R_{k+1}^0)^\mathsf TR_k^0,\quad
\Phi^0_{v\theta}=-[v_{k+1}^0-v_k^0-g_Wh]_\times R_k^0,
$$
$$
\Phi^0_{p\theta}=-[p_{k+1}^0-p_k^0-v_k^0h-\tfrac12g_Wh^2]_\times R_k^0.
$$

Maintain attitude-coordinate maps T_a, initially I, so `delta x=T*e` for the frozen
linear model; T is block diagonal with identity on additive coordinates.
Use `Hx=Hx0*T^-1` via small solves. After injection/reset, use `T'=G*T`, keeping
first estimates fixed. Do not replace a reset derivative by a finite attitude
rotation or silently refresh frozen clone points.

At propagation set the new IMU map to I and retain clone maps; use
`Phi_use=T_next*diag(Phi0,I_clones)*T_previous^-1` with the physical Qd in the new
local coordinates. Copy maps/first estimates on cloning and remove them with
their clone. Failed or inadequately conditioned map solves shall not publish a
partial update or silently fall back to a different consistency policy.

## Acceptance and verification

Using the formulation's four-column translation/yaw gauge basis N, establish
`H_o*N=0`, `Phi_use*N_k=N_next` and compatible augmentation/reset/removal maps.
Test sequences with intervening corrections; single-step Jacobian checks alone
are insufficient. Numerical residuals, correction-size/conditioning limits and
trajectory/NEES-style statistical studies are separate acceptance work. Do not
claim that P has four zero eigenvalues, or that the modified Phi/noise pair is
the exact original continuous ODE model.

## Open decisions

The proposal needs owner review and qualified correction/map-conditioning bounds
before deployment. GNSS or map constraints change the gauge model and require a
separate derivation; no such acceptance is inherited here.
