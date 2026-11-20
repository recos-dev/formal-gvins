# M-FEJ: First estimates and coordinate consistency

**ID:** `M-FEJ`

**Status:** Draft

**Applies to:** Conventional FEJ local VIO with the identity-reset approximation;
fixed calibration/gravity and no absolute GNSS/map constraints.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [M-STATE](state.md), [F-JR](../math/so3/right-jacobian.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

[Consistency and linearization](../../architecture/msckf-formulation.md#consistency-and-linearization)
derives the full FEJ transition and gauge composition. The reference is
[OpenVINS analytic FEJ propagation](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp),
translated to this project's Hamilton/right-local convention and state order.
M-PROPAGATE adopts that path's endpoint inputs and analytical discrete-noise
construction. Exp injection, numerical coefficient evaluation and fully frozen
camera distortion derivatives remain explicit project choices. The pinned visual
path keeps the distortion derivative at the current point while freezing the pose
and normalized-projection factors; the linked derivation explains the difference.
This is not whole-filter equivalence to OpenVINS.

## Required behavior

First estimates shall have the following lifetimes:

| Quantity | Binding |
|---|---|
| Current IMU pose/velocity | Initialize from the accepted initial state; replace only with the newly propagated nominal at each new time, before correction there. Hold through all corrections at that time. |
| Clone pose | Copy the current IMU's first estimate when cloned; freeze until removal. Copy the nominal pose separately, since the two can differ. |
| Temporary feature | One triangulated global point per candidate assembly, common to all rows; rebuilding a candidate rebuilds every row consistently. |
| Biases/held input | Current corrected bias means and declared interval input for the nominal propagation and interval integrals. |

Residuals shall use current nominal poses and the candidate feature estimate.
All geometric Jacobian factors, including projection and distortion derivatives,
shall use the common frozen poses/feature point. Current and frozen projections both require
valid domains. Use `Hx=Hx0`, `Hf=Hf0` directly, without reset-coordinate maps.

For interval h, let the new first estimate be the propagated nominal endpoint.
Define `D_R=(R_next0)^T*R_previous0`,
`dv0=v_next0-v_previous0-g_W*h`,
`dp0=p_next0-p_previous0-v_previous0*h-g_W*h^2/2`, and `phi=omega*h`.
Use A1, A2, B1, B2 of the held-input model with current corrected biases.
In error order `(p,v,theta,ba,bg)`, the complete covariance transition shall be

$$
\Psi=\begin{bmatrix}
I&hI&-[dp^0]_\times R^0&-R^0A_2&R^0B_2\\
0&I&-[dv^0]_\times R^0&-R^0A_1&R^0B_1\\
0&0&D_R&0&-hD_R J_r(-\phi)\\
0&0&0&I&0\\
0&0&0&0&I
\end{bmatrix},\qquad R^0=R_{\mathrm{previous}}^0.
$$

In particular, bias-sensitive blocks shall use the frozen attitude factors
shown here, not unspecified blocks from the current-state physical transition.
The joint transition is `diag(Psi,I_clones)`; M-PROPAGATE owns Qd and covariance
propagation, using the same frozen attitude factors in Gd. Corrections shall
retain first estimates and apply M-CORRECT's
identity reset. No accumulated T map, map solve or `Hx0*T^-1` is part of this
baseline. Adding a physical Jr reset requires a separately reviewed consistency
model; it shall not be inserted only into covariance.

## Acceptance and verification

Using the formulation's frozen four-column gauge `N0=N(x0)`, establish
`Hx0*N0+Hf0*Nf0=0`, `H_o*N0=0`, `diag(Psi,I)*N0_previous=N0_next`, and
`(I-K*H_o)*N0=N0`. Establish compatible cloning, identity reset and selection.
Test repeated corrections, propagation and clone retirement together.

Check every block against the translated analytic reference structure, including
bias blocks not constrained by the gauge identities. With first/current estimates
equal, Psi shall reduce to the ordinary ZOH derivative, using
`E(h)^T*J_r(-phi)=J_r(phi)`. Check nonidentity rotations, nonzero biases and
convention/state-order translation independently of gauge tests.

These obligations concern the selected linear model. They do not establish exact
nonlinear covariance, physical gauge transport under finite injection, whole-filter
OpenVINS equivalence or statistical consistency. Numerical acceptance and synthetic
Monte Carlo/trajectory studies remain separate work.

## Open decisions

The [development profile](../../profiles/local-vins.md) proposes a correction
domain; its adequacy for the identity-reset/frozen-linearization approximations
still needs owner review and empirical qualification. GNSS/map observations
change the information model and require a separate derivation.
