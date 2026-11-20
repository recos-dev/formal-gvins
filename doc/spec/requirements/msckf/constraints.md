# M-CONSTRAINT: Visual constraint elimination and acceptance

**ID:** `M-CONSTRAINT`

**Status:** Draft

**Applies to:** Eligible feature histories with qualified geometry and noise covariance.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [M-VISUAL](visual-geometry.md), [M-FEJ](consistency.md), [F-WHITEN](../math/linalg/whitening.md), [F-NULLSPACE](../math/linalg/nullspace-projection.md), [F-QR](../math/linalg/qr-reduction.md), [F-SOLVE](../math/linalg/spd-solve.md).

## Basis

[Feature elimination, gating and compression](../../architecture/msckf-formulation.md#feature-elimination-gating-and-compression)
defines the information-preserving operations and their statistical premises.

## Required behavior

For m two-dimensional observations, assemble `r,Hx,Hf` with identical row ordering,
then whiten them together using the same qualified SPD covariance `V=L*L^T`.
Accept only rank-three, sufficiently conditioned whitened Hf under declared
criteria. Apply an orthonormal left-nullspace action to all columns, giving
`r_o=U^T*L^-1*r`, `H_o=U^T*L^-1*Hx`, dimension `d=2m-3` and exact noise I.
Reject deficient features rather than assigning rank three or adding their
triangulated position as independent evidence.

Evaluate each feature's innovation gate before row compression:

$$
S_o=H_oP^-H_o^\mathsf T+I,\quad S_oy=r_o,\quad
\gamma=r_o^\mathsf Ty\le\chi^2_{d,1-\alpha}.
$$

Use the dimension d and a declared `0<alpha<1`; equality is accepted. Invalid
inputs, failed solves or invalid/non-finite gate statistics shall not yield an
accepted constraint. A negative computed statistic shall require rejection or a
reviewed numerical-error policy, never silent clamping. The chi-square probability
interpretation depends on the selected linearized Gaussian noise model.

Stack only accepted constraints under a declared cross-feature noise model.
If optional compression uses `Q=[Q1,Q2]` with `Q2^T*H=0`, retain `Q1^T*H` and
`Q1^T*r`, preserving state-column identity after any QR pivoting. State-insensitive
residual rows may be discarded after the gate. Merely small nonzero rows require
a separate approximation/error criterion; numerical rank alone does not authorize
discarding them. An empty accepted set shall skip correction, not create a zero
measurement. Failure shall not consume observations.

## Acceptance and verification

Establish whitening/noise, nuisance-elimination least-squares and compressed
information identities. Test deficient/near-rank features, correlated pixel noise,
column permutations, gate equality and solve failure. Check numerical annihilation,
orthogonality and information residuals. Thresholds, conditioning, cross-feature
correlations and any approximate compression criterion require review; raw inlier
counts do not qualify these statistical assumptions.
