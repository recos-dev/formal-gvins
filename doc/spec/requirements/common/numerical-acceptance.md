# C-NUMERICAL-ACCEPTANCE: Numerical residual acceptance

**ID:** `C-NUMERICAL-ACCEPTANCE`

**Status:** Draft

**Applies to:** Approximate numerical properties claimed for shared operations or estimator compositions.

**Uses:** [C-NUMERICAL-PROFILE](numerical-profile.md), [C-INPUT-DOMAIN](input-domain.md).

## Basis

Adapted from the formal-eskf numerical policy identified in the
[specification index](../../README.md#reuse-and-evidence). Numerical tolerances
belong to identified properties and profiles, not one global epsilon.

## Required behavior

Each approximate acceptance property shall identify its input domain, units,
scalar/backend profile, residual, absolute tolerance and, where applicable,
relative tolerance and scale. For a scalar residual the comparison may be

$$
|a-b|\le\epsilon_{\mathrm{abs}}+\epsilon_{\mathrm{rel}}s,
\qquad s\ge0.
$$

The profile shall define $s$ rather than selecting it after observing a result.
Matrix and geometric checks shall name their residual, for example
$\max|SX-B|$, $\max|P-P^\mathsf T|$ or $|q^\mathsf Tq-1|$.
These are different claims and shall not share a tolerance without justification.

An acceptance claim shall distinguish:

- Exact identities and their mathematical premises.
- Executable checks, such as finiteness and failure preservation.
- Approximation, rounding and math-library error bounds.

A finite normalized quaternion does not by itself establish a specified unit-norm
residual. A symmetric finite covariance does not establish positive
semidefiniteness. A successful solve does not certify conditioning or forward
error. Numerical tests shall not be reported as proofs over untested inputs.

Missing domains or tolerances shall remain explicit acceptance gaps. They shall
not be filled by an implementation-dependent default or a hidden repair.

## Acceptance and verification

For each claimed property, check that its profile supplies the listed quantities
and the evidence actually covers them. Keep compiler/backend/target identity with
results. Review boundary and ill-conditioned cases; distinguish completed evidence
from requirements that merely state an obligation.
