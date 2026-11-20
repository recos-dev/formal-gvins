# F-SOLVE: Symmetric positive-definite linear solve

**ID:** `F-SOLVE`

**Status:** Draft

**Applies to:** Fixed-size and bounded active-size linalg solves in the selected scalar/backend profile.

**Uses:** [C-NUMERICAL-PROFILE](../../common/numerical-profile.md), [C-INPUT-DOMAIN](../../common/input-domain.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

Active solves also use [F-BOUNDED](bounded-matrices.md),
[F-CHOLESKY](cholesky.md) and [F-TRIANGULAR](triangular-solve.md).

## Basis

The linalg solve surface replaces inversion in Kalman-gain calculation.
The [fixed implementation](../../../../../include/formal_gvins/linalg/solve.hpp)
and [active implementation](../../../../../include/formal_gvins/linalg/cholesky.hpp)
share the mathematical operation. Use the [shared domain](../../common/input-domain.md)
and [numerical acceptance](../../common/numerical-acceptance.md) obligations.

## Required behavior

For a square symmetric positive-definite system S and matching right-hand side B,
solve S X = B using Cholesky LLT and triangular solves. Reuse one factorization
for all right-hand-side columns; do not form an explicit inverse.

The exact model is S = L L^T with a lower-triangular L and positive diagonal,
L Y = B, then L^T X = Y. The right solve X S = B uses
S X^T = B^T under the same symmetry premise.

| Condition | Required response |
|---|---|
| Active system empty/nonsquare, mismatched RHS dimension, or zero RHS count | `out_of_range` before coefficient checks |
| Non-finite coefficient in S or B | `non_finite_input` before factorization |
| Mirrored finite coefficients of S are unequal under scalar comparison | `not_positive_definite` |
| Failed positive-definite factorization | `not_positive_definite` |
| Non-finite factor or solution | `non_finite_result` |
| Successful factorization and finite solution | `success`, publishing the complete solution |

Propagate backend failure statuses. Preserve output on every failure. Output may
alias either complete input when dimensions match; otherwise storage is disjoint.
For active matrices, matching storage types permit whole-object aliases even when
the output's previous active shape differs. Success sets the solution's active
shape to that required by the equation. Only active coefficients are consumed;
capacity and access follow [F-BOUNDED](bounded-matrices.md). Fixed dimensions are
positive compile-time premises. For a right solve, RHS columns equal the system
size and RHS rows are the nonzero RHS count.
The right solve has the same domain/publication contract. The caller owns cleanup
of roundoff asymmetry before calling; the solve shall not silently symmetrize S.
Symmetry is numerical equality, not bitwise identity; positive and negative zero
compare equal. A detected non-finite factorization intermediate is a numerical
result failure, not evidence that the exact input matrix is non-SPD.

No condition-number cutoff is imposed by this API. Mathematical SPD does not
make a floating-point factorization infallible. Success establishes the stated
checked computation, not a certified condition number, exact residual or
forward-error bound. Deployment conditioning and residual limits remain separate
obligations under the shared numerical acceptance contract.

## Acceptance and verification

Check factor/triangular-solve coefficient correspondence, status propagation,
non-finite and non-SPD cases, multiple right-hand sides, and all permitted whole
object aliases. Include active shape-error precedence, inactive poisoned storage
and shape publication. Exact solve theorems, bounded production-source checks
and backend conformance establish different claims; accepted numerical error needs explicit
input/conditioning domains and residual bounds.
