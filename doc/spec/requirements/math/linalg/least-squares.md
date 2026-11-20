# F-LEAST-SQUARES: Full-column-rank least squares

**ID:** `F-LEAST-SQUARES`

**Status:** Draft

**Applies to:** Active A of size m by n with m >= n > 0, and nonempty RHS B.

**Uses:** [F-QR](qr-reduction.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

For exact full-column-rank A, X minimizes `||A*X-B||_F^2`.
When m=n this is a general nonsingular solve, not an SPD-only operation.
See [least_squares](../../../../../include/formal_gvins/linalg/qr.hpp).

## Required behavior

Reject m<n as `out_of_range`, then perform F-QR's shape/domain checks and
reduction. Propagate its failures. If the reported rank differs from n, return
`ill_conditioned`; this status denotes failure of the chosen rank criterion,
not a computed condition number.

Solve the leading triangular system and undo the column permutation so output
rows correspond to the original A columns. Reject a non-finite solution as
`non_finite_result`. Success shall publish active shape n by RHS columns;
failure shall preserve output and permitted whole-object input/output aliases.
Do not form normal equations, an explicit inverse or a minimum-norm fallback.

## Acceptance and verification

Establish the exact least-squares minimizer for full-column-rank A. Test
rectangular and nonsymmetric square systems, original variable ordering, deficient
rank, failures and aliases. Compare independent QR solutions and scaled residuals
including `A^T*(A*X-B)`; a small normal residual alone does not bound forward error
on an ill-conditioned system. Required conditioning/error domains remain profile
obligations under C-NUMERICAL-ACCEPTANCE.
