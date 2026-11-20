# F-NULLSPACE: Left-nullspace action

**ID:** `F-NULLSPACE`

**Status:** Draft

**Applies to:** Active A of size m by n with m > n > 0, full column rank, matching RHS B.

**Uses:** [F-QR](qr-reduction.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

For orthogonal `Q=[Q1,Q2]` reducing full-column-rank A, `Q2^T*A=0` and
`Q2^T*Q2=I`. The operation returns `Q2^T*B` without explicitly constructing Q2.
See [project_left_nullspace](../../../../../include/formal_gvins/linalg/qr.hpp).

## Required behavior

Reject m<=n as `out_of_range`, then perform F-QR's remaining checks and reduction.
Propagate its failures. Reject reported rank different from n as `ill_conditioned`.
Success shall publish the last m-n rows of the transformed RHS, with all RHS
columns retained. Failure shall preserve output and permitted whole-object aliases.

The caller shall concatenate corresponding residual/Jacobian columns, or otherwise
ensure they receive the same transformation and row ordering. The primitive does
not establish measurement independence, physical feature identity or acceptable
triangulation. It shall not assign rank three to a deficient feature or construct
a lower-rank pseudoinverse fallback.

## Acceptance and verification

Establish annihilation/orthogonality and, for any b, the exact identity
`min_x ||b-A*x||^2 = ||Q2^T*b||^2`. Check dimensions, threshold-dependent rank,
aliases and transformed blocks. Numerical acceptance shall bound annihilation,
orthogonality and projected-information residuals; bit equality to another basis
is not required. Estimator gating/compression needs its own caller evidence.
