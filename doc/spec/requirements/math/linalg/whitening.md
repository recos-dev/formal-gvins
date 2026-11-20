# F-WHITEN: Covariance whitening

**ID:** `F-WHITEN`

**Status:** Draft

**Applies to:** Active SPD covariance V and a matching nonempty column block B.

**Uses:** [F-CHOLESKY](cholesky.md), [F-TRIANGULAR](triangular-solve.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

For `V=L*L^T`, whitening applies `L^-1`, giving
`Cov(L^-1*n)=I` when `Cov(n)=V`. Solving `V*X=B` instead does not whiten.
The [implementation](../../../../../include/formal_gvins/linalg/cholesky.hpp)
uses the factor and triangular solve directly.

## Required behavior

The operation shall first reject an empty/nonsquare V, incompatible B rows or
zero B columns as `out_of_range`, then non-finite active V/B as `non_finite_input`.
It shall factor V once and solve `L*Y=B` for every column, propagating constituent
failure statuses. Success shall publish Y with B's active shape. Failure shall
preserve output, including permitted whole-object aliases. No inverse, silent
symmetrization or covariance adjustment shall replace the specified operation.

The caller shall assemble residual and all corresponding Jacobian columns using
one row ordering and covariance. Separate calls are equivalent only with the same
factor/model binding. The primitive does not establish that supplied V is the
physical measurement covariance or that different observations are independent.

## Acceptance and verification

Establish the exact covariance identity and weighted-norm identity
`||L^-1*b||^2 = b^T*V^-1*b` under SPD V. Test correlated, non-diagonal V,
multiple columns, ordered failures and aliases. Check factor/solve residuals and
whitened covariance error in the selected numerical domain. Existing bounded
tests do not establish the caller's statistical noise model.
