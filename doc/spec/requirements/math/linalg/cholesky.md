# F-CHOLESKY: Lower Cholesky factorization

**ID:** `F-CHOLESKY`

**Status:** Draft

**Applies to:** Bounded active square matrices in the selected numerical profile.

**Uses:** [F-BOUNDED](bounded-matrices.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

For exact SPD A, the factor satisfies `A=L*L^T`, with lower-triangular L and
positive diagonal. The [implementation](../../../../../include/formal_gvins/linalg/cholesky.hpp)
exposes this factor for reuse; it is not a covariance repair operation.

## Required behavior

The operation shall reject an empty/nonsquare active shape as `out_of_range`,
then non-finite active input as `non_finite_input`, then unequal mirrored
coefficients as `not_positive_definite`. Symmetry uses scalar equality,
including equality of signed zeros; it does not use an implicit tolerance.

Factorization shall require positive computed pivots. A non-finite intermediate
or factor shall return `non_finite_result`; a finite nonpositive pivot shall
return `not_positive_definite`. On success, publish the matching active shape,
lower factor and zero upper entries. Failure shall preserve output, including
an alias to A. There shall be no jitter, symmetrization, eigenvalue clipping or
conditioning cutoff. Exact SPD alone does not guarantee floating-point success.

## Acceptance and verification

Establish exact factor correspondence and uniqueness under the SPD premise.
Test shape/finiteness/symmetry precedence, signed zero, failed pivots, arithmetic
overflow and aliases. Numerical acceptance shall name reconstruction residual
`||A-L*L^T||`, scale, input/conditioning domain and tolerances; success is not
that residual certificate. [Bounded tests](../../../../../tests/test_linalg_bounded.cpp)
are initial native evidence.
