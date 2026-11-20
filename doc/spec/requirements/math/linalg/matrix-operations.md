# F-LINALG: Fixed matrix and vector operations

**ID:** `F-LINALG`

**Status:** Draft

**Applies to:** Fixed, positive compile-time dimensions; selected scalar/backend.

**Uses:** [C-INPUT-DOMAIN](../../common/input-domain.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

The [matrix value type](../../../../../include/formal_gvins/linalg/matrix.hpp) and
[operations](../../../../../include/formal_gvins/linalg/operations.hpp) expose
backend-independent mathematical values. Storage layout is not the coordinate
convention or coefficient-input order.

## Required behavior

A matrix shall own its coefficients, initialize them to zero, and copy by value.
`from_row_major` shall map input coefficient `i*C+j` to mathematical entry `(i,j)`.
Indices are zero-based; the caller establishes bounds. Identity is square only;
vector access is for a single column. Dimensions shall be compatible at compile
time. Backend storage/expression types shall remain behind the matrix interface.

Named operations and operators shall evaluate eagerly with these real targets:

| Operation | Target |
|---|---|
| Addition, subtraction, negation, scalar multiply/divide | Coefficientwise arithmetic; division requires a valid nonzero divisor. |
| Product | `(AB)[i,j] = sum_k A[i,k]*B[k,j]`. |
| Transpose, block extraction/insertion | Transpose indices or copy the selected coefficients; insertion preserves other entries, including for permitted whole-object aliases. |
| Dot, squared norm, norm | `sum_i a[i]*b[i]`, `sum_i v[i]^2`, and its nonnegative square root. |
| Cross | Right-handed three-vector cross product. |
| Trace, diagonal, max_abs | Sum of diagonal, diagonal vector, maximum coefficient magnitude. |
| Symmetrize, sandwich | `(A+A^T)/2` and `T*A*T^T`. No PSD repair is implied. |

These raw arithmetic operations have no failure status and shall not clamp or
repair results. The caller owns input/result finiteness and representable-domain
premises. Exact identities do not prescribe bit equality between backends.
`all_finite` shall inspect every coefficient and reject NaN/infinities.
`is_symmetric(A,t)` shall return false for non-finite A/t or negative t; otherwise
it tests the maximum absolute mirrored difference against t, rejecting non-finite
differences. Checked normalization is [F-NORMALIZE](vector-normalization.md),
and solves are [F-SOLVE](spd-solve.md).

## Acceptance and verification

Check coefficient correspondence on nonsquare shapes, value ownership, block
aliases, non-finite inspections and backend conformance. Establish exact algebra
separately from ordered floating-point behavior and residual bounds. Include
underflow/overflow-sensitive domains; no unchecked operator is an atomic checked
estimator transition. [Native tests](../../../../../tests/test_linalg.cpp) are
starting evidence, not universal or target qualification.
