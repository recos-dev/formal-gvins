# F-TRIANGULAR: Lower-triangular solve

**ID:** `F-TRIANGULAR`

**Status:** Draft

**Applies to:** Active lower-triangular square L and one or more RHS columns B.

**Uses:** [F-BOUNDED](bounded-matrices.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

Forward substitution solves `L*X=B` without an inverse. L need not be a Cholesky
factor: negative nonzero diagonal entries are permitted by this standalone API.
See [solve_lower](../../../../../include/formal_gvins/linalg/cholesky.hpp).

## Required behavior

Checks shall occur in this order:

| Condition | Response |
|---|---|
| Empty/nonsquare L, incompatible B rows, or zero RHS columns | `out_of_range` |
| Non-finite active L or B | `non_finite_input` |
| Any upper-triangular entry compares unequal to zero | `domain_error` |
| Any diagonal compares equal to zero | `zero_or_unsafe_divisor` |
| Non-finite computed solution | `non_finite_result` |
| Otherwise | Publish X with B's active shape and return success. |

No implicit near-zero cutoff is imposed. Callers own any conditioning bound.
All RHS columns shall use the same L. Failure shall preserve the entire output;
whole-object aliases to either input shall be supported when types permit.

## Acceptance and verification

Establish exact substitution under the nonsingular triangular premise. Check
negative diagonal, signed zero, structural error before zero divisor, overflow,
multiple RHS columns and aliases. Bound `||L*X-B||` under an identified numerical
domain; a finite solution does not certify forward error or conditioning.
