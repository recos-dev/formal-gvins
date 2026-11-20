# F-NORMALIZE: Checked vector normalization

**ID:** `F-NORMALIZE`

**Status:** Draft

**Applies to:** Fixed nonempty vectors and a caller-supplied minimum norm.

**Uses:** [F-LINALG](matrix-operations.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

The exact target for nonzero v is `v/||v||`. The [vector operation](../../../../../include/formal_gvins/linalg/operations.hpp)
has a different threshold-status policy from quaternion construction; it shall
not inherit the latter's status remapping.

## Required behavior

The operation shall apply these checks in order:

| Condition | Response |
|---|---|
| Any non-finite input coefficient or minimum norm | `non_finite_input` |
| Non-finite computed norm | `non_finite_result` |
| Minimum norm nonpositive, or computed norm below it | `zero_or_unsafe_divisor` |
| Any non-finite coefficient after division by the computed norm | `non_finite_result` |
| Otherwise | Publish all normalized coefficients and return success. |

Threshold equality shall be accepted. No upper bound of one is imposed on the
threshold. Failure shall preserve the complete output; in-place normalization
shall be supported. The operation shall not select a sign or substitute a default
vector. [Q-NORMALIZE](../quaternion/normalization.md) owns quaternion-specific
validation order and status translation.

## Acceptance and verification

Establish exact normalization for nonzero reals. Check the ordered failures,
threshold equality, aliases and output preservation separately from unit-norm
residual bounds. Include simultaneous invalid conditions, computed-norm overflow
and subnormal input in each claimed numerical profile.
