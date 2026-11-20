# F-BOUNDED: Bounded active matrices

**ID:** `F-BOUNDED`

**Status:** Draft

**Applies to:** Positive static capacities with runtime active extents.

**Uses:** [F-LINALG](matrix-operations.md), [C-CPP-SUBSET](../../common/cpp-subset.md), [C-FAILURE](../../common/failure.md).

## Basis

[BoundedMatrix](../../../../../include/formal_gvins/linalg/bounded.hpp) supports
variable joint-state/observation dimensions without heap allocation. Its storage
capacity and active mathematical shape are distinct.

## Required behavior

New objects shall have active shape `0 x 0`. Each active extent shall stay within
its static capacity. `resize(r,c)` shall reject excess capacity as `out_of_range`
without modification; success shall zero the entire new active rectangle and set
the extents. It shall not promise to preserve previous active values. `from_fixed`
shall copy coefficients and activate the full fixed shape. Coefficient access is
unchecked; callers establish active bounds. Inactive storage shall not participate
in arithmetic, finite checks or indexing premises for active operands.

| Operation | Shape and value behavior |
|---|---|
| Block extraction | Copy the specified active rectangle, including zero extents, to the result shape; reject source/capacity bounds first. |
| Block insertion | Copy into an existing active rectangle and preserve its shape and other entries; reject bounds before mutation. |
| Gather | Output entry `(i,j)` equals source at the supplied row/column indices. Check requested counts, then active index bounds. Order and repeated indices shall be preserved. |
| Transpose | Copy active entries with swapped dimensions. |
| Add | Require equal active shapes; result has that shape. |
| Multiply | Require left columns equal right rows; result shape is left rows by right columns. Empty inner sums are zero. |

Copies, gather and transpose shall preserve raw coefficient values, including
non-finite values; they do not validate numerical input. Add/multiply shall first
reject shape mismatch as `out_of_range`, then non-finite active inputs as
`non_finite_input`, then a non-finite candidate as `non_finite_result`.
Zero-sized active rectangles shall be permitted for these operations; decomposition
and solve contracts separately require nonempty systems.

Checked operations shall preserve complete outputs, including extents, on failure.
Whole-object input/output aliases allowed by the types shall work as if inputs
were read before output publication. No views or overlapping subobject aliases
are introduced. Success may replace inactive storage; callers shall not rely on
its contents. Full-capacity initialization/copies are permitted even though the
arithmetic domain is active only. The caller shall budget nested workspaces and
backend capacity limits. Gather does not enforce unique clone identities.

## Acceptance and verification

Exercise empty/full/over-capacity extents, boundary blocks, repeated/invalid
indices, shape-error precedence and aliases. Poison inactive storage and verify
it is ignored; snapshot complete objects on failure. Establish index/shape
invariants separately from arithmetic residuals and target stack/timing budgets.
[Bounded tests](../../../../../tests/test_linalg_bounded.cpp) provide initial
native cases, not a maximum-capacity deployment proof.
