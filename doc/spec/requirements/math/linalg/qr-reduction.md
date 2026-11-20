# F-QR: Orthogonal reduction and numerical rank

**ID:** `F-QR`

**Status:** Draft

**Applies to:** Active A of size m by n, B of size m by k; m,n,k positive.

**Uses:** [F-BOUNDED](bounded-matrices.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

The [column-pivoted reduction](../../../../../include/formal_gvins/linalg/qr.hpp)
applies identical orthogonal row transformations to A and B. The exact relation is
`A[:,permutation]=Q*R`, `C=Q^T*B`, with orthogonal Q. Q need not be materialized.

## Required behavior

Reject invalid/empty shapes as `out_of_range`, then non-finite active A/B or rank
tolerances as `non_finite_input`, then absolute tolerance below zero or relative
tolerance outside `[0,1)` as `domain_error`. Let `scale` be the largest original
column norm, and `threshold=absolute+relative*scale`. Non-finite computed norms,
threshold or result shall return `non_finite_result`.

At each step, select the largest trailing column norm; ties retain the first
current column. Stop when that norm is at most threshold. Otherwise pivot and
apply Givens row rotations to eliminate entries below that pivot, applying the
same rotations to every B column. Return the number of processed pivots as rank,
the full active R/C shapes and a permutation of all n input columns. Only the
processed columns are guaranteed reduced if stopping occurs early.

Rank loss, including rank zero, shall be a successful reduction with a smaller
reported rank. Trailing coefficients shall remain present; the operation shall
not silently truncate nonzero entries. This rank policy is not a condition-number
bound or permission to treat a small block as exact zero. Failure shall preserve
the complete result, including permutation/rank and input aliases through result
members. Numerical reductions may differ from exact orthogonal algebra.

## Acceptance and verification

Check shape/tolerance precedence, zero/rank-deficient/rectangular systems, pivot
order, threshold equality, subnormals, overflow and member aliases. Establish
orthogonal transformation and permutation identities, then measure reconstruction,
orthogonality and RHS-transform residuals under explicit numerical bounds.
[Bounded tests](../../../../../tests/test_linalg_bounded.cpp) include an independent
QR reference; matching a particular sign of Q is not an acceptance condition.
