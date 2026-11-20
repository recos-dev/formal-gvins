# MSCKF linear algebra inventory

**Status:** Draft implementation assessment

This inventory compares the [MSCKF formulation](../spec/architecture/msckf-formulation.md)
with the imported numerical layer. It is not an approved requirement or a proof
coverage report. It does not change the proposed estimator equations.

The [mathematical requirements](../spec/README.md#reusable-mathematics) own
operation contracts; this inventory explains their MSCKF use and implementation
boundary. Changes to behavior must update those canonical requirements.

| Use in MSCKF | Imported support | Addition in this increment |
|---|---|---|
| Fixed IMU/SO(3) blocks, products, transpose, norms and covariance congruences | Fixed `Matrix`, `operations.hpp`, SO(3) | Reuse unchanged. |
| Current state plus a varying number of pose clones | Compile-time dimensions only | Fixed-capacity storage with checked active extents; active products, transpose and block copies. |
| Clone removal and observation assembly | Compile-time blocks only | Checked indexed row/column selection and runtime-positioned blocks; preserve full cross-covariance. |
| Whitening with correlated measurement covariance | SPD solve returns a complete solution, not a whitening factor | Cholesky factor and lower-triangular solves. Applying V inverse instead of L inverse is not whitening. |
| Feature-position elimination | None | Rank-aware orthogonal reduction of H_f while applying the same row transforms to H_x and r. |
| Constraint compression | None | Reuse QR row transforms; retain the column permutation. Rank truncation needs its own error criterion. |
| Triangulation linear subproblems | Fixed SPD only | Full-column-rank least squares via QR, without normal equations or a matrix inverse. |
| Innovation and joint correction | Fixed SPD left/right solve | Active-size SPD solve and products, keeping the existing fixed-size API. |

## Implementation boundary

Bounded operations sit alongside fixed `Matrix`; they do not replace the small
SO(3)/IMU interface or introduce expression templates. Storage remains supplied by
the selected backend. Active loops use runtime extents bounded by template
capacities. No Eigen/OpenCV types, heap allocation, exceptions or device calls
enter these shared operations.

The initial bounded factorization path uses portable column-pivoted Givens QR
and Cholesky loops over existing storage/scalar access. This gives Eigen and
CMSIS-DSP the same active-size semantics without padding an active problem to its
maximum size or relying on undocumented allocation behavior of backend QR types.
It is a correctness baseline, not a backend-performance claim. Backend-specific
factorizations can replace the kernels later under the same residual tests.

Rank acceptance is an explicit absolute/relative pivot threshold. It is not a
condition-number guarantee. Rank-deficient features are rejected under the current
formulation. The QR result retains trailing rows and the column permutation;
calling code decides whether approximation permits any truncation. A nullspace
operation accepts only full-column-rank input and retains transformed RHS rows
below that rank. Tests need orthogonality/annihilation and least-squares identities,
not a particular sign convention for Q or coefficient equality with another QR.

Full-rank rectangular QR also supports square systems. The selected conventional
FEJ baseline has no attitude-coordinate-map solve.
General eigensolvers, SVD/pseudoinverse, sparse factor-graph solvers and GPU linalg
are not needed for this selected first profile. Degenerate triangulation is
rejected rather than silently assigned a minimum-norm solution.

## API and caller responsibilities

| Header | Operations and behavior |
|---|---|
| [bounded.hpp](../../include/formal_gvins/linalg/bounded.hpp) | `BoundedMatrix`, checked active extents, block extraction/insertion, indexed gather, products, sums and transpose. `resize` zeroes the new active rectangle; it does not preserve its previous contents. |
| [qr.hpp](../../include/formal_gvins/linalg/qr.hpp) | `qr`, `least_squares`, `project_left_nullspace`. The caller supplies absolute and relative rank tolerances; solve/projection reject insufficient rank. |
| [cholesky.hpp](../../include/formal_gvins/linalg/cholesky.hpp) | `cholesky`, `solve_lower`, `whiten`, active left/right SPD solves. Covariance input must be exactly symmetric; Cholesky requires positive computed pivots. No implicit jitter, symmetrization or inverse. |

Checked operations preserve outputs on failure, including input/output aliases.
Coefficient access itself requires valid active indices. Empty rectangles are
allowed for storage/copies/products; decompositions and solves require nonempty
systems and RHS. Copies preserve raw coefficients, including non-finite values;
arithmetic rejects non-finite active inputs/results. Inactive coefficients never
participate in arithmetic. Gather allows repeated indices as a mathematical
selection; clone-index uniqueness remains an estimator responsibility.

For QR, `A[:, permutation] = Q * upper` and `transformed_rhs = Q^T * rhs`.
Whitening should pass the assembled residual/Jacobian columns together so they
receive the same transform. Nullspace projection returns the trailing transformed
RHS rows. Compression uses the returned permutation to maintain state-column
identity. A reported small rank does not authorize dropping a nonzero trailing
block or assigning a minimum-norm solution.

The kernels use fixed-capacity automatic candidates to preserve failure atomicity.
Object copies/initialization can touch full capacity even when arithmetic uses
smaller active dimensions. They are not zero-copy views. Capacity selection must
budget caller storage plus nested temporaries and obey backend compile-time
storage limits, including Eigen's fixed-size allocation limit. No target stack
or worst-case execution-time bound is established by the host tests.

## Review and evidence

The [bounded conformance tests](../../tests/test_linalg_bounded.cpp) cover
shapes/capacity, active/inactive storage, aliased block extraction,
index selection, non-finite values, aliases, rank loss, failed Cholesky and complete
output preservation. Numerical tests compare mathematical residuals and an
independent Eigen Householder-QR reference in float/double. Eigen tests enable
its allocation guard; the same numerical cases run with CMSIS-DSP storage. These
are native tests, not a mathematical proof or an ARM timing qualification.

Remaining system decisions are clone/track capacities, numerical acceptance bounds,
feature conditioning limits and the complete MSCKF transaction. These primitives
do not implement propagation, cloning, triangulation policy, FEJ, correction or
estimator initialization. Lean and bounded source proofs are separate future work.
