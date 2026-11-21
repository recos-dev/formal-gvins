# Exact mathematical proof inventory

This inventory binds Lean statements to the current mathematical contracts and
C++ formula locations. It is a reviewed correspondence, not a mechanically checked
Lean-to-C++ refinement. Requirements remain Draft. A successful build proves the
statements under their displayed premises; it does not prove all requirement
clauses, floating-point executions, backend internals or physical model validity.

## Running

Install Elan and the version in `formal/lean/lean-toolchain`, then run:

```bash
./scripts/verify_lean.sh --cache  # first run: download Mathlib's compiled cache
./scripts/verify_lean.sh          # build local proofs; warnings fail
```

`LAKE` may select another Lake executable. The pinned manifest owns dependency
revisions. The cache accelerates dependencies; the local modules are built from
this repository's sources. Store execution logs in ignored `output/lean/` with
source identity and tool versions when retaining evidence. The runner does not
claim to be an evidence archive or coverage validator.

## Group 1: Quaternion and SO(3)

Sources: [unit quaternion](../../include/formal_gvins/so3/unit_quaternion.hpp),
[operations](../../include/formal_gvins/so3/operations.hpp),
[rotation vector](../../include/formal_gvins/so3/rotation_vector.hpp) and
[right Jacobian](../../include/formal_gvins/so3/right_jacobian.hpp).
Requirements: Q-IDENTITY, Q-MUL, Q-INVERSE, Q-NORMALIZE, Q-SIGN,
Q-ROT-COMPOSE, Q-ROT-MATRIX, Q-ROTATE, SO3-HAT, SO3-EXP, SO3-LOG,
SO3-SMALL-ANGLE and F-JR. The basis is scalar-first Hamilton multiplication, active body-to-world rotation
and right-local perturbations. Quaternion coefficient formulas, rotation direction,
Exp half angle, principal Log sign selection and Jacobian branches were compared
with these local sources; importing the proofs alone would not establish that match.

| Lean module / principal statements | Exact claim and premises |
|---|---|
| `Quaternion`: `hamiltonMul_assoc`, `normSquared_hamiltonMul`, `hasUnitNorm_normalize`, conjugate/inverse lemmas | Quaternion algebra; normalization requires positive squared norm; inverse by conjugation requires unit norm. |
| `SO3`: `rotationMatrix_hamiltonMul`, `rotationMatrix_mem_specialOrthogonalGroup`, `inverseRotate_rotate_of_unit`, `hat_mulVec` | Composition order, proper rotation, inverse action, length preservation and cross-product matrix. Rotation claims use unit quaternions. |
| `RotationVector`: `quaternionExp_hasUnitNorm`, `rotationMatrix_quaternionExp`, `quaternionLog_norm_le_pi`, `quaternionLog_quaternionExp_of_norm_lt_pi` | Exp, Rodrigues, principal Log range and local inverse. Opposite signs at the pi cut are not assigned a globally unique axis. |
| `RightJacobian`: `rightJacobian_axis`, `rightJacobian_halfAngle`, `rightJacobianTaylor_polynomial` | Equivalent exact evaluation forms and the separate small-angle polynomial. |
| `RightJacobianLimits`: `rightJacobianA_tendsto_zero`, `rightJacobianB_tendsto_zero`, remainder lemmas | Removable singularities and exact-real Taylor remainders, not rounded branch guarantees. |
| `QuaternionKinematics`: `constantRate_attitude_kinematics` | Hamilton constant-body-rate quaternion differential equation. |
| `RotationDifferential`: `quaternionExp_line_hasDerivAt_of_ne_zero`, `expCoefficientDerivative_right_trivialized`, `resetAttitude_tangent_matches_exp` | Directional Exp differential, including a separate origin argument, with right-local Jacobian. The physical Log reset is separate from the MSCKF identity reset. |

These modules adapt the owner's formal-eskf foundation at revision
`f98c692e325d1a007006dd02b4271cf51e4c135c`, under this project's Apache-2.0
licensing. INS translation/state/reset policies were not imported as MSCKF claims.
Proof sources are under [formal/lean/FormalGVINS](../../formal/lean/FormalGVINS).

## Group 2: Linear algebra

Sources: `linalg/bounded.hpp`, `cholesky.hpp`, `qr.hpp`, `solve.hpp`.
Requirements: F-LINALG, F-BOUNDED, F-CHOLESKY, F-TRIANGULAR, F-SOLVE,
F-QR, F-LEAST-SQUARES, F-WHITEN and F-NULLSPACE.

| Lean module / principal statements | Exact claim and premises |
|---|---|
| `LinearAlgebra`: row-major round trips, block extraction/replacement, `block_admission`, `inactive_storage_irrelevant`, `active_product_entries` | Valid active indices stay inside capacity; inactive coefficients do not enter products. Mathematical natural indices do not prove machine-size overflow behavior. |
| `Solve`: `choleskyRecurrences_factor`, `cholesky_positive_pivot`, substitution equations | Symmetric input and successful nonzero-pivot recurrences reconstruct LLT and solve every RHS column. This is a loop-result contract; SPD-to-success, rounding and backend execution are not assumed proved. |
| `Covariance`: `rightSolve_equation`, `rightSolve_unique`, `rightSolve_transposed_equation` | Unique right solve and transposition for SPD systems; C++ does not form an inverse. |
| `QR`: `givens_scaled`, `givens_orthogonal`, `orthogonal_product`, `qr_row_invariant`, `pivot_invariant`, `permutation_solution` | Scaled coefficients, embedded row rotations and column permutations preserve QR/RHS relations. Inputs to a nontrivial rotation have positive scale and a nonzero pair. |
| `QR`: `triangular_leastSquares_normal`, `qr_normal_equation`, `leastSquares_minimum` | Solved leading rows plus zero trailing system rows imply the normal equations and a global least-squares minimum. |
| `QR`: `whiten_from_forward_substitution`, `trailing_rows_annihilate`, `trailing_rows_orthonormal`, `nullspace_eliminates`, `nullspace_noise` | Successful triangular whitening and the trailing orthonormal rows eliminate feature error while preserving unit noise covariance. |

QR is column-pivoted **Givens**, as implemented locally. These statements prove
exact transformations and their loop invariants/results, not the C++ control flow
or an exact-rank guarantee from the tolerance test. A threshold stop retains the
remaining matrix; only the full-column-rank consumers use the zero-trailing-row
premise. Numerical rank, conditioning, overflow and non-finite/failure transactions
require separate source/numerical evidence. Covariance PSD is a mathematical
premise, not inferred from diagonal or finiteness checks. Generic indexing,
Cholesky and covariance lemmas adapt the same owner-authored foundation as group 1;
active extents and pivoted Givens/consumer proofs are local additions.

## Group 3: Joint state and clones

Requirements: M-STATE and M-CLONE. Sources: `msckf/state.hpp` and `clones.hpp`.
`Clones.lean` binds D=15+6N, the IMU order (p,v,theta,ba,bg), and clone (p,theta).

| Statements | Exact claim and premises |
|---|---|
| `active_dimension_bound`, `cloneCoordinate_injective`, `poseIndex_entries` | Admitted clone capacity bounds the active dimension; clone coordinates do not overlap; copied IMU coordinates are exactly 0,1,2,6,7,8. |
| `augmentIndex_old`, `augmentIndex_new`, `augment_old_block`, `augment_duplicate_row` | Augmentation retains the entire old covariance and duplicates pose rows/columns, including cross-covariance; it adds no independent noise. |
| `removeIndex_injective`, `removeIndex_preserves_imu`, `removeIndex_skips_block`, `selection_covariance` | Removal selects retained coordinates on both axes; it is not a Schur complement. |
| `augment_psd`, `remove_psd`, `augment_remove_last` | PSD preservation, including singular priors/duplicated variables, and append/remove-last covariance round trip. |
| `canonicalUpper_of_symmetric`, `cloneSnapshot_fields` | Upper-triangle canonicalization is identity on exactly symmetric input; current and first-estimate snapshots are copied separately. |

Exact PSD/symmetry are caller premises. Tolerance-admitted asymmetry, validation
and rollback, machine-size arithmetic, identity/time admission and pending
observation ownership are not established by these mathematical layout proofs.
Snapshot field equations describe immutable values, not C++ object lifetime.

## Group 4: IMU inputs, mean and differential kernels

Requirement: M-PROPAGATE. Sources: `msckf/imu.hpp`, `prediction.hpp`.
The exact mean uses held **body** force/rate, with world-fixed gravity. It does
not substitute a frozen world-acceleration model for nonzero body rotation.

| Statements | Exact claim and premises |
|---|---|
| `ImuPropagation`: interpolation endpoints/range, `endpoint_average_affine`, `average_bias` | Bracket interpolation is convex; endpoint averaging is the midpoint value of the interpolant; current bias subtraction has the stated sign. |
| `A2_weighted`, `A1_derivative`, `A2_derivative`, `velocity_derivative`, `position_derivative` | Continuous rotation integrals give p-dot=v and v-dot=R(t)a+g, with initial conditions from `mean_zero_time`; group 1 supplies the quaternion ODE. |
| `A1_zero_rate`, `A2_zero_rate`, zero-rate mean, `constantRate_add`, `velocity_split`, `position_split` | Constant-acceleration limit and arbitrary-rate exact mean composition for identical held inputs, using the original velocity for position. |
| `gammaTerm`, `gammaSum_zero`, `gamma_zero_matches_integrals` | Exact interpretation of the finite recurrence and agreement at zero angular rate; no equality to the infinite series away from zero is claimed. |
| `PropagationDifferential`: `rotatedVector_derivative`, `gyroBias_kernel_derivative`, `rightAttitude_action_derivative`, `accelerometerBias_derivative` | Actual directional derivatives of rotated force fix the attitude/accelerometer-bias negative signs and gyro-bias positive B1/B2 integrands. Group 1's `conjugatedIncrement_hasDerivAt` supplies E-transpose attitude transport. |
| `quadrature_gyroBias_derivative` | The finite weighted derivative sum is the derivative of the corresponding quadrature mean; this is not the derivative of the production truncated-Gamma mean. |
| `integrated_gyroBias_derivative` | The continuous weighted integral derivative under explicit measurability, integrability and local integrable-Lipschitz domination hypotheses. These regularity hypotheses are visible in the theorem, not assumed derivative conclusions. |
| `hat_rotated_mul_unit` | The frame identity relating ordinary `-R*hat(A*a)` to `-hat(R*A*a)*R` for a unit attitude. |

The continuous-integral theorem is conditional on its analytic regularity
hypotheses; it does not discharge a concrete uniform domination/error bound for
the admitted implementation profile. The executable Gamma truncation and GL5
quadrature are distinct approximations. Their discrepancy, concrete profile
bounds, floating-point evaluation and
a Fréchet derivative of the assembled executable transition are not claimed.
The ordinary transition's exact differential kernels and the FEJ frame/block
comparison are separate claims; group 7 owns the latter. Bias/clone immutability
and atomic publication still require C++ evidence.

## Remaining sequence

Groups 5–7 are listed in the [work list](../roadmap.md#exact-lean-proof-sequence)
and will receive their own theorem/premise mappings as implemented. Groups 8–11
remain backlog. No completion percentage is inferred from declaration counts.
