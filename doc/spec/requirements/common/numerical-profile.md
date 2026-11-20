# C-NUMERICAL-PROFILE: Numerical execution profile

**ID:** `C-NUMERICAL-PROFILE`

**Status:** Draft

**Applies to:** Shared scalar, linalg and SO(3) operations and their callers.

## Basis

Adapted from the formal-eskf numerical policy identified in the
[specification index](../../README.md#reuse-and-evidence). Application-specific
ESKF options and prior verification results are not imported.

## Required behavior

Exact mathematical specifications shall use real arithmetic. An implementation
or verification configuration shall identify binary32 or binary64 separately;
a real-arithmetic proof shall not be presented as a floating-point proof.

Each numerical execution profile shall identify:

- Scalar format, rounding mode and subnormal/flush-to-zero behavior.
- Compiler, target, build flags and FMA contraction behavior.
- Linalg backend, elementary math library and their versions.
- Algorithm selections affecting numerical evaluation.

The default profile shall use round-to-nearest, ties-to-even, disable fast-math
transformations and disable FMA contraction. A different profile requires
separate evidence for affected claims. Subnormal behavior shall be recorded,
not inferred from the scalar type or a successful host test. Evidence relying
on gradual underflow shall not be reused for flush-to-zero without review.

Non-finite values shall not be treated as valid consumed estimator inputs.
The division between caller premises and executable checks is defined by
[C-INPUT-DOMAIN](input-domain.md); checked failures follow
[C-FAILURE](failure.md).

## Acceptance and verification

Inspect effective compiler flags and target floating-point settings. Record the
profile with each result and review changes to compilers, backends or arithmetic
selections before reusing evidence. An interface contract or host result does
not establish behavior of a different backend, math library or target.
