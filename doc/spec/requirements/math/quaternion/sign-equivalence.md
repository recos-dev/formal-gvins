# Q-SIGN: Quaternion sign equivalence

**ID:** `Q-SIGN`

**Status:** Draft

**Applies to:** Exact quaternion/SO(3) behavior; application frames are defined by V-FRAMES.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](multiplication.md).

## Required behavior

Unit quaternions form a double cover of `SO(3)`:

~~~text
q ≡ -q
R(q) = R(-q)
rotate(q, v) = rotate(-q, v)
~~~

Coefficient equality and tolerance-based rotation comparison are different behaviors:

~~~text
same_coefficients(q1, q2)
same_rotation(q1, q2, tolerance)
~~~

`same_coefficients` shall compare all four coefficients with scalar equality;
positive and negative zero compare equal. It shall not normalize or select a sign.

For finite coefficients and a finite nonnegative tolerance, `same_rotation` shall
return true when either `max_abs(q1-q2) <= tolerance` or
`max_abs(q1+q2) <= tolerance`. Tolerance is a dimensionless maximum coefficient
residual, not an angular distance. A negative or non-finite tolerance shall return
false. The caller establishes finite coefficients and the unit premise needed to
interpret this as a rotation comparison; the comparison does not validate norm,
normalize operands or provide a status channel. Numerical acceptance of the
underlying quaternions remains separate from the comparison result.

No operation implicitly forces `q0 >= 0`. Sign selection is performed only when a unique local representation is required, such as the principal `Log_q`.

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.
Test both signs, exact versus approximate coefficient comparisons, threshold
equality and negative/non-finite tolerances independently.
