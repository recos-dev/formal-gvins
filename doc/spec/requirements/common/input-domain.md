# C-INPUT-DOMAIN: Input-domain and caller obligations

**ID:** `C-INPUT-DOMAIN`

**Status:** Draft

**Applies to:** Shared numerical operations and their caller boundaries.

**Uses:** [C-NUMERICAL-PROFILE](numerical-profile.md), [C-FAILURE](failure.md).

## Basis

Adapted from the formal-eskf numerical policy identified in the
[specification index](../../README.md#reuse-and-evidence). Fixed ESKF prediction
checks and sensor-specific domains are outside this shared contract.

## Required behavior

Each operation contract shall distinguish mathematical premises, caller-owned
validation and checks performed by the operation. A caller shall establish every
premise required by its claim, including consumed-value finiteness, dimensions,
index bounds, units, frames and any required geometric or matrix properties.

| Operation | Domain and responsibility |
|---|---|
| Raw scalar/matrix arithmetic and coefficient access | Caller establishes valid shapes, indices and operands; a checked caller validates required result properties before publication. Raw operators have no status channel. |
| Checked square root | Operation rejects non-finite or negative input. |
| Checked normalization | Operation validates the positive finite threshold and rejects a computed norm below it. |
| Quaternion rotation/inverse and principal Log identities | Caller establishes the unit-quaternion premise. A type name or finite coefficients alone do not establish it. |
| SPD solve | Caller supplies an SPD system and any conditioning bound needed by its claim. Operation checks finite inputs, exact numerical symmetry, factorization and finite output. |

Operations shall follow their own ordered checks and failure contracts; this
document shall not be interpreted as a requirement to repeat all checks at every
internal call. In particular, quaternion Log does not measure a unit-norm residual,
and SPD solve does not estimate a condition number.

Bounds required for a numerical claim shall be explicit in its selected profile.
An unspecified bound prevents that claim's acceptance. It does not authorize a
hidden clamp, normalization, covariance repair or physical-sensor validity claim.

## Acceptance and verification

For each caller/callee boundary, identify who establishes each premise and what
the executable checks actually reject. Exercise domain boundaries and check
failure precedence against the operation contract. Separate physical assumptions,
exact algebra and numerical bounds; status checks alone discharge none of the
unmeasured premises.
