# C-FAILURE: Numerical failure reporting and atomicity

**ID:** `C-FAILURE`

**Status:** Draft

**Applies to:** Checked numerical APIs; frontend and runtime lifecycle decisions have separate contracts.

## Basis

Adapted from the formal-eskf numerical policy identified in the
[specification index](../../README.md#reuse-and-evidence), scoped to shared
operations without assuming an implemented estimator transaction.

## Required behavior

Checked numerical operations shall report success or failure explicitly through
[formal_gvins::Status](../../../../include/formal_gvins/status.hpp). The failure
vocabulary is:

```text
non_finite_input       domain_error          out_of_range
zero_or_unsafe_divisor invalid_quaternion_norm
not_positive_definite ill_conditioned       non_finite_result
```

Each operation shall specify the categories it detects and their precedence.
The presence of an enum value shall not imply an implemented check; in particular,
SPD solve does not impose an `ill_conditioned` cutoff.

On failure, a checked operation shall preserve its complete caller-visible output,
including permitted input/output aliases. On success it shall publish the complete
result. A caller combining operations into an atomic update shall validate the
complete candidate before committing it; atomic primitives alone do not establish
an atomic composition.

Operations shall not report success by silently replacing invalid input with
identity, zero, a previous value or a clamped value. An operation whose specified
purpose includes normalization shall follow that explicit contract. Failure
handling shall not silently change the mathematical operation.

Raw operators and unchecked geometry operations have no status channel. Logging,
retry, sample retirement and estimator recovery belong to their callers, not to
the numerical primitives.

## Acceptance and verification

Check success publication, failure precedence and output preservation for each
declared alias and failure path. Use snapshots including non-finite and signed-zero
values where relevant. A success status establishes only that operation's checked
postconditions, not an unstated numerical error bound or physical validity.
