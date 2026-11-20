# M-LIFECYCLE: Local estimator execution and transactions

**ID:** `M-LIFECYCLE`

**Status:** Draft

**Applies to:** Local MSCKF scheduling, capacities and observation processing.

**Parent:** [SW-MSCKF](../software/local-vio.md).

**Uses:** [V-TRACK](../vision/feature-tracks.md), [V-ACQUISITION](../vision/acquisition.md), [M-STATE](state.md), [M-PROPAGATE](propagation.md), [M-CLONE](clones.md), [M-CONSTRAINT](constraints.md), [M-CORRECT](correction.md).

## Basis

[Clone removal and execution order](../../architecture/msckf-formulation.md#clone-removal-and-execution-order)
defines the composed local image update. Core computation remains bounded and
independent of host scheduling/I/O.

## Required behavior

The image update shall follow this dependency order:

1. Validate identity/time/binding; propagate to the modeled exposure time using
   qualified IMU intervals and establish the current first estimate.
2. Augment the IMU-pose clone with matching first estimate, and admit observations.
3. Select eligible tracks, including those affected by impending clone retirement.
4. Triangulate, validate geometry, assemble/whiten, eliminate feature error, gate
   and optionally compress accepted constraints.
5. Correct/inject the complete joint estimate, apply identity reset and retain
   frozen first estimates.
6. Commit successful consumption consistently, then retire records/remove clones
   under the declared policy.

Propagation and augmentation may be earlier successful transactions. A failed
correction shall preserve its own complete pre-correction state/covariance/cache
and observation ledger; it shall not retroactively label earlier operations failed
or observations fused. If no constraints are accepted, skip correction/reset and
apply only the declared retirement/removal policy. Distinguish invalid input,
not-ready/no-update, rejected observations, numerical failure and successful fusion
in caller-visible outcomes.

Runtime profiles shall declare maximum clones/tracks/observations, workspace,
admitted delays/gaps, overflow/retirement, ownership and recovery. Capacity loss
shall not overwrite pending data silently. Timestamp reversal or absent IMU
coverage shall not attach observations to a convenient current pose. Loss of
visual tracking shall not fabricate measurements or silently reset covariance;
propagation-only validity and reinitialization are explicit policies.

## Acceptance and verification

Exercise complete sequences with time splits, delayed/duplicate images, missing
IMU, full buffers, clone retirement and failures/retries at each stage. Check
once-only consumption and consistent source/epoch identities in outputs. Measure
end-to-end resources under the selected profile; algorithmic bounds do not establish
host deadline compliance. Specific capacity, delay and recovery choices remain
unselected and prevent accepting an operational runtime profile.
