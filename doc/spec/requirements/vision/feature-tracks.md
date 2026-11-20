# V-TRACK: Feature observation identity and ownership

**ID:** `V-TRACK`

**Status:** Draft

**Applies to:** Frontend observations, local MSCKF track storage and transfer to a future map owner.

**Uses:** [V-FRAMES](../common/frames.md).

## Basis

MSCKF constraints depend on observations of the same feature at identified poses.
This requirement owns the composition of identity, pose association and consumption
used by the [MSCKF derivation](../../architecture/msckf-formulation.md).

## Required behavior

The local observation path shall preserve a record's identity, time/calibration
binding, pose association and consumption state from admission through retirement.
Successful frontend tracking shall not imply that a record has a valid clone or
has been fused. Every consumer shall retain a valid lifetime or independent copy.
Feature histories and temporary triangulated positions shall remain outside the
MSCKF joint state/covariance; a future map is a separate owner.

### Observation identity

[V-OBSERVATION](observation-admission.md) owns identity, stable continuation,
duplicates/conflicts and admission. Rejection shall preserve existing records.

### Admission and clone association

[V-ASSOCIATION](track-association.md) owns live-clone/time binding, delayed data,
capacity and retirement. An admitted observation is not usable until that binding
is established; slot reuse shall not change its meaning.

### Consumption and lifetime

[V-CONSUMPTION](observation-consumption.md) owns once-only incorporation and atomic
estimate/ledger publication. Retaining data for mapping shall not authorize
independent reuse in the same estimator.

## Decomposition

| Child | Allocated obligation |
|---|---|
| [V-OBSERVATION](observation-admission.md) | Observation identity and admission. |
| [V-ASSOCIATION](track-association.md) | History-to-clone binding, capacity and cross-owner lifetime. |
| [V-CONSUMPTION](observation-consumption.md) | Successful consumption, failure/retry and prevention of double fusion. |

## Acceptance and verification

Require child evidence plus sequences crossing all three boundaries: admission,
constraint construction, successful or failed update, retry and retirement.
Include duplicates, calibration/session changes, stereo identities, slot reuse
and capacity exhaustion. Frontend-only tests cannot accept this composition.
Source checks establish software identity/lifetime/transaction properties, not
physical feature identity, timestamp calibration or noise independence.

## Open decisions

Runtime/acquisition profiles must bind capacities, timing, splitting/retirement,
noise and concurrency. Map persistence, relocalization and correlated feedback
remain separate requirements; this decomposition does not implement them.
