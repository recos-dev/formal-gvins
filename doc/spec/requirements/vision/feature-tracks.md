# V-TRACK: Feature observation identity and ownership

**ID:** `V-TRACK`

**Status:** Draft

**Applies to:** Frontend observations, local MSCKF track storage and transfer to a future map owner.

**Uses:** [V-FRAMES](../common/frames.md).

## Basis

MSCKF constraints depend on observations of the same feature at identified poses.
This requirement owns the data contract used by the
[MSCKF derivation](../../architecture/msckf-formulation.md), independently of the
feature detector or storage container. Long-term map ownership is separate from
the local filter's temporary feature estimates.

## Required behavior

### Observation identity

Each admitted observation shall identify its acquisition session, camera, image,
feature track, measurement time, image coordinates and calibration/noise-model
binding. Image identity shall not rely on timestamp alone. The tuple of session,
camera, image and track shall identify at most one measurement record.

A feature ID shall remain stable for accepted continuations of a track. A storage
slot, array index or descriptor value shall not serve as persistent identity.
IDs shall not be reused while any consumer can still reference the old identity;
session/generation identifiers may distinguish reused local counters. Cross-camera
observations may share a track ID only after an explicit association establishes
that relationship.

### Admission and clone association

| Condition | Required behavior |
|---|---|
| Known camera/image, finite coordinates and compatible calibration/noise binding | Admit at most one record for its identity; estimator use additionally requires a valid pose association. |
| An identical observation is received again | Do not append a second measurement or fuse it again as independent evidence. |
| The same identity carries conflicting data | Report the conflict and preserve the existing record. |
| Invalid identity, coordinates or calibration binding | Reject admission without modifying existing records. |
| Missing or retired clone | Do not construct a local visual constraint from a reused slot or a substituted current pose. |

Local estimator consumption shall resolve each observation to the live clone at
its modeled measurement time, after the declared clock conversion. Shared stereo
clones require a reviewed synchronization model; equal arrival times do not
establish equal exposure times. A delayed image shall not silently attach to the
newest clone. Buffering, rejection and timing tolerances are caller policies.

### Consumption and lifetime

Feature tracks and temporary triangulated positions shall remain outside the
MSCKF joint state/covariance. Transferring data to a sparse map shall give the map
its own ownership or an explicitly guaranteed lifetime. Retiring a clone or local
track shall not leave dangling references in another consumer.

Successful incorporation of a visual constraint shall mark all observation
records used to construct that constraint as consumed by that local estimator.
The estimate and its consumption record shall commit consistently. A failed
estimator update shall not mark those observations as successfully fused.
Explicit rejection/retirement is distinct from successful consumption.

A consumed observation shall not be incorporated again as independent evidence
in that estimator. Retention for map matching or relinearization in another
representation is not prohibited, but it does not authorize independent feedback
of the same information into the filter. Such feedback requires a separate
correlation and update contract.

Track and observation counts shall stay within declared capacities. Capacity
exhaustion shall not silently overwrite measurements or clones still required by
pending constraints. Any rejection or retirement policy shall preserve surviving
identities, pose associations and consumption records.

## Acceptance and verification

Exercise duplicates, conflicting records, session changes, ID/slot reuse,
cross-camera identities, delayed images, missing clones, full capacity and
calibration changes. Verify successful consumption, failure/retry without double
fusion, and retirement without dangling references. Test simultaneous stereo
records without assuming a stereo profile is already selected.

Source checks can establish identity/index/lifetime and transaction properties.
They do not establish physical feature identity, timestamp calibration accuracy
or independence of measurement noise. Those require separate model and system
evidence.

## Open decisions

Capacities, track splitting/retirement, delayed-image handling, clock tolerances,
noise-model values and concurrency ownership require runtime/acquisition profiles.
Map persistence, relocalization and loop-correction feedback are outside this
requirement.
