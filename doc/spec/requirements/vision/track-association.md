# V-ASSOCIATION: Track, clone and lifetime association

**ID:** `V-ASSOCIATION`

**Status:** Draft

**Applies to:** Local track histories, live pose clones and transfers to another owner.

**Parent:** [V-TRACK](feature-tracks.md).

**Uses:** [V-OBSERVATION](observation-admission.md), [V-ACQUISITION](acquisition.md).

## Basis

MSCKF constraints use a feature's observations at identified historical poses;
the immediately previous image held by a temporal tracker is not that history.

## Required behavior

Before constructing a visual constraint, each observation shall resolve to a live
clone at its modeled measurement time after the declared clock conversion.
A missing/retired clone shall not be replaced by the newest clone or a reused
storage slot. Shared stereo clones require a reviewed exposure synchronization
model; simultaneous arrival is insufficient. A delayed image shall follow an
explicit buffering/rejection policy, preserving the associations of retained data.

Track/observation/clone counts shall stay within declared capacities. Capacity
exhaustion shall not silently overwrite data required by pending constraints.
Track splitting and retirement shall preserve surviving identities, time bindings
and consumption records. Explicitly discarded data shall not be reported as fused.

Feature histories and temporary triangulated points shall remain outside the joint
filter state/covariance. A map receiving data shall own a copy or have a guaranteed
retention lifetime independent of clone retirement. Removal/reset shall leave no
dangling references or silently reassigned observations in another owner.

## Acceptance and verification

Check delayed images, missing clones, slot reuse, interleaved cameras, full
capacity and retirement with pending consumers. Verify association/lifetime
invariants over sequences, not only single calls. Test simultaneous stereo records
without assuming the first development profile implements stereo tracking.

## Open decisions

The runtime profile must bind capacities, retirement/splitting order, clock
tolerances, delayed-image handling and concurrency ownership before integration
acceptance. No track-history or clone implementation is currently supplied by
the temporal frontend.
