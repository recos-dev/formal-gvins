# V-EXECUTION: Host frontend execution and ownership

**ID:** `V-EXECUTION`

**Status:** Draft

**Applies to:** Blocking CPU/CUDA image frontend outside the numerical core.

**Uses:** [C-CPP-SUBSET](../common/cpp-subset.md), [V-ACQUISITION](acquisition.md).

## Basis

OpenCV/device operations and their allocation are isolated in the host frontend.
The selected [development profile](../../profiles/orb-frontend.md) identifies
algorithm settings separately from execution choice.

## Required behavior

Each frontend instance shall have one caller/owner at a time; concurrent use of
one instance is outside its contract. CPU/CUDA selection shall be explicit.
Unavailable CUDA execution shall report `backend_unavailable`, not silently run
on CPU. Invalid inputs/configuration shall report `invalid_input`; contained
OpenCV failures shall report `backend_failure`. A returned failure shall preserve
caller-visible output and existing temporal history.
Initialization/configuration failure takes precedence over operation input checks;
an unavailable instance need not inspect an image to report unavailability.

CUDA work producing a returned result shall complete before successful return.
Returned descriptors shall own host storage, and retained history shall not alias
mutable caller output. Backend containers/device buffers shall not enter numerical
estimator state or equations. A successful featureless image shall publish an
empty set and retire previous temporal continuations. Reset shall retire temporal
history without reusing track IDs. Changes of camera/session/calibration/noise
binding or image geometry require an explicit reset of this tracker profile.

Host allocation and exceptions are permitted here. Standard allocation exceptions
are not promised a VisionStatus translation; they shall not be mistaken for a
successful frame. The deployment adapter shall define recovery/discard behavior
before use in a control pipeline. This permission does not extend to the core.
Resource acceptance shall include input-image bounds, OpenCV/thread settings,
queue/overload policy and end-to-end deadlines. Feature caps alone do not bound
library allocation or worst-case latency.

## Acceptance and verification

Check CPU/CUDA availability behavior, completion/ownership, invalid input and
reset/empty-frame sequences. Test caller mutation of published descriptors and
device failures without fallback. Measure transfer, synchronization and total
latency/memory on the named target; match algorithm settings for acceleration
claims. Native GPU tests are not a proof of driver internals or real-time bounds.

## Open decisions

Deployment allocation-failure recovery, overload behavior and timing/memory limits
are not selected. The current blocking interface does not authorize an unreviewed
asynchronous pipeline or automatic backend switching.
