# V-EXECUTION: Host frontend execution and ownership

**ID:** `V-EXECUTION`

**Status:** Draft

**Applies to:** Blocking CPU/CUDA image frontend outside the numerical core.

**Uses:** [C-CPP-SUBSET](../common/cpp-subset.md), [V-ACQUISITION](acquisition.md), [V-OBSERVATION](observation-admission.md).

## Basis

OpenCV/device operations and their allocation are isolated in the host frontend.
The selected [development profile](../../profiles/orb-frontend.md) identifies
algorithm settings separately from execution choice.

## Required behavior

### Tracker interface

FeatureTracker denotes a shared operation contract, not a required base class or
forwarding wrapper. A concrete host tracker shall support
`process(identity, image, mask, output)` returning `VisionStatus`, and `reset()`.
The host image/mask inputs use `cv::Mat`; the image identity uses `ImageIdentity`.
Method-specific options belong to construction/configuration of the concrete
tracker, not to a common ORB-shaped options structure. A template caller may bind
a concrete tracker through these named operations without virtual dispatch.

The owning `TrackedImage` result shall contain image bindings, an explicit pixel
domain, `FeatureObservation` records and per-camera lifecycle events under V-OBSERVATION. It shall not require
keypoints, descriptors, descriptor distances or geometry-estimator containers.
The common result header shall compile without OpenCV or a particular tracking
method. This host container may allocate; numerical consumers shall use its
records under their own bounded storage/admission contract.

A tracker may provide an additional method-specific diagnostic output. Omitting
diagnostics shall preserve the same tracking decisions under the same execution
and random inputs. When requested, diagnostics shall be published with the
corresponding observation batch, remain unchanged on a returned failure, and
not expose mutable aliases of retained tracking history. Diagnostics are not
required inputs to the estimator; descriptor-free trackers need none.

### Execution and failure

Each frontend instance shall have one caller/owner at a time; concurrent use of
one instance is outside its contract. CPU/CUDA selection shall be explicit.
Unavailable CUDA execution shall report `backend_unavailable`, not silently run
on CPU. Invalid inputs/configuration shall report `invalid_input`; contained
OpenCV failures shall report `backend_failure`. A returned failure shall preserve
caller-visible output and existing temporal history.
Extractor initialization/configuration failure takes precedence over its operation
input checks. The ORB tracker checks its own options and stream identity before
invoking extraction; backend availability failure therefore follows those tracker
checks. This ordering does not require evaluating image content on an unavailable
extractor.

CUDA work producing a returned result shall complete before successful return.
Returned descriptors shall own host storage, and retained history shall not alias
mutable caller output. Backend containers/device buffers shall not enter numerical
estimator state or equations. A successful featureless image shall publish an
empty set and end every previous temporal continuation. A missing/invalid image
shall fail without generating retirement events. Each successful frame shall
explicitly identify started, continued and ended IDs for that camera. Reset shall retire temporal
history without reusing track IDs. Changes of camera/session/calibration/noise
binding or image geometry require an explicit reset of this tracker profile.
Reset itself retires all current tracks; it does not modify an already returned
snapshot or reuse counters. Camera parameters remain fixed for the lifetime of
the object. A different numerical calibration requires a new configured object
and a fresh acquisition session; changing an integer calibration ID alone does
not change the camera model. IDs are unique within the owning tracker/session,
not across independently constructed trackers.

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
Compile the common result without image-library headers; exercise a caller that
uses only the shared tracker operations and output. Compare tracking with and
without diagnostics under controlled execution, including failed/empty frames and
reset. Check both outputs' failure atomicity and observation/diagnostic pairing.

## Open decisions

Deployment allocation-failure recovery, overload behavior and timing/memory limits
are not selected. The current blocking interface does not authorize an unreviewed
asynchronous pipeline or automatic backend switching.

On GNU/Clang ELF hosts, `tracker_failure` wraps the pinned OpenCV essential-fit
symbol in that test executable only. It forces a backend exception after initial
history exists and compares a successful retry against an untouched control
tracker. Production code has no fault hook. This checks exception handling and
transaction boundaries, not physical GPU/driver fault recovery.
