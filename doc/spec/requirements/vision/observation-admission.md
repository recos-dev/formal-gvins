# V-OBSERVATION: Observation identity and admission

**ID:** `V-OBSERVATION`

**Status:** Draft

**Applies to:** Frontend publication and local observation-store admission.

**Parent:** [V-TRACK](feature-tracks.md).

**Uses:** [V-FRAMES](../common/frames.md), [V-ACQUISITION](acquisition.md).

## Basis

This allocates V-TRACK's observation identity/admission obligations independently
of the detector, container or estimator update.

## Required behavior

Each admitted observation shall bind acquisition session, camera, image, track,
measurement time, coordinates and calibration/noise model. Image identity shall
not rely on timestamp alone. The tuple `(session, camera, image, track)` shall
identify at most one record. Binding validity belongs to the selected profile;
an integer identifier alone does not establish the calibration/noise model.

Accepted continuations shall retain track identity. Storage slots, descriptor
values and array indices shall not serve as persistent IDs. IDs shall not be
reused while any consumer can still reference them; an explicit session/generation
may distinguish reused counters. Cross-camera ID sharing shall require an explicit
association, not matching arrival times or accidental counter equality.

| Admission condition | Required response |
|---|---|
| Valid identity, finite coordinates and compatible binding | Admit at most one record; estimator use still requires a live pose association. |
| Identical existing record | Ignore or reject the duplicate without appending/fusing it again. |
| Existing identity with conflicting data | Report conflict and preserve the stored record. |
| Invalid identity, coordinates or binding | Reject without modifying existing records. |

Publication and storage shall preserve coordinate/descriptor pairing and ownership.
Changing calibration shall not reinterpret retained records silently. Frontend
frame rejection can prevent duplicates at that interface, but does not replace
the observation store's conflict checks for records received from other callers.

### Tracker publication

Each successful tracker result shall publish the complete active observation set
for one identified camera image. Relative to the preceding successful result,
retained IDs denote continuations, new IDs denote newly admitted tracks, and
absent IDs retire that camera's temporal continuation. A failed frame shall not
retire tracks. An empty successful frame retires all prior continuations; reset
does likewise without recycling IDs. Callers shall propagate reset to their
history owner and shall not infer retirement from a frame they did not receive.
This does not delete stored observations or authorize their reuse in the filter.

The batch shall explicitly identify raw or undistorted pixel coordinates under
the calibration binding. An unspecified domain is not an admitted measurement.
Consumers shall not infer the domain from the tracker method or change it merely
by relabeling coordinates. The current ORB path emits undistorted pixels; the
raw-image migration remains a separate implementation change. Derived normalized
bearings are not additional pixel observations.

## Acceptance and verification

Exercise duplicate/conflicting records, session/generation rollover, cross-camera
IDs, invalid bindings and caller-buffer reuse. Check identity continuity and
admission atomicity separately from correctness of physical feature association.
Check complete-set lifecycle across success, failure, empty frames and reset, and
verify that coordinate-domain metadata agrees with the actual processing path.
The current ImageIdentity/FeatureObservation and tracker implement only the
frontend portion; store-level evidence is required before accepting the whole
contract.

## Host lifecycle publication

Each successful image batch shall include one `started` or `continued` event
for each current observation and one `ended` event for each previous camera-local
ID absent from the new active set. An empty successful set ends all previous
tracks. A failed call publishes no changes. Reset is an explicit retirement of
the owner's entire active set; the next frame starts new IDs. Events do not
authorize reuse of old measurements or merging independent camera identities.
