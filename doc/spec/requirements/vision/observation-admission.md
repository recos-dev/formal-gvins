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

## Acceptance and verification

Exercise duplicate/conflicting records, session/generation rollover, cross-camera
IDs, invalid bindings and caller-buffer reuse. Check identity continuity and
admission atomicity separately from correctness of physical feature association.
The current ImageIdentity/FeatureObservation and tracker implement only the
frontend portion; store-level evidence is required before accepting the whole
contract.
