# V-CONSUMPTION: Once-only observation consumption

**ID:** `V-CONSUMPTION`

**Status:** Draft

**Applies to:** Local estimator measurement incorporation and retry/retirement.

**Parent:** [V-TRACK](feature-tracks.md).

**Uses:** [V-OBSERVATION](observation-admission.md), [V-ASSOCIATION](track-association.md).

## Basis

A feature constraint is built from several observations. They are not independent
new data when retried, regrouped, relinearized or retained for mapping.

## Required behavior

A successful local correction shall mark every observation used to construct its
accepted constraints as consumed by that estimator. The joint estimate, covariance,
applicable linearization caches and consumption records shall commit together.
A numerical failure shall preserve the transaction's inputs and shall not mark
observations fused. Earlier successful propagation may be a separate transaction.

A consumed observation shall not enter another update as independent evidence.
Retry shall use unchanged unconsumed records or an explicitly rebuilt candidate;
it shall not partially consume a failed update. Rejection/retirement shall be
distinguishable from successful incorporation. Observation splitting, ID changes
or clone removal shall not erase the information needed to prevent double fusion.

Retention for map matching or another representation is permitted, but feedback
to the same filter requires a separate correlation/update contract. No independent
map pose measurement is authorized by retention alone.

## Acceptance and verification

Exercise success, failure at each candidate stage, retry, duplicate delivery,
partially overlapping tracks and retirement after success/failure. Check state,
covariance/cache and ledger publication as one transaction. This is a software
ownership property; it does not prove statistical independence of distinct pixels.
