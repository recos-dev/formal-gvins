# R-RING-BUFFER: Fixed-capacity retained sequence

**ID:** `R-RING-BUFFER`

**Status:** Draft

**Applies to:** Portable single-owner storage with a positive compile-time capacity.

**Uses:** [C-CPP-SUBSET](../common/cpp-subset.md),
[V-ACQUISITION](../vision/acquisition.md) when storing sensor measurements.

## Basis

Adapt the owner's [formal-eskf RingBuffer at revision 2211ffb](https://github.com/recos-dev/formal-eskf/blob/2211ffb3a74ac9dca62ed3933bc75a2b186b3f71/include/formal_eskf/runtime/ring_buffer.hpp).
Retain fixed backing storage and circular indexing. For GVINS, reject full pushes,
expose logical rather than physical indices, and separate reading from retirement.
These changes let IMU consumers retain complete intervals and interpolation
endpoints. PX4's timestamp-selection API and default age window are not imported.

The selected capacity is the template capacity; no separate runtime length or
allocation/configuration API is needed. Samples have no required timestamp field.
In particular, existing `msckf::ImuSample::time_ns` is copied without conversion.
This is storage, not an IMU history owner or a cross-thread queue.

## Required behavior

`runtime::RingBuffer<Sample, Capacity>` shall reserve all samples inside the object.
`Capacity` shall be positive. Construction shall produce an empty, usable buffer;
copying and moving the owner shall be disabled. The caller shall supply a value
type whose default construction and copy assignment are bounded, non-blocking,
allocation-free and non-throwing. Samples shall not depend on external mutable
storage without a separately established lifetime/ownership contract.

Let S be the retained sequence, oldest first, and C the capacity.

| Operation | Required result and state effect |
|---|---|
| `push(sample)` | If `length(S) < C`, return true and append one copy. Otherwise return false and preserve all storage and metadata. Never overwrite pending entries. A sample borrowed from a retained slot is permitted. |
| `operator[](i)` | Under caller premise `i < entries()`, return a const reference to S[i], without consumption. The index shall be logical oldest-to-newest regardless of physical wrap. |
| `discard(n)` | If `n <= length(S)`, return true and retain `drop(n,S)`. Otherwise return false with all storage and metadata unchanged. `discard(0)` shall be a no-op. Discard changes metadata only; it does not clear retired payloads. |
| `entries()`, `capacity()`, `empty()`, `full()` | Report sequence length, C, length zero and length C, respectively, without mutation. |
| `reset()` | Clear every backing slot to `Sample{}`, restore the initial indices and empty the sequence. Capacity is unchanged. |

Count and physical indices shall remain bounded by capacity; ring-index arithmetic
shall not depend on unsigned wraparound. Push, read, discard and queries shall
take constant work for the selected sample type. Construction/reset shall touch
at most C samples. No operation shall allocate, lock, log or consult a clock.
Object placement and target stack/memory limits remain caller responsibilities.

References and logical indices shall not be retained across a successful push,
positive discard or reset. Rejected operations and zero discard preserve views.
All access shall be serialized by one owner; concurrent reads with mutation are
outside the contract. Const access does not provide synchronization.

Sensor owners shall admit timestamps, calibration and source identity; storage
does not sort or reject duplicate/reversed timestamps. The IMU owner shall retain
the left interpolation endpoint and all samples still needed by any consumer,
including output repropagation, before discarding a prefix. Full push failure
shall be handled explicitly. This component neither chooses a loss policy nor
guarantees interval coverage, recovery, deadlines or safe publication.

## Acceptance and verification

- Compare mixed push/discard/reset sequences against an independent linear
  sequence at capacities 1, 3, 8 and 257, including repeated wraps and full drains.
  Check every retained value and operation result after each step.
- Check full-push and invalid-discard preservation, zero discard, borrowed inputs,
  read-only logical access, non-copyable ownership, and counts near `SIZE_MAX`.
- Store exact nanosecond epochs through `UINT64_MAX`; demonstrate that time policy
  is external. Count payload assignments to check bounded copies and full reset.
- Exercise the existing MSCKF interpolation/interval functions with retained IMU
  samples: repeated reads, wrap, explicit prefix retirement, missing coverage and
  bias changes must not corrupt the raw measurements.

Implementation and native tests are mapped in the
[requirements inventory](../../../analysis/requirements-inventory.md#runtime-storage).
These tests and source inspection do not import the original buffer's Lean/ESBMC
claims: its overwrite, physical-index and consuming-selection semantics differ.
Local formal proofs, actual history-owner integration, concurrency and target
timing/resource qualification remain separate evidence.
