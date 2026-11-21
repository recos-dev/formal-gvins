# H-PUBLISH: Timestamp-paced local sensor publication

**ID:** `H-PUBLISH`

**Status:** Draft

**Applies to:** Host-only 1x replay of recorded sensor arrivals to a local caller.

**Uses:** [H-RECORDING](sensor-recording.md),
[C-CPP-SUBSET](../common/cpp-subset.md).

## Basis

Replay the availability order seen by the recorder, rather than publishing GNSS
auxiliary data at its possibly earlier reference time. This provides a local
input source; it does not specify estimator output frequency or a flight transport.

## Required behavior

After source/queue preparation, `start()` shall capture a monotonic epoch t0 for
this replay session. For record time r and the first record time r0, its deadline
shall be t0 + (r - r0), at 1x speed. Original message timestamps and recording times
shall remain unchanged. Integer subtraction shall precede clock conversion;
invalid order, unrepresentable intervals and deadline overflow shall fail.
Deadline arithmetic shall be testable independently of OS waits. The current
Linux host binding requires a nanosecond `steady_clock` duration; another clock
representation requires a separately checked conversion.

Publication shall not precede the deadline. Same-time records shall retain their
sequence order. Preparation, parsing and consumer execution shall not shift the
epoch or accumulate relative sleep offsets. Each delivery shall report its
scheduled offset, observed elapsed time and lateness. In report-only mode, overdue
events shall be delivered in order without implicit dropping or retiming. A
positive caller-selected lateness budget shall instead stop with an error before
publishing an event outside that budget. No default deployment deadline is implied.

One loader shall exclusively read the borrowed source, and one caller shall own
`start()`/`next()`. A bounded queue shall apply backpressure to loading when full,
without overwriting or losing records. File I/O and parsing shall execute outside
the handoff lock. Startup shall prefill that queue or reach EOF before taking t0.
The source shall outlive replay and shall not be accessed concurrently by others.

`stop()` shall wake startup/publication/queue waits and may be called from another
thread. It does not interrupt an operating-system read already in progress.
Destruction shall stop and join the worker; no detached worker shall outlive its
owner. Restart shall require a new source and replay object. EOF/stop return false;
source/schedule failures throw and are terminal. No failure/EOF/stop shall replace
the caller's last successful publication. Source failure aborts even queued
records; a simultaneous recorded failure takes precedence over stop/EOF.

This host component may allocate and block. It shall not claim hard real-time
scheduling, target WCET, calibrated clocks, sensor synchronization or estimator
readiness. It shall not require ROS or MAVLink.

## Acceptance and verification

- Compare schedule offsets to exact recorded differences, including equal times,
  large epochs and unrepresentable deadlines; verify no early delivery and no
  mutation of native timestamps.
- Exercise queue capacities one, two and the default, wrap, EOF, invalid order,
  injected source errors, stop during a deadline wait, repeated stop, restart
  rejection and startup preloading. Check preservation of the caller's output.
- Induce consumer delay and verify explicit lateness-budget failure, with no
  silently skipped/re-timestamped event. Run thread/race checks where supported.
- Run timestamp-paced actual-data replay; record source identity, platform,
  queue setting, event counts and lateness distribution. Observed maxima are not
  a worst-case bound or evidence for another target.
