# Development work list

This list tracks planned work. It does not approve requirements or claim completed
verification. Existing frontend decisions are recorded in the
[ORB reference comparison](analysis/orb-frontend-settings.md).

## B-01: Per-module unit benchmarks

**Status:** In progress; C++ ORB execution on a fixed EuRoC window, known-translation
scoring, per-frame timings/reports and unit/integration tests are implemented for
CPU and CUDA, with local RTX 3060 execution checked.
Per-stage profiling, qualified CPU/CUDA comparison and reviewed baselines remain
pending. See the [runnable benchmark](guides/frontend.md#fixed-dataset-orb-benchmark).

Build independently runnable benchmarks for each implemented module, reporting
execution time together with an appropriate correctness or accuracy metric.
Extend the inventory as MSCKF modules become available.

The [feature benchmark proposal](analysis/feature-benchmarks.md) defines a shared
runner, independent scorer and reviewed baselines for multiple frontend methods.
It separates same-method regression from cross-method quality/cost comparison;
adapters for new methods reuse the protocol rather than adding separate scorers.

| Module | Accuracy / correctness metrics |
|---|---|
| Linalg and SO(3) operations | Reference error, solve residuals, orthogonality and rotation error; identify precision and active dimensions. |
| Image preprocessing and ORB extraction | Coordinate RMSE and repeatability with known image transformations/correspondences; report feature count and spatial coverage. |
| Descriptor matching, geometric rejection and track management | Match precision/recall with labelled correspondences, reprojection error where defined, track survival and identity violations. Descriptor distances and inlier counts are not ground-truth accuracy. |
| Future MSCKF propagation and update modules | Position/velocity RMSE and geodesic attitude error against an identified reference. Full-trajectory ATE/RPE belongs to the additional integration benchmark. |

Completion criteria:

- Keep an inventory of module, callable entry point, input workload, reference,
  metric/units and runnable benchmark command. State where a metric does not apply
  or ground truth is unavailable; do not assign RMSE to every module mechanically.
- Report sample count, warmup policy, median, P95 and maximum observed execution
  time, plus failures/rejections. Observed maximum latency is not a WCET proof.
- Compare CPU/GPU under matched algorithm/preprocessing settings. Separate kernel
  time, transfer/synchronization time and complete operation latency; also retain
  an end-to-end pipeline benchmark to expose overhead between modules.
- Bind results to source identity, configuration, compiler/build options,
  hardware/threading, dataset or fixture identity and reference alignment.
  Define RMSE's quantity, units, included samples and aggregation explicitly.
- Export machine-readable results and a concise comparison table, enabling later
  regression baselines and links to relevant requirements. Keep run artifacts in
  ignored output directories; review acceptance thresholds separately.

Descriptor-smoothing settings are aligned; controlled timing baselines and Orin
measurements remain pending. Add estimator accuracy benchmarks when the
corresponding estimator modules exist.
