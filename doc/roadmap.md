# Development work list

This list tracks planned work. It does not approve requirements or claim completed
verification. Existing frontend decisions are recorded in the
[ORB reference comparison](analysis/orb-frontend-settings.md).

## MSCKF mathematics implementation sequence

**Status:** The formulation, reference comparison and local profile are committed
as Draft at `b568004`. The action sequence below is proposed for stepwise review
and implementation; it does not approve those requirements or authorize commits.
The current repository has numerical primitives, an ORB frontend, EuRoC input and
feature benchmarks. It does not yet implement the joint MSCKF estimator.

The current scope is formula verification and the C++ mathematical filter:
bounded state/covariance storage, propagation, cloning, camera/feature geometry,
camera residual/Jacobian evaluation and generic correction. Use Eigen/double, fixed calibration and the
selected Hamilton/FEJ formulation. Callers supply valid initial states, covered IMU
intervals and observations associated with the requested clones. Synthetic tests
exercise those numerical operations directly.

Triangulation, feature-error elimination, measurement compression, feature-history
management, initialization policy, frontend changes, replay,
event scheduling and real-time integration are deferred work, not deliverables
of this mathematical implementation sequence. The broader local VINS milestone
remains recorded below; it does not expand the current scope.

The [MSCKF formulation](spec/architecture/msckf-formulation.md) and its
[requirement allocation](spec/requirements/software/local-vio.md) own the model.
In particular, right-local errors, analytical interval-equivalent noise derived
from continuous IMU density, and full joint covariance must remain consistent;
importing formal-eskf's INS implementation does not establish that it implements
this model. These Draft requirements and
the revised conventional FEJ/identity-reset binding still require review.

### Current scope: commit-sized mathematical actions

Each numbered row is one planned implementation/review unit, normally one commit
including its focused tests and requirement mapping. Existing VINS IDs retain their
meaning; suffixes split a broad item without renumbering requirement IDs. Execute
in the displayed order. A newly discovered prerequisite is reported against its
owning row rather than silently replacing the plan.

The Solà/OpenVINS comparison is available in the design review. Start with
repeatable equation checks, not another general literature survey. Bind an action
to the affected Draft clauses before implementation; resolve its open model or
numerical choices explicitly. Unrelated profile gaps need not block independent
state/layout work. Formula verification continues with each numerical module.

| Order / action | Deliverable / requirement owner | Acceptance before review and commit |
|---|---|---|
| 1 / VINS-01.1 | Persist the formula-verification baseline and obligation inventory / SW-MSCKF, VINS-LIT. | Map equation/claim, assumptions, requirement, independent reference and intended test/proof method. Retain runnable convention, Jacobian, FEJ gauge, source-order and limiting-case checks already explored; name unsupported claims. Reference code must not call the production operation it judges. This creates reproducible checks, not a claim that all formulas are formally proved. |
| 2 / VINS-02 | Joint state, layout and workspace ownership / M-STATE. | Bind backend and peak clone capacity at compile time; retain runtime N and D=15+6N. Store IMU/clones, complete covariance, timestamps, identities and FEJ caches. Test zero/partial/full capacity, invalid counts, input-state validation, preserved cross-blocks and measured storage. No OpenCV types, hot-path allocation or hidden large automatic temporaries. |
| 3 / VINS-04.1 | IMU interval arithmetic / M-PROPAGATE. | Given caller-supplied bracketing samples and interval boundaries, compute interpolation, endpoint averages and bias-corrected held inputs. Check interpolation bounds, units, finite values, zero/reversed intervals and bias changes between segments. Buffer coverage, sample selection and scheduling belong to later integration. |
| 4 / VINS-04.2 | Analytical nominal propagation / M-PROPAGATE. | Implement A1/A2 and Hamilton Exp using shared math. Compare against independent high-precision integration, zero-rate limits and identical-held-input composition. Initial velocity feeds position; biases/clones are unchanged. Bound coefficient approximation on the admitted domain and preserve state on failure. |
| 5 / VINS-04.3 | FEJ covariance propagation / M-PROPAGATE, M-FEJ. | Implement B1/B2, full Psi and Gd*(Qc/h)*Gd^T with the selected frozen references. Verify ordinary derivatives separately from FEJ, all bias/noise blocks, density/time scaling, source-order translation and IMU/clone cross-covariances. Test segment recurrence without requiring unsplit-Qd equality; publish mean/covariance/time/FEJ together. |
| 6 / VINS-09 | Joint correction, injection and reset / M-CORRECT. | Use an SPD solve and Joseph covariance update; inject all IMU/clone means with Hamilton Exp and apply the selected identity reset. Verify cross-blocks, unchanged first estimates, correction limits and failure atomicity. Compare exact Joseph identities and physical-reset discrepancy separately; do not import the formal-eskf reset behavior. Compose production propagation, cloning and correction in synthetic numerical tests using supplied states and observations, without adding an event loop. |
| 7 / VINS-05 | Clone augmentation and removal / M-CLONE. | Implement deterministic pose duplication and retained-coordinate selection, including nominal/FEJ identities and full cross-covariance. Test middle-clone removal, capacity boundaries and valid singular PSD covariance. The low-level removal contract requires the caller to discharge pending-observation ownership. |
| 8 / VINS-07.1 | Camera model and visual Jacobians / M-VISUAL, M-FEJ. | Implement radial-tangential raw-pixel projection, bounded pointwise inverse, extrinsics/lever arm and current/frozen factors. Fix the inverse domain, iteration cap and residual limits before accepting it. Compare derivatives with an independent reference, and test round trips, off-axis/zero-distortion/invalid-depth cases and both reference FEJ distortion policies without silently changing the selected one. |

Measurement QR compression is an optional efficiency step, deferred from this
mathematical scope. Triangulation and feature-error nullspace projection are also deferred.
Using QR for nullspace projection remains distinct from measurement compression. VINS-09 accepts
bounded residual/Jacobian/noise inputs without requiring measurement compression.
The existing Draft profile budgets compressed batches. Before implementing an
uncompressed correction, VINS-02/09 must bind its measurement-row capacity and
workspace explicitly; the compressed workspace bound cannot be reused or the
profile silently changed.

### State capacity and C++ boundaries

Use a simple template binding for the linalg backend and `MaxClones`, meaning
**peak simultaneous clone capacity**, with one resolved state/covariance layout.
The covariance dimension capacity is `15+6*MaxClones`; the active error dimension
remains `15+6*N`. These are error coordinates, not a flattened nominal-state
layout: nominal IMU/clone attitudes retain four quaternion coefficients each.
Do not instantiate a different filter type for every runtime N, or turn the filter
steps into traits/CRTP/policy dispatch. Follow the existing C++ subset and reuse
[BoundedMatrix](../include/formal_gvins/linalg/bounded.hpp).

The first profile retains 10 clones after an image transaction and needs an
additional augmentation slot: `MaxClones=11`, peak D=81. The covariance coefficient
storage alone is 52,488 bytes for double. Measure state, candidate, history and
workspace storage separately; fixed capacity does not by itself bound stack use
or establish a real-time deadline.

`BoundedMatrix::resize` clears its active rectangle. Clone augmentation/removal
must explicitly preserve the required blocks through a candidate/workspace; it
cannot use resize as a data-preserving container operation. Inactive coefficients
are not operands. Capacity overflow is reported before mutation, with the state,
FEJ cache and observation ledger contracts kept distinct and composable.

### Per-action acceptance and commit policy

- Reuse the existing scalar, linalg, SO(3), CMake/CTest and benchmark facilities.
  Add missing operations only for a demonstrated caller; keep backend internals
  and OpenCV outside the filter equations. Use Eigen/double for the first complete
  profile; exercise applicable generic paths separately for other supported
  precision/backend profiles without claiming identical numerical behavior.
- Before coding a formula, fix its input/output frames, error coordinates,
  linearization points, approximation domain and requirement-owned failures.
  Exact symbolic identities, high-precision references and finite differences
  have different scopes. Compare FEJ factors with their frozen-point definitions,
  not with a derivative of the current residual that they intentionally replace.
- Each numerical action extends the independent reference checks and tests the
  actual C++ operation. Include adversarial/boundary cases, retained cross-blocks,
  failure preservation and relevant non-allocation/resource checks. Run targeted
  tests and read-only formatting checks; add sanitizer coverage for new indexing
  and ownership paths. Do not replace failed cases with mocks or weaken tolerances.
- Add the callable workload and relevant accuracy/cost metric to B-01 with the
  module; integrated timing qualification remains deferred under VINS-RT. Keep execution logs and measurements in
  ignored output; commit reusable methods, fixtures and reviewed mappings.
- Self-review must state what changed, why the equations/implementation correspond,
  whether the selected behavior is preserved and what remains unproved. The owner has authorized committing each completed mathematical action after
  self-review and proceeding to the next. This does not authorize deferred work
  or approve Draft requirements.

This sequence verifies equations and implements mathematical filter operations; it does not
claim complete formal verification. The repository currently has no local
Lean/ESBMC suite. Exact mathematical proofs and bounded source checks remain
explicit obligations in the [verification policy](spec/verification.md), with a
separate tooling/proof workstream to be scoped from action 1's inventory. Importing
formal-eskf primitives or passing reference tests does not discharge them. No
full-image, full-trajectory or maximum-dimension symbolic unrolling is assumed.

### Deferred optimization and local VINS integration

These existing items are retained for later planning and are outside the current
mathematical implementation scope. They require a subsequent instruction to
start. End-to-end trajectory/consistency qualification does not replace the
focused equation and numerical composition checks in the mathematical actions.

| Action | Deliverable / requirement owner | Later acceptance boundary |
|---|---|---|
| VINS-07.2 | Triangulation and point refinement / M-VISUAL. | Use normalized bearings for rank-aware initialization and the selected weighted raw-pixel XYZ refinement. Check known 3D scenes, pure rotation, low parallax, conditioning, behind-camera points and bounded nonconvergence; accept no invented landmark prior. Temporary feature storage remains outside the filter state. |
| VINS-08.1 | Per-feature visual constraint and gate / M-CONSTRAINT. | Assemble matching residual/Hx/Hf rows, whiten together, eliminate the feature and gate with the actual retained dimension. Verify annihilation, whitened covariance and least-squares equivalence; exercise correlated noise, rank failure, gate equality and solver failure. |
| VINS-08.2 | Bounded batch accumulation and QR compression / M-CONSTRAINT. | Gate features before incremental compression; retain state-column identity and all informative rows. Verify H^T*H and H^T*r invariance, empty/full batches and overflow behavior. Measure the profile's bounded scratch use; do not allocate the full uncompressed innovation system. |
| VINS-06 | Feature-history ownership / V-TRACK, M-LIFECYCLE. | Bind observations to stable clone/image/camera IDs and retain once-only consumption/retirement records. Test lost/continuing tracks, reused slots, pending clone retirement, history/batch capacity and failed/retried updates. Frontend descriptor ownership stays separate. |
| VINS-10.1 | C++ filter interface and synthetic integration / SW-MSCKF, M-LIFECYCLE. | Connect state validation, propagation, cloning, track selection, constraints, correction and retirement through direct calls. Begin from explicitly supplied valid states. Known synthetic IMU/pixel sequences exercise real production functions; test no-update, degeneracy, delayed/duplicate input and failure transactions. No dataset truth enters the estimator API. |
| VINS-03 | Stationary-start initializer / M-STATE. | Implement the selected IMU/visual-window admission, gravity/yaw convention, bias priors, coupled initial covariance and propagation from the initialization timestamp. Test false stationarity, insufficient tracks, unavailable coverage and invalid priors with synthetic fixtures; empirical threshold qualification remains separate. |
| VINS-11.1 | Raw-image EuRoC/frontend migration / V-ACQUISITION, V-ORB. | Expose original images/calibration and raw feature coordinates; use pointwise undistortion for geometric screening with explicit metric/threshold units. Verify observation identity, coordinate/calibration preservation and inverse failures. Distinguish new raw-image results from earlier remapped-image benchmark baselines. |
| VINS-11.2 | EuRoC VINS replay and trajectory scoring / SW-MSCKF. | Run the C++ filter and initializer with actual frontend observations. Export identified trajectory, update counts, validity/rejection/failure reasons and timing. Score ATE/RPE and attitude/velocity errors with declared timestamp/frame alignment and no free scale fitting; ground truth is evaluation-only. |
| VINS-12 | Integrated regression and consistency qualification / SW-MSCKF. | Extend synthetic sequences with controlled noise and repeated FEJ updates; evaluate observable-subspace or explicitly anchored NEES. Compare IMU-only and VINS, weak geometry/lost vision, and held-out EuRoC data. Select justified accuracy/coverage/numerical thresholds before pinning a baseline; retain failed cases rather than relaxing bounds to pass. |
| VINS-RT | Timing, CPU headroom and real-time execution binding / B-01, M-LIFECYCLE. | Aggregate per-module measurements, test peak capacities and sensor-paced execution, and identify the camera/CPU/GPU/I/O bottleneck. Select budgets, queue/overload policy and output cadence from evidence; test deadlines, backlog and state age. Separate host and Orin qualification; measured maxima are not WCET proofs. |

### VINS-LIT: Literature-based design review

Treat the current formulation/profile as candidates to challenge, not a design to
justify after the fact. This review precedes estimator implementation and produces
one design-analysis document with a source inventory and decision matrix.

The [review](analysis/msckf-design-review.md) covers 12 design areas and recommends
retaining the MSCKF architecture with a fully specified conventional FEJ baseline.
M-FEJ, M-CORRECT, the formulation and profile now define the coordinated Draft
revision: full FEJ transition, Hamilton Exp injection, identity reset and no
accumulated maps. Owner review remains required. Observation-noise qualification and
weak-geometry/no-update status distinctions remain explicit follow-up work.

- Establish coverage through primary papers, supplements and author implementations.
  Record publication/version, search cutoff, scope, review status and inclusion or
  exclusion reasons. Follow references and relevant newer work; distinguish
  foundational methods, established baselines and recent proposals instead of
  labelling every paper SOTA. An abstract or repository README is not equation review.
- Cover original MSCKF, FEJ and observability-constrained VINS; use
  [OpenVINS](https://yangyulin.net/papers/2020_icra_ov.pdf) and
  [S-MSCKF](https://arxiv.org/abs/1712.00036) as implementation-oriented references.
  Compare invariant/equivariant alternatives, including
  [EqVIO](https://arxiv.org/abs/2205.01980) and
  [MSCEqF](https://arxiv.org/abs/2311.11649), and screen recent work such as
  [pose-only MSCKF](https://arxiv.org/abs/2407.01888),
  [immediate updates](https://ieeexplore.ieee.org/document/10918798/) and
  [equivariant filter transformations](https://arxiv.org/abs/2603.24130).
  These are discovery seeds, not a completed or exhaustive review inventory.
- Compare complementary design choices in
  [VINS-Mono](https://arxiv.org/abs/1708.03852),
  [OKVIS2](https://arxiv.org/abs/2202.09199),
  [square-root sliding-window estimation](https://arxiv.org/abs/2109.02182) and
  [ORB-SLAM3](https://arxiv.org/abs/2007.11898). Separate local VIO from mapping,
  loop correction and GNSS extensions; do not inherit smoother equations or
  landmark-reuse rules into a filter without deriving the change.
- Cross-check state/error conventions, observability, propagation/noise integration,
  FEJ/injection/reset, initialization and priors, clone/track lifetime, triangulation,
  visual noise/Jacobians, nullspace/whitening/gating, compression/covariance update,
  failure/recovery, calibration/time assumptions and resource/timing tradeoffs.
  Translate each source into the project's frames, state order and perturbation
  side before comparing formulas. Treat ORB measurement quality and the selected
  thresholds as empirical questions, not consequences of adopting MSCKF.
- For each design decision, record the owning requirement/profile, source
  section/equation, relevant premises, local difference, mathematical/numerical
  evidence still needed and disposition: supported, justified adaptation, change
  required, or unresolved. Pin implementation revisions for code comparisons.
  Resolve correctness-critical differences before affected code proceeds; retain
  explicit acceptance work for empirical thresholds and deployment performance.
- Review the complete FEJ transition, analytical Gd/Qd binding and identity-reset
  approximation together. Check conventions and unobservable directions beyond
  algebraic cancellation. Keep linear-model identities, numerical comparisons
  and nonlinear/statistical consistency as separate evidence; the revised Draft
  specifies the reference differences instead of claiming complete equivalence.

Completion means every listed design area has a reviewed disposition and all
implementation-blocking issues have a resolution proposed for owner review. It
does not mean every published VINS paper was read, or that matching literature
proves this implementation correct. Identify unreviewed relevant sources and
unresolved evidence explicitly. Feed timing/cost questions into VINS-RT and
consistency/accuracy experiments into VINS-10/12.

### VINS-RT: CPU headroom, sustainable rates and bottlenecks

Collect B-01 measurements as each module is implemented, and assess initial CPU
headroom after VINS-09 before integration. The deferred VINS-RT action uses the integrated VINS-10/11
workloads to select and qualify real-time execution. This task owns estimator
scheduling and timely state delivery to an independent flight controller; it does
not add an attitude/rate controller.

1. Measure preprocessing, extraction, matching, IMU propagation, triangulation,
   constraint reduction, correction/reset and output handling separately. Include
   bounded maximum clone/track workloads. Report CPU time, wall time, P50/P95/P99
   and observed maximum; separate file decoding, sensor waits and GPU transfers/
   synchronization where applicable from estimator computation.
2. Report per-thread/per-core utilization and headroom, not just machine-wide idle
   percentage. Include IMU work between images, background load and thermal/power
   settings. Select an explicit reserve and deadline budget from the measurements;
   an idle core does not remove a saturated sequential stage or a latency spike.
3. Sweep offered processing rates and workload sizes to estimate a sustainable
   compute ceiling, then validate sensor-paced execution with bounded queues.
   Report backlog, dropped frames, deadline misses and state age alongside achieved
   Hz. A single inverse mean runtime or accelerated replay is insufficient. Give
   the measured operating range and remaining uncertainty, not a presumed rate.
4. Separate camera acquisition rate, visual-correction rate, IMU propagation rate
   and state-publication rate. The first EuRoC profile supplies 20 Hz images and
   200 Hz IMU. Faster publication can carry an IMU-propagated estimate; it does not
   create additional visual corrections. If computation has adequate headroom at
   the camera rate, explicitly identify acquisition as the visual-rate limit.
   Otherwise identify the limiting CPU stage, GPU/transfer, I/O or scheduling path.
5. Use the result to select execution ownership, priorities, queue capacities,
   stale-data/overload behavior and any required current-time output prediction.
   Validate these choices during integration, including acquisition-to-publication
   latency where timestamps permit it. Do not silently drop data or change timing
   semantics to meet a throughput target.
6. Record desktop and Orin Nano results separately, including CPU/thread settings,
   build, frontend, image size and estimator capacities. Host/RTX 3060 results give
   a development estimate, not Orin qualification. Report observed timing and
   selected margin explicitly; measurements alone do not prove hard-real-time WCET.

### Verification alongside implementation

- Use independently generated synthetic motion/geometry to isolate equations
  before debugging real-image association. Verify ordinary derivatives at their
  evaluation points; FEJ Jacobians are intentionally different from current-state
  derivatives and need their own frozen-gauge and approximation checks.
- Prioritize exact propagation/clone, elimination, Joseph/reset and FEJ identities
  for mathematical proofs. Scope source verification to bounded layout, numerical
  operations, transaction failure and observation-consumption properties.
  OpenCV/CUDA internals and complete image streams are outside those proof claims.
- Extend B-01 with each implemented module's accuracy/residual and cost metrics.
  Add a trajectory benchmark category when the estimator exists. Native tests,
  mathematical proofs and source verification remain distinct evidence; imported
  mathematics does not import completed Lean/ESBMC coverage into this repository.
- A working local VINS milestone requires qualified initialization, actual visual
  updates, measured trajectory behavior and explicit invalid/lost states. It does
  not establish full formal verification, flight readiness or Orin deadlines.

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
