# Development work list

This list tracks planned work. It does not approve requirements or claim completed
verification. Existing frontend decisions are recorded in the
[ORB reference comparison](analysis/orb-frontend-settings.md).

## Local VINS implementation sequence

**Status:** Plan retained for review. VINS-01 has a
[Draft profile](spec/profiles/local-vins.md) ready for owner review;
VINS-LIT has a [literature/design review](analysis/msckf-design-review.md)
ready for owner review. The conventional-FEJ/Exp/identity-reset revision is
Draft; owner review and remaining profile clarifications precede dependent code.
VINS-02 through VINS-12 and VINS-RT are unimplemented.
The current repository has numerical primitives, an ORB frontend, EuRoC input and
feature benchmarks. It does not yet implement the joint MSCKF estimator.

The first milestone is an offline monocular visual-inertial estimator producing
metric local pose, velocity, biases and covariance from cam0 images and IMU data.
Start with CPU ORB, Eigen/double, fixed calibration and a declared camera/IMU
clock binding. The selected raw-image profile requires the frontend migration
below. Use a qualified stationary-start policy initially;
moving-start initialization is a separate extension. Ground truth is evaluation
input only. This plan does not make GNSS, stereo, loop closure, relocalization,
online calibration or Orin optimization prerequisites for the first local VINS.

The [MSCKF formulation](spec/architecture/msckf-formulation.md) and its
[requirement allocation](spec/requirements/software/local-vio.md) own the model.
In particular, right-local errors, analytical interval-equivalent noise derived
from continuous IMU density, and full joint covariance must remain consistent;
importing formal-eskf's INS implementation does not establish that it implements
this model. These Draft requirements and
the revised conventional FEJ/identity-reset binding still require review.

| Order / stable ID | Deliverable | Completion boundary |
|---|---|---|
| 1 / VINS-01 | Review the first estimator profile and consistency design. | Select capacities/workspaces, startup assumptions, interval/noise evaluation, geometry/noise/gate criteria and failure outcomes. Review the complete conventional-FEJ/Exp/identity-reset binding and its approximation limits before dependent production code; update affected Draft contracts explicitly if the design changes. |
| 2 / VINS-LIT | Cross-check the design against MSCKF/VINS literature. | Build a source inventory covering foundations and relevant recent methods; review every estimator design area against primary equations and assumptions. Resolve unsupported adaptations, especially FEJ/reset, before dependent production code. Deliver a decision matrix and required design revisions for owner review. |
| 3 / VINS-02 | Bounded joint state and layout. | Current IMU state, time, clone identities, full `D=15+6N` covariance and required FEJ caches have explicit ownership. Zero/full-capacity and block-index tests pass; OpenCV types do not enter the numerical core. |
| 4 / VINS-03 | Stationary initialization. | Use a qualified sensor window to establish gravity-aligned attitude, gyro-bias estimate and declared velocity/accelerometer-bias priors, with justified initial covariance. Origin/yaw are local gauge choices. Reject an unsuitable window without claiming that all biases or motion are observable from stationary IMU alone. |
| 5 / VINS-04 | IMU nominal and joint-covariance propagation. | Implement endpoint averaging/interpolation, analytical held-body-input mean, ordinary derivative, full FEJ transition and Gd*(Qc/h)*Gd^T with consistent frozen references. Verify coefficient evaluation, source-order translation, zero-rate limits, cross-blocks and image-time segment recurrence; do not require split/unsplit Qd equality. Split mean propagation and covariance propagation into separate reviewable changes. |
| 6 / VINS-05 | Pose clone augmentation and removal. | Clone at the actual image time; retain all IMU/clone and clone/clone covariance blocks and corresponding frozen first estimates. Test middle-clone removal, bounded capacity and exact PSD/singular augmentation semantics. |
| 7 / VINS-06 | Feature-history ownership and consumption. | Associate observations with stable image/clone IDs; retain bounded histories, handle track loss and pending clone retirement, and prepare once-only fusion records. Failed/retried updates and reused storage cannot rebind or double-consume observations. |
| 8 / VINS-07 | Projection, triangulation and Jacobians. | Implement fixed extrinsics/lever arm, raw-pixel distortion/projection, bounded pointwise inverse and triangulation/refinement, and current/frozen linearization bindings. Independent synthetic scenes verify camera-domain limits, derivatives, pure rotation, low parallax and invalid depth. Camera projection/inverse and triangulation can be reviewed separately. |
| 9 / VINS-08 | MSCKF visual constraints. | Assemble/whiten matching residual and Jacobian rows, eliminate feature error, apply per-feature innovation gates and bound batches/compression. Check annihilation and information identities; rejected/empty constraints do not become measurements. |
| 10 / VINS-09 | Joint correction, injection and reset. | SPD solve and Joseph update modify current IMU and all clones through the full covariance. Apply Hamilton Exp injection and the declared identity-reset approximation, retaining first estimates and all Joseph cross-blocks. Candidate failure preserves the complete pre-correction transaction. |
| 11 / VINS-RT | Real-time execution for control consumers. | Analyze CPU headroom and stage latency first, estimate sustainable IMU/visual/output rates, and identify the limiting resource. Select scheduling, deadlines and overload behavior from that evidence; qualify the integrated result in VINS-10/11. See the timing scope below. |
| 12 / VINS-10 | Local estimator event loop and synthetic integration. | Integrate IMU coverage, image-time propagation, cloning, track selection, correction, consumption and retirement. A known synthetic trajectory runs through the entire estimator; test lost vision, duplicate/out-of-order input, missing IMU, capacity limits and failures/retries. |
| 13 / VINS-11 | EuRoC VINS replay and trajectory evaluation. | Connect the existing reader and CPU frontend to the estimator; export time/frame/validity-bound state, trajectory plots, failures and latency. Run the selected sequence from a qualified startup window, not only the 100-frame frontend benchmark. Ground truth never initializes or corrects the estimator in the normal replay. |
| 14 / VINS-12 | Review and pin the first VINS regression profile. | Compare IMU-only and visual-inertial estimates, trajectory ATE/RPE, attitude/velocity errors, valid coverage and numerical health. Fix timestamp/frame alignment without free scale fitting, inspect failure cases, and validate additional sequences not used for tuning. Set reviewed thresholds before accepting a compact baseline; retain host/build/input identities. |

Complete VINS-LIT after VINS-01, then review the resulting formulation/profile
before VINS-02 implementation. Small numerical checks support design exploration;
they do not establish literature alignment, production-code or trajectory
acceptance. Preserve these IDs when recording progress; expose newly
discovered work as a dependency or extension instead of silently replacing this
sequence. A row is an acceptance boundary, not a requirement to combine all its
code into one commit. Each change includes the relevant contract, implementation,
tests and self-review before submission for review.

The revised camera binding adds a prerequisite to VINS-03's visual startup window
and VINS-07/11: expose original EuRoC images and calibration, keep ORB observations
in raw pixels, use pointwise undistortion for essential-matrix geometry, and bind
the geometric metric/threshold units. Update coordinate metadata/contracts and
tests together; qualify raw-image tracking/noise and benchmark results separately
from existing remapped-image runs. This migration is pending, not part of the
completed frontend. See the [ORB target](spec/profiles/orb-frontend.md#local-vins-target).

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

Start after VINS-09 with the implemented modules and B-01 measurements. This task
owns estimator scheduling and timely state delivery to an independent flight
controller; it does not add an attitude/rate controller. Initial estimates precede
integration; final sustainable-rate claims require VINS-10/11 execution.

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
