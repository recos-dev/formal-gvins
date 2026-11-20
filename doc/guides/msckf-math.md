# MSCKF mathematical core

The core implements the selected Hamilton/right-local Draft model. It has no
feature manager, triangulation, feature elimination, image pipeline or event loop.
[Requirements](../spec/README.md) own behavior; [verification mappings](../analysis/msckf-verification.md)
separate native/reference checks from remaining proof obligations.

## Interfaces and ownership

All functions are in `formal_gvins::msckf`; headers are under
`include/formal_gvins/msckf/`.

| Header | Interface | Responsibility |
|---|---|---|
| `state.hpp` | `State<Linalg, MaxClones>`, `validate`, `assign` | IMU state, clones, first estimates and joint covariance; bounded layout/numerical admission |
| `imu.hpp` | `interpolate`, `prepare_interval` | Covered-bracket arithmetic and current-bias endpoint averaging |
| `prediction.hpp` | `predict_nominal`, `predict`, `PredictionWorkspace` | Analytical held-input mean and full FEJ covariance/noise propagation |
| `correction.hpp` | `correct`, `CorrectionWorkspace<Linalg, MaxClones, MaxRows>` | Supplied r/H/V update; Joseph, all-pose Exp and identity reset |
| `clones.hpp` | `augment_clone`, `remove_clone` | Full duplication congruence and retained covariance submatrix |
| `camera.hpp` | `project`, `camera_observation` | Raw-pixel radtan projection, residual and frozen pose/feature Jacobians |

For Eigen/double, bind `Linalg` to `linalg::EigenBackend<double>`.
`MaxClones` is peak capacity, not the current count. N=0 has 15 error coordinates;
N clones have 15+6N. The Draft local profile retains 10 clones and reserves one
augmentation slot. No allocation, exceptions, RTTI or OpenCV enters these APIs.

The caller constructs a prior and initial first estimates, establishes PSD and
frame/time/calibration premises, then uses `assign` to admit it. A zero-constructed
state is numerically representable, not a physically initialized estimator.
`Tolerances<double>{1e-12, 1e-12}` selects the current double unit-norm/symmetry
checks. They do not qualify binary32 automatically.

The interval domain is 1 ns through 10 ms, corrected angular-rate norm at most
10 rad/s and specific-force norm at most 100 m/s^2. Input preparation must be redone
with the current biases after correction. The caller ensures there is no skipped
interior raw IMU sample and that interpolation does not bypass an excessive raw
gap. Gravity is supplied explicitly in W; it is not an estimated state component.

`predict` uses Qc density ordered `(n_a,n_g,n_ba,n_bg)`, the full frozen Psi and
`Gd*(Qc/h)*Gd^T`. It is not the continuous-white-noise covariance integral.
`correct` uses nonempty, already admitted r/H/V in matching coordinates; maximum
attitude correction is an explicit argument in [0,0.2] rad. There is no gate,
compression or observation-consumption policy in this function. An unknown feature's
`H_feature` cannot simply be dropped to turn a camera observation into a constraint.

State is preserved on failure. Large candidate/work matrices are caller-owned
scratch; keep scratch disjoint from state and other inputs. Scratch may change on
failure. Numerical covariance admission checks finite values and relative
infinity-norm symmetry, then the owner mirrors the upper triangle. It does not
prove PSD or add diagonal jitter. Solver temporaries add bounded stack use beyond
`sizeof(workspace)`; inspect compiler stack reports for the selected build/capacity.

Camera projection requires explicit depth/radius limits and calibrated intrinsics.
`q_BC` rotates camera to body; `p_BC` is the camera origin in body coordinates.
Residuals use current poses, while H_pose/H_feature use first estimates and the
same supplied global point. Raw image/calibration identity remains external.
Pointwise inverse and feature estimation are deferred with triangulation.

## Tests and benchmark

```sh
python3 -m pip install -r tests/reference/requirements.txt
cmake -S . -B build/math -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
    -DFORMAL_GVINS_BUILD_MATH_REFERENCE_TESTS=ON
cmake --build build/math --parallel 2
ctest --test-dir build/math --output-on-failure
```

To measure production operations independently of image input:

```sh
cmake -S . -B build/math-release -DCMAKE_BUILD_TYPE=Release \
    -DFORMAL_GVINS_BUILD_MATH_BENCHMARKS=ON
cmake --build build/math-release --target msckf_benchmark --parallel 2
mkdir -p output/math
./build/math-release/msckf_benchmark > output/math/benchmark.csv
```

The serial Eigen/double benchmark reports 50 timed samples after five warmups,
median/P95/maximum wall time and a named analytical residual. Fixture resets and
result checks are outside the timed operation. N=0 and peak clone workloads are
separate rows; augmentation begins with ten clones. Correction uses three supplied
position rows. These simple checks accompany, not replace, randomized oracle tests.
A failed status, state check or residual check aborts the benchmark with a nonzero
exit code. Core tests, the probe and benchmark retain Eigen allocation assertions
in Debug and Release. No timing threshold or regression baseline is approved. Record source identity,
compiler/flags, host and dependency versions with each retained execution; host
measurements do not establish Orin throughput, flight readiness or WCET.
