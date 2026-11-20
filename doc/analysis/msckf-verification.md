# MSCKF mathematics verification

This inventory separates equation checks from C++ correspondence and formal proof.
The [formulation](../spec/architecture/msckf-formulation.md) owns the equations;
the [work list](../roadmap.md) owns implementation scope. Requirements remain Draft.

Run the independent baseline with:

```sh
python3 -m pip install -r tests/reference/requirements.txt
python3 -m unittest discover -s tests/reference -v
```

Alternatively enable `FORMAL_GVINS_BUILD_MATH_REFERENCE_TESTS` in CMake/CTest.
The reference uses SciPy matrix exponentials, complex-step differentiation and
60-digit mpmath calculations, not production coefficient series or quadrature.
Fixed seeds and boundary cases make the checks repeatable. These are numerical
checks, not exhaustive proofs or a Lean-to-C++ refinement.

| Requirement / obligation | Independent check | Remaining proof or acceptance |
|---|---|---|
| M-PROPAGATE: held-input integrals and zero-rate limits | `test_integrals_high_precision_and_limits`; augmented matrix exponential and 60-digit comparison | Uniform truncation/rounding bounds, supported precision/target profiles |
| M-PROPAGATE: full physical derivative | `test_physical_derivative_and_fej_gauge`; rotating-frame ODE exponential and finite differences | Exact derivative proof; uniform floating-point bounds |
| M-FEJ: frozen transition | Same test; four-column gauge transport with different current/first estimates | Exact gauge and source-convention correspondence; repeated-update statistical qualification |
| M-PROPAGATE: interval-equivalent noise | `test_noise_scaling_and_segment_recurrence`; h^3/4 position variance, 5h^3/16 split example and bias-only scaling | Congruence PSD proof; uniform numerical bounds and segmentation-model qualification |
| M-CLONE: augmentation/selection | `test_clone_selection_joseph_and_identity_reset`; full covariance congruence | Exact selection/PSD proofs; lifecycle integration |
| M-CORRECT: Joseph and reset | Same test; Joseph/conditional-covariance identity and physical Jr derivative | Exact identities, uniform numerical bounds and accumulated consistency |
| M-VISUAL: projection and current/frozen derivatives | `msckf_cpp_reference` central differences and gauge checks; native camera tests | Exact derivatives and uniform numerical bounds; no triangulation, nullspace or image-pipeline claim in this scope |

C++ tests must call the actual production operations. Reference inputs must cover
N=0 and populated clone states, off-diagonal correlations, nonidentity attitudes,
current/FEJ divergence, invalid domains and transactional failure. PSD of a supplied
prior or density remains a caller premise; finite/symmetry checks do not prove it.
Keep native memory checks, mathematical identities and statistical evidence distinct.

The benchmark protocol in B-01 applies to each callable mathematical module.
Execution logs, timings and stack reports belong in ignored output, not this file.

## State admission

`msckf::State`, `validate` and `assign` implement bounded layout and numerical
admission (`M-STATE`), with tests in `tests/test_msckf_state.cpp`. N=0 and capacity
11 are exercised with allocation disabled. Tests cover active dimensions, duplicate
IDs/times, inactive slots, asymmetry, non-finite inputs and failed assignment.
They do not prove the PSD or physical suitability of a caller-supplied prior.

## IMU interval arithmetic

`interpolate`, `prepare_interval` and `validate_interval` implement M-PROPAGATE's
input arithmetic; `tests/test_msckf_imu.cpp` checks integer-epoch interpolation,
endpoint reuse, bias recomputation, original-gap limits, domains and atomicity.
The caller supplies the raw bracket, prevents interior-sample skipping and owns
buffer/time scheduling. No sensor stream or noise-correlation claim is made.

## Nominal propagation

`predict_nominal` implements the held-body-input mean. Native tests check old
velocity, unchanged biases, aliasing, split-interval composition and failure.
`msckf_cpp_reference` sends boundary/random inputs to the production probe and
compares against the independent augmented-exponential oracle. The double profile
uses 1e-12 coefficient tolerances for this mean comparison; this does not certify
uniform floating-point bounds or arbitrary time-varying input accuracy.

## Joint covariance propagation

`predict` and `PredictionWorkspace` implement the full frozen transition and
interval-equivalent Qd. The production probe checks every Psi/Qd coefficient and
joint covariance against independent exponentials/complex-step sensitivities,
including differing first estimates, correlated/zero noise, domain endpoints and
both sides of the reference small-rate threshold. Native tests exercise peak clone
capacity, preserved clone blocks, timestamps, workspace alias rejection and
transactional failure. Test tolerances follow the profile's 1e-11 transition and
1e-9 relative Qd limits; sampled agreement is not a uniform error proof.

## Generic correction

`correct` consumes externally assembled r/H/V and implements the joint Joseph
update, all-pose Hamilton injection and identity reset. `CorrectionWorkspace`
separately bounds measurement rows; compressed-batch storage is not assumed.
Native cases include N=0, peak clones, singular PSD P, invalid V with positive S,
last-clone angle rejection, non-finite inputs and late overflow atomicity. The
production oracle checks correlated V, every mean and the complete posterior
against a separately evaluated conditional-covariance identity. First estimates
remain unchanged. These tests do not supply MSCKF feature constraints or validate
an end-to-end visual estimator.

## Clone operations

`augment_clone` copies the IMU pose and its distinct first estimate using the
complete duplication congruence. `remove_clone` selects the retained principal
submatrix. Native tests inspect every retained/copy coefficient, peak/zero capacity,
identities/times, roundoff canonicalization and failure preservation. A synthetic
sequence composes prediction, supplied-position correction, cloning and removal.
The caller still owns observation retirement and session-wide identity; this is
not an implemented feature lifecycle or a visual-observability experiment.

## Camera observation

`project` and `camera_observation` implement radtan raw-pixel prediction and the
fully frozen visual derivatives. The oracle differentiates an independent forward
model under right perturbations, including nonidentity extrinsics/lever arms and
nonzero distortion, and checks the four-column geometric gauge cancellation.
Native tests separate current residual from frozen Jacobians and check depth,
calibration, radius/finiteness and failure preservation. Reference bounds are
1e-10 pixels for residuals and 2e-7 absolute / 2e-8 relative for central-difference
Jacobians (step 1e-6), with 1e-10 gauge residual. Pointwise inverse, triangulation
and nullspace construction remain deferred; this is not a complete visual update.
