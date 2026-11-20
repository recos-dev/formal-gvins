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
| M-PROPAGATE: full physical derivative | `test_physical_derivative_and_fej_gauge`; rotating-frame ODE exponential and finite differences | Exact derivative proof; production correspondence |
| M-FEJ: frozen transition | Same test; four-column gauge transport with different current/first estimates | Exact gauge and source-convention correspondence; repeated-update statistical qualification |
| M-PROPAGATE: interval-equivalent noise | `test_noise_scaling_and_segment_recurrence`; h^3/4 position variance, 5h^3/16 split example and bias-only scaling | Congruence PSD proof; correlated-driver C++ tests; segmentation-model qualification |
| M-CLONE: augmentation/selection | `test_clone_selection_joseph_and_identity_reset`; full covariance congruence | Layout, selection and failure-preservation C++ checks; exact PSD proof |
| M-CORRECT: Joseph and reset | Same test; Joseph/conditional-covariance identity and physical Jr derivative | C++ correspondence, failure/alias/domain checks; exact identities and accumulated consistency |
| M-VISUAL: projection and current/frozen derivatives | Pending independent derivative and gauge checks with camera implementation | No triangulation, nullspace or image-pipeline claim in this scope |

C++ tests must call the actual production operations. Reference inputs must cover
N=0 and populated clone states, off-diagonal correlations, nonidentity attitudes,
current/FEJ divergence, invalid domains and transactional failure. PSD of a supplied
prior or density remains a caller premise; finite/symmetry checks do not prove it.
Keep native memory checks, mathematical identities and statistical evidence distinct.

The benchmark protocol in B-01 applies to each callable mathematical module.
Execution logs, timings and stack reports belong in ignored output, not this file.
