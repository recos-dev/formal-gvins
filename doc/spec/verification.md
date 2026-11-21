# Verification and evidence policy

**Status:** Draft

This policy defines evidence acceptance for the [requirements](README.md).
[The template](TEMPLATE.md) owns requirement identity and approval. The
[inventory](../analysis/requirements-inventory.md) records candidate source/test
mappings and gaps; it is not an execution report or an automated verifier.

## Mapping and evidence identity

For each claim, record requirement ID and clause, specification revision,
production symbol, verification obligation/case, method, input domain and profile.
A parent also needs its own composition obligations. Preserve declared denominators
when requirements are decomposed; count parents, children and individual claims
separately rather than manufacturing coverage by adding links.

An execution record identifies source commit plus dirty-content identity if any,
mapping/specification revisions, tool/compiler/dependency versions, flags, scalar,
backend, target, input/dataset identity and outcome. Failed, incomplete, stale and
not-run work remains distinguishable. A file or test name alone is not a passing
result. Keep transient logs/measurements in ignored output; retain the method,
reviewed mapping and any deliberately selected release evidence under its policy.

## Separate claim types

| Evidence | Can address | Does not establish by itself |
|---|---|---|
| Exact mathematical proof | Algebra, derivatives, covariance/orthogonality/gauge identities under named premises | Floating-point code, physical model validity or target execution |
| Scoped C++/floating-point verification | Selected production paths, bounded indices, failure/alias transactions and supported numerical properties | Unchecked caller premises, external-library internals or all configurations |
| Native tests and sanitizers | Concrete cases, reference comparisons and exercised memory/undefined-behavior faults | Exhaustive proof, model truth or WCET |
| Module/integration benchmarks | Defined residual/accuracy/resource measurements on identified inputs/targets | General numerical bounds, physical feature identity from inlier counts, or unseen hardware |
| Agent/manual review | Traceability, premise and implementation/spec correspondence findings | A machine-checked Lean-to-C++ refinement or absence of all defects |

Imported math/code/tests require a local correspondence and applicability review.
CPU/CUDA, Eigen/CMSIS and binary32/binary64 claims have separate profiles. Reviewed
backend summaries are assumptions, not proofs of their complete implementations.
Scope any formal-check dimension/domain/resource limits explicitly; tool completion
alone does not establish the completeness of the claim set.

## Numerical and benchmark acceptance

[C-NUMERICAL-ACCEPTANCE](requirements/common/numerical-acceptance.md) owns residual
and tolerance obligations. Bind domain, units, norm, scale and preselected bounds
before using measurements to accept a claim. Tests can guide a proposed bound;
do not choose a tolerance after seeing a failure and call it unchanged acceptance.
Maintain distinct rounding, approximation and math-library assumptions.

The [module benchmarks](../roadmap.md#b-01-per-module-unit-benchmarks)
record workload, warmup, sample count, median/P95/maximum observed latency and
failures, together with an appropriate reference/error metric. CPU/GPU comparisons
need matched algorithm settings and transfer/synchronization costs. RMSE requires
a defined quantity, units, reference/alignment and sample set; report unavailable
ground truth explicitly. Observed maximum is not WCET. Frontend timing and inliers
do not establish trajectory accuracy.

## Acceptance and changes

Accept a requirement claim only when its required premises, numerical/profile
decisions and mapped evidence are discharged. Explicit owner approval of the
requirement is a separate decision. Open profile values do not silently become
defaults or waived obligations. Parent acceptance needs child, dependency and
composition evidence, including transaction/time/ownership behavior.

Changes to contracts, source, dependencies, compiler flags, calibration/noise,
capacity or target trigger review of affected mappings and evidence applicability.
Editorial moves preserve IDs and repair links without creating new execution
results. This policy does not imply that a proof runner, coverage dashboard or
release evidence archive has already been implemented.
