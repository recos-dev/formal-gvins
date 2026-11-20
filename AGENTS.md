# Agent Instructions

formal-gvins develops a portable C++20 MSCKF-based GNSS/visual/inertial estimator.
The numerical core is independent of deployment platforms and linalg backends.
Do not infer implemented capabilities or verification from the project name or
from reference projects. Inspect the current source and evidence first.

## Start with the task

Run `git status --short`; preserve unrelated edits, staged files and ongoing runs.
Work in this repository. Sibling projects are references, not edit or commit
targets unless explicitly requested. Read affected source, callers and tests,
then the relevant contracts below.

| Task | Read |
|---|---|
| Behavior or architecture | [Documentation index](doc/README.md), then the affected [specifications](doc/spec/README.md) |
| C++ implementation | [Coding style](doc/guides/coding-style.md), [C++ subset and ArduPilot design baseline](doc/spec/requirements/common/cpp-subset.md), [.clang-format](.clang-format) |
| Specification changes | [Writing template and approval rules](doc/spec/TEMPLATE.md) |
| Verification or evidence | [Evidence ownership](doc/README.md#evidence-and-maintenance), the affected requirement's acceptance criteria and available verification tooling |

Specifications own behavior; logs and analysis proposals do not override them.
Verify paper notation/equations against the reference. State rotation direction,
quaternion convention, perturbation side and coordinate frames before adapting
formulas. Do not mix OpenVINS conventions with this project's Hamilton formulation.
Surface conflicts before changing the model. Draft proposals require review;
task completion or permission to commit does not approve a requirement.

## Make the change

- Preserve behavior, defaults, numerical evaluation order, failure precedence,
  atomicity and aliasing during refactors. Separate intended behavior changes
  from structural cleanup; do not bundle unrelated work.
- Keep numerical storage and execution bounded, without dynamic allocation,
  exceptions, RTTI or OS facilities in the shared core. Active counts may vary
  only within declared capacities. Keep large workspaces and ownership visible.
- Use the shared scalar, linalg and SO(3) interfaces. Keep Eigen/CMSIS-DSP types,
  image-processing libraries and platform effects outside estimator equations.
- Follow ArduPilot's same-layer design baseline through the linked subset.
  Prefer direct numerical/control flow; avoid redundant wrappers and dispatch.
- Keep feature-track measurements separate from the filter's joint state and
  covariance. Any change to clone ownership, timestamps or measurement reuse
  needs explicit behavioral review.
- Reuse repository tools when present; verify their options before use. Do not
  assume sibling projects' runners, configuration or dependencies exist here.
  Use uppercase shell variables and readable functions.
- Preserve upstream license and attribution notices when importing code or docs.
  Do not vendor external platform/library code without authorization.
- Write documentation in English. Preserve requirement IDs, approval states and
  claim boundaries. Link canonical definitions instead of copying requirements,
  mutable proof statistics or development diaries.

## Validate the affected scope

Choose checks for the changed behavior, dependencies and configurations. Prefer
targeted builds/tests and read resource limits before heavy verification. Record
compiler, backend, precision, build options and source identity with evidence.
Documentation-only changes need link, reference and consistency checks, not a
full numerical test or proof run.

- Keep formatting checks read-only; use clang-format 14 with `.clang-format`.
- Build and test the affected backend profiles. Host results do not establish
  ARM execution, timing or embedded resource bounds.
- Verify production paths and required producer/caller dependencies. Never weaken
  assertions, assume away failures or substitute a rewritten implementation just
  to pass. Missing/stale evidence and incomplete checks remain gaps.
- Keep exact mathematical proofs, bounded source/IEEE checks, native tests and
  agent review distinct. Imported source does not import proof coverage. Claim a
  Lean-to-C++ refinement only when an actual checked connection exists.

## Review and hand off

- Review the complete task diff: what changed, why it is correct, whether behavior
  is preserved, and what the checks establish. Report failures, unrun checks and
  remaining gaps; do not extend claims to unverified targets.
- Stage, commit, push or rewrite commit metadata only when explicitly authorized.
  Select exact task files and review the staged diff. Exclude trial notes, logs
  and transient outputs; generated release artifacts require a declared policy.
- Keep drafts requested for review out of commits. Do not edit an unrelated
  specification or silently expand the verification scope to close a task.
