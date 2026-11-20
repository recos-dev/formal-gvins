# Documentation

formal-gvins develops GNSS/visual/inertial estimation around an MSCKF architecture.
Start with the relevant specification or guide; directory structure is not a
requirement-decomposition relationship.

| Subject | Entry |
|---|---|
| Specification format and approval | [Writing standard and templates](spec/TEMPLATE.md) |
| Requirements | [Specification index](spec/README.md) |
| Verification and evidence acceptance | [Verification policy](spec/verification.md) |
| Requirement implementation and evidence gaps | [Requirements inventory](analysis/requirements-inventory.md) |
| Planned development work | [Work list](roadmap.md) |
| C++ readability and organization | [Coding style](guides/coding-style.md) |
| Language and execution restrictions | [C++ subset](spec/requirements/common/cpp-subset.md) |
| Offline experiments and regression data | [Dataset selection and regression proposal](analysis/offline-datasets.md) |
| Downloading development recordings | [Dataset download guide](guides/datasets.md) |
| CPU/CUDA frontend and EuRoC replay | [Frontend build and usage](guides/frontend.md) |
| ORB settings and reference implementations | [Frontend parameter comparison](analysis/orb-frontend-settings.md) |
| Feature frontend regression and comparison | [Benchmark design proposal](analysis/feature-benchmarks.md) |
| Bounded MSCKF numerical operations | [Linear algebra inventory and implementation boundary](analysis/msckf-linear-algebra.md) |

## Evidence and maintenance

Specifications own required behavior and acceptance. Requirements start as Draft;
permission to commit does not approve them. Preserve stable IDs for traceability.
Architecture and derivations explain the model without creating duplicate
requirement definitions. Keep draft proposals distinct from accepted behavior.

Verification records must identify requirements, assumptions, source revision,
configuration and tool/target identity. A source file, test or proof copied from
another project does not transfer its verification claim. Development logs and
trial outputs stay outside versioned documentation. Git owns revision history.

## Source attribution

The writing template, coding guide, C++ subset, formatter configuration,
reusable mathematical requirements and tailored numerical policies are
adapted from [formal-eskf revision 6e4ffb5](https://github.com/recos-dev/formal-eskf/tree/6e4ffb5bc5932804c177cbacfd5a10e3126f981a).
The project code and documentation use [Apache-2.0](../LICENSE). New vision and
frame contracts are project-authored proposals, indexed with their Draft status.
