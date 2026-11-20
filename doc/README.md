# Documentation

formal-gvins develops GNSS/visual/inertial estimation around an MSCKF architecture.
Start with the relevant specification or guide; directory structure is not a
requirement-decomposition relationship.

| Subject | Entry |
|---|---|
| Specification format and approval | [Writing standard and templates](spec/TEMPLATE.md) |
| Requirements | [Specification index](spec/README.md) |
| C++ readability and organization | [Coding style](guides/coding-style.md) |
| Language and execution restrictions | [C++ subset](spec/requirements/common/cpp-subset.md) |

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

The writing template, coding guide, C++ subset and formatter configuration are
adapted from [formal-eskf revision 6e4ffb5](https://github.com/recos-dev/formal-eskf/tree/6e4ffb5bc5932804c177cbacfd5a10e3126f981a).
