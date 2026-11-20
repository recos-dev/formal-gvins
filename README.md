# formal-gvins

A portable GNSS/visual/inertial estimation project using an MSCKF architecture
and specification-oriented development.

The current implementation provides fixed-size linear algebra, scalar math and
SO(3)/unit-quaternion operations. An MSCKF estimator, visual frontend and GNSS
integration are not implemented yet. No formal-verification coverage is claimed
by this initial import.

## Build and test

Install a C++20 compiler, CMake and Eigen 3.3 or newer, then run:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure --no-tests=error
```

Link `formal_gvins::eigen` for the Eigen adapter. Include the backend and shared
operation headers explicitly, for example `<formal_gvins/linalg/backend/eigen.hpp>`
and `<formal_gvins/so3/so3.hpp>`. The base target
`formal_gvins::formal_gvins` has no Eigen or CMSIS-DSP dependency. These targets
are header-only; no compiled estimator library is provided at this stage.

CMSIS-DSP is optional and remains an external dependency. An embedding CMake
project may create its CMSIS-DSP target before adding this project as a
subdirectory, or provide an installed `CMSISDSP` CMake package. Configure:

```text
FORMAL_GVINS_BUILD_CMSIS_DSP_BACKEND=ON
FORMAL_GVINS_CMSIS_DSP_TARGET=<target providing CMSIS-DSP headers and library>
```

Then link `formal_gvins::cmsis_dsp`. Set
`FORMAL_GVINS_BUILD_EIGEN_BACKEND=OFF` for a CMSIS-only build. The embedding build
owns CPU/FPU/ABI flags and SIMD selection. Both adapters provide float and double
profiles; target timing, stack use and SIMD behavior need separate validation.
The tests retain the inherited fixed-size scope and do not establish MSCKF-sized
workspace or QR support.

## Documentation

See the [documentation index](doc/README.md),
[specification template](doc/spec/TEMPLATE.md) and
[coding style](doc/guides/coding-style.md).

## License

formal-gvins is licensed under [Apache-2.0](LICENSE).
Copyright (c) 2026, Sheng-Wen (Colin) Cheng.

Eigen and CMSIS-DSP remain external dependencies with their own licenses.
