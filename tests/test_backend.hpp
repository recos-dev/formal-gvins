/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <iostream>

#if defined(FORMAL_GVINS_TEST_CMSIS_DSP)
#include <formal_gvins/linalg/backend/cmsis_dsp.hpp>
#else
#include <formal_gvins/linalg/backend/eigen.hpp>
#endif

namespace formal_gvins::test
{

#if defined(FORMAL_GVINS_TEST_CMSIS_DSP)
template <typename Scalar> using Backend = linalg::CmsisDspBackend<Scalar>;
#else
template <typename Scalar> using Backend = linalg::EigenBackend<Scalar>;
#endif

inline void configure_backend_test()
{
#if defined(FORMAL_GVINS_TEST_CMSIS_DSP)
    std::cout << "Backend: CMSIS-DSP\n";
#else
    Eigen::internal::set_is_malloc_allowed(false);
    std::cout << "Backend: Eigen (runtime allocation guard enabled)\n";
#endif
}

} /* end namespace formal_gvins::test */
