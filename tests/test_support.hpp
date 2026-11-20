/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cmath>
#include <iostream>
#include <string_view>

namespace formal_gvins::test
{

class Context
{
public:
    void expect(bool condition, std::string_view profile, std::string_view description)
    {
        if (!condition)
        {
            ++m_failures;
            std::cerr << "FAIL [" << profile << "]: " << description << '\n';
        }
    }

    int failures() const { return m_failures; }

private:
    int m_failures = 0;
};

template <typename T> bool near(T actual, T expected, T tolerance)
{
    using std::abs;
    return abs(actual - expected) <= tolerance;
}

} /* end namespace formal_gvins::test */
