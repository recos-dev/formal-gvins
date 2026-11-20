/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

namespace formal_gvins
{

enum class Status
{
    success,
    non_finite_input,
    domain_error,
    out_of_range,
    zero_or_unsafe_divisor,
    invalid_quaternion_norm,
    not_positive_definite,
    ill_conditioned,
    non_finite_result,
};

constexpr bool succeeded(Status status) { return status == Status::success; }

} /* end namespace formal_gvins */
