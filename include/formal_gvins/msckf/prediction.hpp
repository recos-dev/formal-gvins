/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <formal_gvins/msckf/imu.hpp>

namespace formal_gvins::msckf
{

template <typename Linalg> struct RotationIntegrals
{
    linalg::Matrix<Linalg, 3U, 3U> A1;
    linalg::Matrix<Linalg, 3U, 3U> A2;
};

namespace detail
{
/** Requires the admitted interval domain. Gamma tails are specified in local-vins.md. */
template <typename Linalg> RotationIntegrals<Linalg> rotation_integrals(ImuInterval<Linalg> const & input)
{
    using Value = typename Linalg::value_type;
    using Matrix = linalg::Matrix<Linalg, 3U, 3U>;
    Value const h = input.duration();
    Matrix const W = so3::hat(input.omega * h);
    Matrix term1 = Matrix::identity();
    Matrix term2 = term1 * Value{0.5};
    Matrix gamma1 = term1;
    Matrix gamma2 = term2;
    for (std::size_t order = 1; order <= 10; ++order)
    {
        term1 = (term1 * W) / static_cast<Value>(order + 1);
        term2 = (term2 * W) / static_cast<Value>(order + 2);
        gamma1 = gamma1 + term1;
        gamma2 = gamma2 + term2;
    }
    return {gamma1 * h, gamma2 * (h * h)};
}

/** Inputs already validated by the owning public transition. */
template <typename Linalg>
Status integrate_mean(ImuState<Linalg> const & input, ImuInterval<Linalg> const & interval,
                      linalg::Vector<Linalg, 3U> const & gravity, RotationIntegrals<Linalg> const & integrals,
                      ImuState<Linalg> & output)
{
    using Value = typename Linalg::value_type;
    constexpr Value minimum_quaternion_norm = Value{0.5};
    Value const h = interval.duration();
    auto const R = so3::to_rotation_matrix(input.pose.q_WB);
    so3::UnitQuaternion<Linalg> increment;
    Status status = so3::exp(interval.omega * h, minimum_quaternion_norm, increment);
    if (!succeeded(status))
    {
        return status;
    }
    auto candidate = input;
    status = so3::compose_normalized(input.pose.q_WB, increment, minimum_quaternion_norm, candidate.pose.q_WB);
    if (!succeeded(status))
    {
        return status;
    }
    candidate.pose.p_WB =
        input.pose.p_WB + input.v_WB * h + R * integrals.A2 * interval.a + gravity * (h * h * Value{0.5});
    candidate.v_WB = input.v_WB + R * integrals.A1 * interval.a + gravity * h;
    if (!linalg::all_finite(candidate.pose.p_WB) || !linalg::all_finite(candidate.v_WB))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}
} /* end namespace detail */

/** Analytical held-body-input mean (M-PROPAGATE). Independent of covariance.
 * Current biases remain unchanged; interval inputs must have been prepared using them.
 * Failure preserves output; input/output aliases are allowed.
 */
template <typename Linalg>
[[nodiscard]] Status predict_nominal(ImuState<Linalg> const & input, ImuInterval<Linalg> const & interval,
                                     linalg::Vector<Linalg, 3U> const & gravity,
                                     Tolerances<typename Linalg::value_type> const & tolerance,
                                     ImuState<Linalg> & output)
{
    if (!valid_tolerances<Linalg>(tolerance))
    {
        return Status::domain_error;
    }
    Status status = validate_interval(interval);
    if (!succeeded(status))
    {
        return status;
    }
    status = validate_pose(input.pose, tolerance.quaternion_squared_norm);
    if (!succeeded(status))
    {
        return status;
    }
    if (!linalg::all_finite(input.v_WB) || !linalg::all_finite(input.b_a) || !linalg::all_finite(input.b_g) ||
        !linalg::all_finite(gravity))
    {
        return Status::non_finite_input;
    }
    ImuState<Linalg> candidate;
    status = detail::integrate_mean(input, interval, gravity, detail::rotation_integrals(interval), candidate);
    if (!succeeded(status))
    {
        return status;
    }
    status = validate_pose(candidate.pose, tolerance.quaternion_squared_norm);
    if (!succeeded(status))
    {
        return status;
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_gvins::msckf */
