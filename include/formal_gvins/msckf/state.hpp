/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <formal_gvins/linalg/bounded.hpp>
#include <formal_gvins/so3/so3.hpp>

namespace formal_gvins::msckf
{

/** Numerical admission only; PSD and physical/frame priors remain caller premises. */
template <typename Scalar> struct Tolerances
{
    Scalar quaternion_squared_norm;
    Scalar symmetry;
};

template <typename Linalg> struct Pose
{
    linalg::Vector<Linalg, 3U> p_WB;
    so3::UnitQuaternion<Linalg> q_WB;
};

template <typename Linalg> struct ImuState
{
    Pose<Linalg> pose;
    linalg::Vector<Linalg, 3U> v_WB;
    linalg::Vector<Linalg, 3U> b_a;
    linalg::Vector<Linalg, 3U> b_g;
};

template <typename Linalg> struct FirstEstimate
{
    Pose<Linalg> pose;
    linalg::Vector<Linalg, 3U> v_WB;
};

template <typename Linalg> struct Clone
{
    Pose<Linalg> pose;
    Pose<Linalg> first;
    std::uint64_t id = 0;
    std::uint64_t time_ns = 0;
};

/** Error order (p,v,theta,ba,bg), followed by (p,theta) for each active clone.
 * MaxClones includes the temporary augmentation slot. Inactive slots are not read.
 * Public storage lets the owner construct a prior; validate before accepting it.
 */
template <typename Linalg, std::size_t MaxClones> struct State
{
    static_assert(MaxClones <= (std::numeric_limits<std::size_t>::max() - 15U) / 6U);
    using linalg_type = Linalg;
    using value_type = typename Linalg::value_type;
    static constexpr std::size_t clone_capacity = MaxClones;
    static constexpr std::size_t dimension_capacity = 15U + 6U * MaxClones;
    using covariance_type = linalg::BoundedMatrix<Linalg, dimension_capacity, dimension_capacity>;

    ImuState<Linalg> imu;
    FirstEstimate<Linalg> first;
    std::array<Clone<Linalg>, MaxClones> clones{};
    std::size_t clone_count = 0;
    std::uint64_t time_ns = 0;
    covariance_type P;

    State()
    {
        // The compile-time capacity always admits the initial IMU block.
        (void)P.resize(15U, 15U);
    }
    std::size_t dimension() const { return 15U + 6U * clone_count; }
    static constexpr std::size_t clone_offset(std::size_t index) { return 15U + 6U * index; }
};

template <typename Linalg>
[[nodiscard]] bool valid_tolerances(Tolerances<typename Linalg::value_type> const & tolerance)
{
    using math_type = typename Linalg::scalar_math_type;
    return math_type::is_finite(tolerance.quaternion_squared_norm) && math_type::is_finite(tolerance.symmetry) &&
           tolerance.quaternion_squared_norm >= 0 && tolerance.quaternion_squared_norm < 1 && tolerance.symmetry >= 0 &&
           tolerance.symmetry < 1;
}

template <typename Linalg>
[[nodiscard]] Status validate_pose(Pose<Linalg> const & pose, typename Linalg::value_type norm_tolerance)
{
    using math_type = typename Linalg::scalar_math_type;
    auto const q = pose.q_WB.coefficients();
    if (!linalg::all_finite(pose.p_WB) || !linalg::all_finite(q))
    {
        return Status::non_finite_input;
    }
    if (!math_type::is_finite(norm_tolerance) || norm_tolerance < 0 || norm_tolerance >= 1)
    {
        return Status::domain_error;
    }
    auto const residual = linalg::squared_norm(q) - typename Linalg::value_type{1};
    if (!math_type::is_finite(residual) || scalar::absolute<math_type>(residual) > norm_tolerance)
    {
        return Status::invalid_quaternion_norm;
    }
    return Status::success;
}

/** Infinity-norm symmetry policy; does not repair the matrix or establish PSD. */
template <typename Linalg, std::size_t Size>
[[nodiscard]] Status validate_covariance(linalg::BoundedMatrix<Linalg, Size, Size> const & matrix,
                                         typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    if (matrix.rows() == 0 || matrix.rows() != matrix.columns())
    {
        return Status::out_of_range;
    }
    if (!math_type::is_finite(tolerance) || tolerance < 0 || tolerance >= 1)
    {
        return Status::domain_error;
    }
    if (!linalg::all_finite(matrix))
    {
        return Status::non_finite_input;
    }
    value_type norm = 0;
    value_type asymmetry = 0;
    for (std::size_t row = 0; row < matrix.rows(); ++row)
    {
        value_type row_norm = 0;
        value_type row_asymmetry = 0;
        for (std::size_t column = 0; column < matrix.columns(); ++column)
        {
            row_norm += scalar::absolute<math_type>(matrix(row, column));
            row_asymmetry += scalar::absolute<math_type>(matrix(row, column) - matrix(column, row));
        }
        if (!math_type::is_finite(row_norm) || !math_type::is_finite(row_asymmetry))
        {
            return Status::non_finite_result;
        }
        if (row_norm > norm)
        {
            norm = row_norm;
        }
        if (row_asymmetry > asymmetry)
        {
            asymmetry = row_asymmetry;
        }
    }
    if (asymmetry > tolerance * (norm > 1 ? norm : value_type{1}))
    {
        return Status::domain_error;
    }
    return Status::success;
}

/** Caller has checked symmetry; mirror only the active upper triangle. */
template <typename Linalg, std::size_t Size> void mirror_upper(linalg::BoundedMatrix<Linalg, Size, Size> & matrix)
{
    for (std::size_t row = 0; row < matrix.rows(); ++row)
    {
        for (std::size_t column = 0; column < row; ++column)
        {
            matrix(row, column) = matrix(column, row);
        }
    }
}

/** Validate layout and numerical representation. This does not validate a physical prior. */
template <typename Linalg, std::size_t MaxClones>
[[nodiscard]] Status validate(State<Linalg, MaxClones> const & state,
                              Tolerances<typename Linalg::value_type> const & tolerance)
{
    if (!valid_tolerances<Linalg>(tolerance))
    {
        return Status::domain_error;
    }
    if (state.clone_count > MaxClones || state.P.rows() != state.dimension() || state.P.columns() != state.dimension())
    {
        return Status::out_of_range;
    }
    Status status = validate_pose(state.imu.pose, tolerance.quaternion_squared_norm);
    if (!succeeded(status))
    {
        return status;
    }
    status = validate_pose(state.first.pose, tolerance.quaternion_squared_norm);
    if (!succeeded(status))
    {
        return status;
    }
    if (!linalg::all_finite(state.imu.v_WB) || !linalg::all_finite(state.imu.b_a) ||
        !linalg::all_finite(state.imu.b_g) || !linalg::all_finite(state.first.v_WB))
    {
        return Status::non_finite_input;
    }
    for (std::size_t i = 0; i < state.clone_count; ++i)
    {
        auto const & clone = state.clones[i];
        if (clone.time_ns > state.time_ns || (i > 0 && clone.time_ns <= state.clones[i - 1].time_ns))
        {
            return Status::domain_error;
        }
        for (std::size_t j = 0; j < i; ++j)
        {
            if (clone.id == state.clones[j].id)
            {
                return Status::domain_error;
            }
        }
        status = validate_pose(clone.pose, tolerance.quaternion_squared_norm);
        if (!succeeded(status))
        {
            return status;
        }
        status = validate_pose(clone.first, tolerance.quaternion_squared_norm);
        if (!succeeded(status))
        {
            return status;
        }
    }
    return validate_covariance(state.P, tolerance.symmetry);
}

/** Accept a fully specified candidate without changing the destination on failure.
 * Input/output aliasing is allowed. The caller supplies PSD and frame/time priors.
 */
template <typename Linalg, std::size_t MaxClones>
[[nodiscard]] Status assign(State<Linalg, MaxClones> const & input,
                            Tolerances<typename Linalg::value_type> const & tolerance,
                            State<Linalg, MaxClones> & output)
{
    Status const status = validate(input, tolerance);
    if (!succeeded(status))
    {
        return status;
    }
    output = input;
    mirror_upper(output.P);
    return Status::success;
}

} /* end namespace formal_gvins::msckf */
