/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <formal_gvins/msckf/state.hpp>
#include <formal_gvins/linalg/cholesky.hpp>

namespace formal_gvins::msckf
{

/** Explicit bounded storage for one uncompressed update. MaxRows is independent
 * of clone capacity; no hidden assumption that measurement count <= state size.
 * Scratch must be disjoint from state and all measurement inputs. Its contents
 * are unspecified after failure. Solver-local bounded temporaries are additional.
 */
template <typename Linalg, std::size_t MaxClones, std::size_t MaxRows> struct CorrectionWorkspace
{
    using state_type = State<Linalg, MaxClones>;
    static constexpr std::size_t dimension = state_type::dimension_capacity;
    state_type candidate;
    linalg::BoundedMatrix<Linalg, MaxRows, MaxRows> V;
    linalg::BoundedMatrix<Linalg, MaxRows, MaxRows> S;
    linalg::BoundedMatrix<Linalg, dimension, MaxRows> PHt;
    linalg::BoundedMatrix<Linalg, dimension, MaxRows> K;
    linalg::BoundedMatrix<Linalg, dimension, MaxRows> KV;
    typename state_type::covariance_type A;
    typename state_type::covariance_type AP;
    linalg::Vector<Linalg, dimension> error;
};

namespace detail
{
template <typename Linalg>
Status inject_pose(Pose<Linalg> & pose, linalg::Vector<Linalg, 3U> const & position_error,
                   linalg::Vector<Linalg, 3U> const & angle_error, typename Linalg::value_type maximum_angle)
{
    using value_type = typename Linalg::value_type;
    if (!(linalg::squared_norm(angle_error) <= maximum_angle * maximum_angle))
    {
        return Status::domain_error;
    }
    so3::UnitQuaternion<Linalg> increment;
    Status status = so3::exp(angle_error, value_type{0.5}, increment);
    if (!succeeded(status))
    {
        return status;
    }
    status = so3::compose_normalized(pose.q_WB, increment, value_type{0.5}, pose.q_WB);
    if (!succeeded(status))
    {
        return status;
    }
    pose.p_WB = pose.p_WB + position_error;
    return linalg::all_finite(pose.p_WB) ? Status::success : Status::non_finite_result;
}
} /* end namespace detail */

/** Generic correction of supplied r/H/V; no feature geometry, gate or consumption ledger.
 * H uses the selected error/FEJ coordinates, V must be SPD and P must be PSD.
 * Empty measurements are rejected; callers skip absent constraints. Maximum angle
 * must be in [0,0.2] rad under the current Draft profile. No clipping or Jr reset.
 * All means and Joseph covariance publish together; first estimates stay fixed.
 */
template <typename Linalg, std::size_t MaxClones, std::size_t MaxRows>
[[nodiscard]] Status
correct(State<Linalg, MaxClones> & state, linalg::BoundedMatrix<Linalg, MaxRows, 1U> const & residual,
        linalg::BoundedMatrix<Linalg, MaxRows, State<Linalg, MaxClones>::dimension_capacity> const & H,
        linalg::BoundedMatrix<Linalg, MaxRows, MaxRows> const & V,
        Tolerances<typename Linalg::value_type> const & tolerance, typename Linalg::value_type maximum_angle,
        CorrectionWorkspace<Linalg, MaxClones, MaxRows> & work)
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    if (&state == &work.candidate)
    {
        return Status::domain_error;
    }
    Status status = validate(state, tolerance);
    if (!succeeded(status))
    {
        return status;
    }
    if (!math_type::is_finite(maximum_angle) || maximum_angle < 0 || maximum_angle > value_type{0.2})
    {
        return Status::domain_error;
    }
    std::size_t const n = state.dimension();
    std::size_t const m = H.rows();
    if (m == 0 || H.columns() != n || residual.rows() != m || residual.columns() != 1 || V.rows() != m ||
        V.columns() != m)
    {
        return Status::out_of_range;
    }
    if (!linalg::all_finite(H) || !linalg::all_finite(residual))
    {
        return Status::non_finite_input;
    }
    status = validate_covariance(V, tolerance.symmetry);
    if (!succeeded(status))
    {
        return status;
    }
    work.V = V;
    mirror_upper(work.V);
    // Validate V independently: an indefinite V can still produce an SPD S.
    status = linalg::cholesky(work.V, work.S);
    if (!succeeded(status))
    {
        return status;
    }
    work.candidate = state;
    mirror_upper(work.candidate.P);
    (void)work.PHt.resize(n, m);
    for (std::size_t i = 0; i < n; ++i)
    {
        for (std::size_t j = 0; j < m; ++j)
        {
            value_type sum = 0;
            for (std::size_t k = 0; k < n; ++k)
            {
                sum += work.candidate.P(i, k) * H(j, k);
            }
            work.PHt(i, j) = sum;
        }
    }
    for (std::size_t i = 0; i < m; ++i)
    {
        for (std::size_t j = 0; j < m; ++j)
        {
            value_type sum = work.V(i, j);
            for (std::size_t k = 0; k < n; ++k)
            {
                sum += H(i, k) * work.PHt(k, j);
            }
            work.S(i, j) = sum;
        }
    }
    status = validate_covariance(work.S, tolerance.symmetry);
    if (!succeeded(status))
    {
        return status == Status::non_finite_input ? Status::non_finite_result : status;
    }
    mirror_upper(work.S);
    if (!linalg::all_finite(work.PHt))
    {
        return Status::non_finite_result;
    }
    status = linalg::right_solve_spd(work.PHt, work.S, work.K);
    if (!succeeded(status))
    {
        return status;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        value_type sum = 0;
        for (std::size_t j = 0; j < m; ++j)
        {
            sum += work.K(i, j) * residual(j, 0);
        }
        if (!math_type::is_finite(sum))
        {
            return Status::non_finite_result;
        }
        work.error(i) = sum;
    }
    auto dp = work.error.template segment<0, 3>();
    auto dtheta = work.error.template segment<6, 3>();
    status = detail::inject_pose(work.candidate.imu.pose, dp, dtheta, maximum_angle);
    if (!succeeded(status))
    {
        return status;
    }
    work.candidate.imu.v_WB = state.imu.v_WB + work.error.template segment<3, 3>();
    work.candidate.imu.b_a = state.imu.b_a + work.error.template segment<9, 3>();
    work.candidate.imu.b_g = state.imu.b_g + work.error.template segment<12, 3>();
    for (std::size_t c = 0; c < state.clone_count; ++c)
    {
        std::size_t const offset = state.clone_offset(c);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            dp(axis) = work.error(offset + axis);
            dtheta(axis) = work.error(offset + 3 + axis);
        }
        status = detail::inject_pose(work.candidate.clones[c].pose, dp, dtheta, maximum_angle);
        if (!succeeded(status))
        {
            return status;
        }
    }
    // Explicit work matrices avoid D-by-D automatic result temporaries.
    (void)work.A.resize(n, n);
    (void)work.AP.resize(n, n);
    (void)work.KV.resize(n, m);
    for (std::size_t i = 0; i < n; ++i)
    {
        for (std::size_t j = 0; j < n; ++j)
        {
            value_type sum = 0;
            for (std::size_t k = 0; k < m; ++k)
            {
                sum += work.K(i, k) * H(k, j);
            }
            work.A(i, j) = (i == j ? value_type{1} : value_type{0}) - sum;
        }
        for (std::size_t j = 0; j < m; ++j)
        {
            value_type sum = 0;
            for (std::size_t k = 0; k < m; ++k)
            {
                sum += work.K(i, k) * work.V(k, j);
            }
            work.KV(i, j) = sum;
        }
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        for (std::size_t j = 0; j < n; ++j)
        {
            value_type sum = 0;
            for (std::size_t k = 0; k < n; ++k)
            {
                sum += work.A(i, k) * work.candidate.P(k, j);
            }
            work.AP(i, j) = sum;
        }
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        for (std::size_t j = 0; j < n; ++j)
        {
            value_type sum = 0;
            for (std::size_t k = 0; k < n; ++k)
            {
                sum += work.AP(i, k) * work.A(j, k);
            }
            for (std::size_t k = 0; k < m; ++k)
            {
                sum += work.KV(i, k) * work.K(j, k);
            }
            work.candidate.P(i, j) = sum;
        }
    }
    status = validate(work.candidate, tolerance);
    if (!succeeded(status))
    {
        return status == Status::non_finite_input ? Status::non_finite_result : status;
    }
    mirror_upper(work.candidate.P);
    state = work.candidate;
    return Status::success;
}

} /* end namespace formal_gvins::msckf */
