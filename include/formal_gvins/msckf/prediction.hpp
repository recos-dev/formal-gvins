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
    using value_type = typename Linalg::value_type;
    using matrix_type = linalg::Matrix<Linalg, 3U, 3U>;
    value_type const h = input.duration();
    matrix_type const W = so3::hat(input.omega * h);
    matrix_type term1 = matrix_type::identity();
    matrix_type term2 = term1 * value_type{0.5};
    matrix_type gamma1 = term1;
    matrix_type gamma2 = term2;
    for (std::size_t order = 1; order <= 10; ++order)
    {
        term1 = (term1 * W) / static_cast<value_type>(order + 1);
        term2 = (term2 * W) / static_cast<value_type>(order + 2);
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
    using value_type = typename Linalg::value_type;
    constexpr value_type minimum_quaternion_norm = value_type{0.5};
    value_type const h = interval.duration();
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
        input.pose.p_WB + input.v_WB * h + R * integrals.A2 * interval.a + gravity * (h * h * value_type{0.5});
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

/** Explicit scratch ownership. Contents are unspecified after a failed operation.
 * Scratch must be disjoint from state and all inputs passed to predict.
 */
template <typename Linalg, std::size_t MaxClones> struct PredictionWorkspace
{
    using state_type = State<Linalg, MaxClones>;
    state_type candidate;
    linalg::Matrix<Linalg, 15U, 15U> Psi;
    linalg::Matrix<Linalg, 15U, 15U> Qd;
    linalg::Matrix<Linalg, 15U, 12U> Gd;
    linalg::BoundedMatrix<Linalg, 15U, state_type::dimension_capacity> cross;
};

namespace detail
{
template <typename Linalg>
Status translation_sensitivities(ImuInterval<Linalg> const & interval, linalg::Matrix<Linalg, 3U, 3U> & B1,
                                 linalg::Matrix<Linalg, 3U, 3U> & B2)
{
    using value_type = typename Linalg::value_type;
    using matrix_type = linalg::Matrix<Linalg, 3U, 3U>;
    // Fixed five-point Gauss-Legendre rule; targets the integrals in M-PROPAGATE.
    constexpr std::array<double, 5> nodes{-0.9061798459386640, -0.5384693101056831, 0, 0.5384693101056831,
                                          0.9061798459386640};
    constexpr std::array<double, 5> weights{0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
                                            0.4786286704993665, 0.2369268850561891};
    value_type const h = interval.duration();
    B1 = matrix_type{};
    B2 = matrix_type{};
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        value_type const time = h * (value_type{1} + static_cast<value_type>(nodes[i])) * value_type{0.5};
        value_type const weight = h * static_cast<value_type>(weights[i]) * value_type{0.5};
        so3::UnitQuaternion<Linalg> q;
        Status status = so3::exp(interval.omega * time, value_type{0.5}, q);
        if (!succeeded(status))
        {
            return status;
        }
        matrix_type Jr;
        status = so3::right_jacobian(interval.omega * time, Jr);
        if (!succeeded(status))
        {
            return status;
        }
        matrix_type const value = (so3::to_rotation_matrix(q) * so3::hat(interval.a) * Jr) * (time * weight);
        B1 = B1 + value;
        B2 = B2 + value * (h - time);
    }
    return Status::success;
}
} /* end namespace detail */

/** M-PROPAGATE / M-FEJ joint transition. Caller supplies symmetric PSD Qc in
 * (n_a,n_g,n_ba,n_bg) density units and a PSD prior. No Cholesky test on P/Qc.
 * Numerical symmetry checks precede upper-triangle mirroring; no diagonal jitter.
 * Mean, P, time and the new IMU first estimate publish atomically. Clones stay fixed.
 */
template <typename Linalg, std::size_t MaxClones>
[[nodiscard]] Status predict(State<Linalg, MaxClones> & state, ImuInterval<Linalg> const & interval,
                             linalg::Vector<Linalg, 3U> const & gravity, linalg::Matrix<Linalg, 12U, 12U> const & Qc,
                             Tolerances<typename Linalg::value_type> const & tolerance,
                             PredictionWorkspace<Linalg, MaxClones> & work)
{
    using value_type = typename Linalg::value_type;
    using matrix3_type = linalg::Matrix<Linalg, 3U, 3U>;
    using matrix15_type = linalg::Matrix<Linalg, 15U, 15U>;
    if (&state == &work.candidate)
    {
        return Status::domain_error;
    }
    Status status = validate(state, tolerance);
    if (!succeeded(status))
    {
        return status;
    }
    status = validate_interval(interval);
    if (!succeeded(status))
    {
        return status;
    }
    if (interval.start_ns != state.time_ns)
    {
        return Status::domain_error;
    }
    if (!linalg::all_finite(gravity))
    {
        return Status::non_finite_input;
    }
    auto density = linalg::BoundedMatrix<Linalg, 12U, 12U>::from_fixed(Qc);
    status = validate_covariance(density, tolerance.symmetry);
    if (!succeeded(status))
    {
        return status;
    }
    mirror_upper(density);
    linalg::Matrix<Linalg, 12U, 12U> rates;
    value_type const h = interval.duration();
    for (std::size_t i = 0; i < 12; ++i)
    {
        for (std::size_t j = 0; j < 12; ++j)
        {
            rates(i, j) = density(i, j) / h;
        }
    }
    if (!linalg::all_finite(rates))
    {
        return Status::non_finite_result;
    }
    work.candidate = state;
    mirror_upper(work.candidate.P);
    auto const integrals = detail::rotation_integrals(interval);
    status = detail::integrate_mean(state.imu, interval, gravity, integrals, work.candidate.imu);
    if (!succeeded(status))
    {
        return status;
    }
    status = validate_pose(work.candidate.imu.pose, tolerance.quaternion_squared_norm);
    if (!succeeded(status))
    {
        return status;
    }
    work.candidate.time_ns = interval.end_ns;
    work.candidate.first = {work.candidate.imu.pose, work.candidate.imu.v_WB};
    matrix3_type B1, B2;
    status = detail::translation_sensitivities(interval, B1, B2);
    if (!succeeded(status))
    {
        return status;
    }
    matrix3_type const R0 = so3::to_rotation_matrix(state.first.pose.q_WB);
    matrix3_type const Rnext = so3::to_rotation_matrix(work.candidate.first.pose.q_WB);
    matrix3_type const D = linalg::transpose(Rnext) * R0;
    auto const dp = work.candidate.first.pose.p_WB - state.first.pose.p_WB - state.first.v_WB * h -
                    gravity * (h * h * value_type{0.5});
    auto const dv = work.candidate.first.v_WB - state.first.v_WB - gravity * h;
    matrix3_type Jr;
    status = so3::right_jacobian(-interval.omega * h, Jr);
    if (!succeeded(status))
    {
        return status;
    }
    work.Psi = matrix15_type::identity();
    work.Psi.template set_block<0, 3>(matrix3_type::identity() * h);
    work.Psi.template set_block<0, 6>(-so3::hat(dp) * R0);
    work.Psi.template set_block<3, 6>(-so3::hat(dv) * R0);
    work.Psi.template set_block<6, 6>(D);
    work.Psi.template set_block<0, 9>(-R0 * integrals.A2);
    work.Psi.template set_block<3, 9>(-R0 * integrals.A1);
    work.Psi.template set_block<0, 12>(R0 * B2);
    work.Psi.template set_block<3, 12>(R0 * B1);
    work.Psi.template set_block<6, 12>((D * Jr) * (-h));
    work.Gd = {};
    work.Gd.template set_block<0, 0>(work.Psi.template block<0, 9, 9, 6>());
    work.Gd.template set_block<9, 6>(linalg::Matrix<Linalg, 6U, 6U>::identity() * h);
    work.Qd = work.Gd * rates * linalg::transpose(work.Gd);
    if (!linalg::all_finite(work.Psi) || !linalg::all_finite(work.Qd))
    {
        return Status::non_finite_result;
    }
    auto noise = linalg::BoundedMatrix<Linalg, 15U, 15U>::from_fixed(work.Qd);
    status = validate_covariance(noise, tolerance.symmetry);
    if (!succeeded(status))
    {
        return status == Status::non_finite_input ? Status::non_finite_result : status;
    }
    mirror_upper(noise);
    std::size_t const dimension = state.dimension();
    (void)work.cross.resize(15, dimension);
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < dimension; ++j)
        {
            value_type sum = 0;
            for (std::size_t k = 0; k < 15; ++k)
            {
                sum += work.Psi(i, k) * work.candidate.P(k, j);
            }
            work.cross(i, j) = sum;
        }
    }
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            value_type sum = noise(i, j);
            for (std::size_t k = 0; k < 15; ++k)
            {
                sum += work.cross(i, k) * work.Psi(j, k);
            }
            work.candidate.P(i, j) = sum;
        }
        for (std::size_t j = 15; j < dimension; ++j)
        {
            work.candidate.P(i, j) = work.cross(i, j);
            work.candidate.P(j, i) = work.cross(i, j);
        }
    }
    status = validate_covariance(work.candidate.P, tolerance.symmetry);
    if (!succeeded(status))
    {
        return status == Status::non_finite_input ? Status::non_finite_result : status;
    }
    mirror_upper(work.candidate.P);
    state = work.candidate;
    return Status::success;
}

} /* end namespace formal_gvins::msckf */
