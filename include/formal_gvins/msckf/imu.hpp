/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <formal_gvins/msckf/state.hpp>

namespace formal_gvins::msckf
{

// Local VINS coefficient domain: |omega| <= 10 rad/s, |a| <= 100 m/s^2, h <= 10 ms.
inline constexpr std::uint64_t maximum_interval_ns = 10'000'000;

template <typename Linalg> struct ImuSample
{
    std::uint64_t time_ns = 0;
    linalg::Vector<Linalg, 3U> specific_force;
    linalg::Vector<Linalg, 3U> angular_rate;
};

template <typename Linalg> struct ImuInterval
{
    std::uint64_t start_ns = 0;
    std::uint64_t end_ns = 0;
    linalg::Vector<Linalg, 3U> a;
    linalg::Vector<Linalg, 3U> omega;

    /** Requires end_ns > start_ns and an admitted interval. */
    typename Linalg::value_type duration() const
    {
        return static_cast<typename Linalg::value_type>(end_ns - start_ns) * typename Linalg::value_type{1e-9};
    }
};

template <typename Linalg> [[nodiscard]] Status validate_interval(ImuInterval<Linalg> const & input)
{
    using value_type = typename Linalg::value_type;
    if (input.end_ns <= input.start_ns || input.end_ns - input.start_ns > maximum_interval_ns)
    {
        return Status::out_of_range;
    }
    if (!linalg::all_finite(input.a) || !linalg::all_finite(input.omega))
    {
        return Status::non_finite_input;
    }
    if (!(linalg::squared_norm(input.a) <= value_type{10000}) ||
        !(linalg::squared_norm(input.omega) <= value_type{100}))
    {
        return Status::domain_error;
    }
    return Status::success;
}

/** Interpolate calibrated, bias-independent measurements within a supplied raw bracket.
 * Both samples and the original gap must be valid even at an exact endpoint.
 * Missing bracket selection and scheduling belong to the caller. Aliasing is allowed.
 */
template <typename Linalg>
[[nodiscard]] Status interpolate(ImuSample<Linalg> const & left, ImuSample<Linalg> const & right, std::uint64_t time_ns,
                                 ImuSample<Linalg> & output)
{
    using value_type = typename Linalg::value_type;
    if (right.time_ns <= left.time_ns || right.time_ns - left.time_ns > maximum_interval_ns || time_ns < left.time_ns ||
        time_ns > right.time_ns)
    {
        return Status::out_of_range;
    }
    if (!linalg::all_finite(left.specific_force) || !linalg::all_finite(right.specific_force) ||
        !linalg::all_finite(left.angular_rate) || !linalg::all_finite(right.angular_rate))
    {
        return Status::non_finite_input;
    }
    if (time_ns == left.time_ns)
    {
        output = left;
        return Status::success;
    }
    if (time_ns == right.time_ns)
    {
        output = right;
        return Status::success;
    }
    value_type const fraction =
        static_cast<value_type>(time_ns - left.time_ns) / static_cast<value_type>(right.time_ns - left.time_ns);
    ImuSample<Linalg> candidate;
    candidate.time_ns = time_ns;
    candidate.specific_force = left.specific_force * (value_type{1} - fraction) + right.specific_force * fraction;
    candidate.angular_rate = left.angular_rate * (value_type{1} - fraction) + right.angular_rate * fraction;
    if (!linalg::all_finite(candidate.specific_force) || !linalg::all_finite(candidate.angular_rate))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** Endpoint averaging with current bias means. Caller ensures no interior raw sample
 * and that any interpolated endpoints came from an admissible original bracket.
 * All failures preserve output; no noise density is scaled here.
 */
template <typename Linalg>
[[nodiscard]] Status prepare_interval(ImuSample<Linalg> const & left, ImuSample<Linalg> const & right,
                                      linalg::Vector<Linalg, 3U> const & b_a, linalg::Vector<Linalg, 3U> const & b_g,
                                      ImuInterval<Linalg> & output)
{
    using value_type = typename Linalg::value_type;
    if (right.time_ns <= left.time_ns || right.time_ns - left.time_ns > maximum_interval_ns)
    {
        return Status::out_of_range;
    }
    if (!linalg::all_finite(left.specific_force) || !linalg::all_finite(right.specific_force) ||
        !linalg::all_finite(left.angular_rate) || !linalg::all_finite(right.angular_rate) || !linalg::all_finite(b_a) ||
        !linalg::all_finite(b_g))
    {
        return Status::non_finite_input;
    }
    ImuInterval<Linalg> candidate;
    candidate.start_ns = left.time_ns;
    candidate.end_ns = right.time_ns;
    candidate.a = left.specific_force * value_type{0.5} + right.specific_force * value_type{0.5} - b_a;
    candidate.omega = left.angular_rate * value_type{0.5} + right.angular_rate * value_type{0.5} - b_g;
    Status const status = validate_interval(candidate);
    if (!succeeded(status))
    {
        return status == Status::non_finite_input ? Status::non_finite_result : status;
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_gvins::msckf */
