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

/** Append the current IMU pose and its separately cached first estimate.
 * No independent clone noise. The augmented covariance is allowed to be singular.
 * The caller owns session-wide identity and observation association. A synchronized
 * multi-camera exposure reuses the existing clone rather than appending twice.
 * Scratch must be distinct from state; failure leaves state unchanged.
 */
template <typename Linalg, std::size_t MaxClones>
[[nodiscard]] Status augment_clone(State<Linalg, MaxClones> & state, std::uint64_t id,
                                   Tolerances<typename Linalg::value_type> const & tolerance,
                                   State<Linalg, MaxClones> & candidate)
{
    if (&candidate == &state)
    {
        return Status::domain_error;
    }
    Status const status = validate(state, tolerance);
    if (!succeeded(status))
    {
        return status;
    }
    if (state.clone_count == MaxClones)
    {
        return Status::out_of_range;
    }
    if (state.clone_count > 0 && state.clones[state.clone_count - 1].time_ns >= state.time_ns)
    {
        return Status::domain_error;
    }
    for (std::size_t i = 0; i < state.clone_count; ++i)
    {
        if (state.clones[i].id == id)
        {
            return Status::domain_error;
        }
    }
    candidate = state;
    candidate.clones[state.clone_count] = {state.imu.pose, state.first.pose, id, state.time_ns};
    ++candidate.clone_count;
    std::size_t const old_dimension = state.dimension();
    (void)candidate.P.resize(candidate.dimension(), candidate.dimension());
    constexpr std::array<std::size_t, 6> pose_indices{0, 1, 2, 6, 7, 8};
    // Canonicalize the source before duplicating coordinates. Near-symmetric input
    // must not give different copies of the same random variable.
    for (std::size_t i = 0; i < candidate.dimension(); ++i)
    {
        std::size_t const source_i = i < old_dimension ? i : pose_indices[i - old_dimension];
        for (std::size_t j = i; j < candidate.dimension(); ++j)
        {
            std::size_t const source_j = j < old_dimension ? j : pose_indices[j - old_dimension];
            auto const value = source_i <= source_j ? state.P(source_i, source_j) : state.P(source_j, source_i);
            candidate.P(i, j) = value;
            candidate.P(j, i) = value;
        }
    }
    state = candidate;
    return Status::success;
}

/** Select the retained covariance submatrix (E*P*E^T), not a Schur complement.
 * Caller must consume/retire observations using the removed clone before this
 * numerical operation. Remaining identities, order and first estimates are kept.
 */
template <typename Linalg, std::size_t MaxClones>
[[nodiscard]] Status remove_clone(State<Linalg, MaxClones> & state, std::size_t index,
                                  Tolerances<typename Linalg::value_type> const & tolerance,
                                  State<Linalg, MaxClones> & candidate)
{
    if (&candidate == &state)
    {
        return Status::domain_error;
    }
    Status const status = validate(state, tolerance);
    if (!succeeded(status))
    {
        return status;
    }
    if (index >= state.clone_count)
    {
        return Status::out_of_range;
    }
    candidate = state;
    --candidate.clone_count;
    for (std::size_t i = index; i < candidate.clone_count; ++i)
    {
        candidate.clones[i] = state.clones[i + 1];
    }
    candidate.clones[candidate.clone_count] = {};
    (void)candidate.P.resize(candidate.dimension(), candidate.dimension());
    std::size_t const removed_offset = state.clone_offset(index);
    for (std::size_t i = 0; i < candidate.dimension(); ++i)
    {
        std::size_t const source_i = i < removed_offset ? i : i + 6;
        for (std::size_t j = i; j < candidate.dimension(); ++j)
        {
            std::size_t const source_j = j < removed_offset ? j : j + 6;
            auto const value = state.P(source_i, source_j);
            candidate.P(i, j) = value;
            candidate.P(j, i) = value;
        }
    }
    state = candidate;
    return Status::success;
}

} /* end namespace formal_gvins::msckf */
