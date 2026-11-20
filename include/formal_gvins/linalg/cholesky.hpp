/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <formal_gvins/linalg/bounded.hpp>

namespace formal_gvins::linalg
{

/**
 * A=L*L^T for nonempty, finite, exactly symmetric SPD A; upper L entries are zero.
 * Shape errors precede finite-input errors, then symmetry/positive-pivot checks.
 * No conditioning cutoff. Non-finite arithmetic is non_finite_result.
 * All operations here preserve outputs on failure and permit input/output aliases.
 */
template <typename Linalg, std::size_t Size>
[[nodiscard]] Status cholesky(BoundedMatrix<Linalg, Size, Size> const & system,
                              BoundedMatrix<Linalg, Size, Size> & output)
{
    using Value = typename Linalg::value_type;
    using Math = typename Linalg::scalar_math_type;
    if (system.rows() == 0 || system.rows() != system.columns())
    {
        return Status::out_of_range;
    }
    if (!all_finite(system))
    {
        return Status::non_finite_input;
    }
    for (std::size_t row = 0; row < system.rows(); ++row)
    {
        for (std::size_t column = 0; column < row; ++column)
        {
            if (system(row, column) != system(column, row))
            {
                return Status::not_positive_definite;
            }
        }
    }
    BoundedMatrix<Linalg, Size, Size> candidate;
    Status const status = candidate.resize(system.rows(), system.columns());
    if (!succeeded(status))
    {
        return status;
    }
    for (std::size_t row = 0; row < system.rows(); ++row)
    {
        for (std::size_t column = 0; column <= row; ++column)
        {
            Value value = system(row, column);
            for (std::size_t inner = 0; inner < column; ++inner)
            {
                value -= candidate(row, inner) * candidate(column, inner);
            }
            if (!Math::is_finite(value))
            {
                return Status::non_finite_result;
            }
            if (row == column)
            {
                if (!(value > Value{0}))
                {
                    return Status::not_positive_definite;
                }
                candidate(row, column) = Math::sqrt(value);
            }
            else
            {
                candidate(row, column) = value / candidate(column, column);
            }
            if (!Math::is_finite(candidate(row, column)))
            {
                return Status::non_finite_result;
            }
        }
    }
    output = candidate;
    return Status::success;
}

/** Solve lower*X=B. Nonzero upper entries are domain_error; zero diagonal is zero_or_unsafe_divisor. */
template <typename Linalg, std::size_t Size, std::size_t Columns>
[[nodiscard]] Status solve_lower(BoundedMatrix<Linalg, Size, Size> const & lower,
                                 BoundedMatrix<Linalg, Size, Columns> const & rhs,
                                 BoundedMatrix<Linalg, Size, Columns> & output)
{
    using Value = typename Linalg::value_type;
    if (lower.rows() == 0 || lower.rows() != lower.columns() || rhs.rows() != lower.rows() || rhs.columns() == 0)
    {
        return Status::out_of_range;
    }
    if (!all_finite(lower) || !all_finite(rhs))
    {
        return Status::non_finite_input;
    }
    for (std::size_t row = 0; row < lower.rows(); ++row)
    {
        for (std::size_t column = row + 1; column < lower.columns(); ++column)
        {
            if (lower(row, column) != Value{0})
            {
                return Status::domain_error;
            }
        }
    }
    for (std::size_t row = 0; row < lower.rows(); ++row)
    {
        if (lower(row, row) == Value{0})
        {
            return Status::zero_or_unsafe_divisor;
        }
    }
    auto candidate = rhs;
    for (std::size_t row = 0; row < lower.rows(); ++row)
    {
        for (std::size_t column = 0; column < rhs.columns(); ++column)
        {
            auto value = rhs(row, column);
            for (std::size_t inner = 0; inner < row; ++inner)
            {
                value -= lower(row, inner) * candidate(inner, column);
            }
            candidate(row, column) = value / lower(row, row);
        }
    }
    if (!all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** Whiten all supplied RHS columns using covariance=L*L^T and output=L^-1*rhs. */
template <typename Linalg, std::size_t Size, std::size_t Columns>
[[nodiscard]] Status whiten(BoundedMatrix<Linalg, Size, Size> const & covariance,
                            BoundedMatrix<Linalg, Size, Columns> const & rhs,
                            BoundedMatrix<Linalg, Size, Columns> & output)
{
    if (covariance.rows() == 0 || covariance.rows() != covariance.columns() || rhs.rows() != covariance.rows() ||
        rhs.columns() == 0)
    {
        return Status::out_of_range;
    }
    if (!all_finite(covariance) || !all_finite(rhs))
    {
        return Status::non_finite_input;
    }
    BoundedMatrix<Linalg, Size, Size> factor;
    Status const status = cholesky(covariance, factor);
    if (!succeeded(status))
    {
        return status;
    }
    return solve_lower(factor, rhs, output);
}

/** Active-size SPD solve; same symmetry/positive-pivot policy as fixed solve_spd. */
template <typename Linalg, std::size_t Size, std::size_t Columns>
[[nodiscard]] Status solve_spd(BoundedMatrix<Linalg, Size, Size> const & system,
                               BoundedMatrix<Linalg, Size, Columns> const & rhs,
                               BoundedMatrix<Linalg, Size, Columns> & output)
{
    if (system.rows() == 0 || system.rows() != system.columns() || rhs.rows() != system.rows() || rhs.columns() == 0)
    {
        return Status::out_of_range;
    }
    if (!all_finite(system) || !all_finite(rhs))
    {
        return Status::non_finite_input;
    }
    BoundedMatrix<Linalg, Size, Size> factor;
    Status status = cholesky(system, factor);
    if (!succeeded(status))
    {
        return status;
    }
    BoundedMatrix<Linalg, Size, Columns> candidate;
    status = solve_lower(factor, rhs, candidate);
    if (!succeeded(status))
    {
        return status;
    }
    for (std::size_t remaining = system.rows(); remaining > 0; --remaining)
    {
        std::size_t const row = remaining - 1;
        for (std::size_t column = 0; column < rhs.columns(); ++column)
        {
            auto value = candidate(row, column);
            for (std::size_t inner = row + 1; inner < system.rows(); ++inner)
            {
                value -= factor(inner, row) * candidate(inner, column);
            }
            candidate(row, column) = value / factor(row, row);
        }
    }
    if (!all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

template <typename Linalg, std::size_t Rows, std::size_t Size>
[[nodiscard]] Status right_solve_spd(BoundedMatrix<Linalg, Rows, Size> const & rhs,
                                     BoundedMatrix<Linalg, Size, Size> const & system,
                                     BoundedMatrix<Linalg, Rows, Size> & output)
{
    BoundedMatrix<Linalg, Size, Rows> transposed;
    Status status = transpose(rhs, transposed);
    if (!succeeded(status))
    {
        return status;
    }
    status = solve_spd(system, transposed, transposed);
    if (!succeeded(status))
    {
        return status;
    }
    return transpose(transposed, output);
}

} /* end namespace formal_gvins::linalg */
