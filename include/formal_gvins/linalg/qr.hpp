/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <utility>
#include <formal_gvins/linalg/bounded.hpp>

namespace formal_gvins::linalg
{

template <typename Scalar> struct RankTolerance
{
    Scalar absolute = 0;
    Scalar relative = 0;
};

template <typename Linalg, std::size_t Rows, std::size_t Columns, std::size_t RightColumns> struct QrResult
{
    BoundedMatrix<Linalg, Rows, Columns> upper;
    BoundedMatrix<Linalg, Rows, RightColumns> transformed_rhs;
    std::array<std::size_t, Columns> permutation{};
    std::size_t rank = 0;
};

namespace detail
{

template <typename Math>
typename Math::value_type stable_hypot(typename Math::value_type a, typename Math::value_type b)
{
    a = Math::absolute(a);
    b = Math::absolute(b);
    if (a < b)
    {
        std::swap(a, b);
    }
    if (a == typename Math::value_type{0})
    {
        return a;
    }
    auto const ratio = b / a;
    return a * Math::sqrt(typename Math::value_type{1} + ratio * ratio);
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
typename Linalg::value_type column_norm(BoundedMatrix<Linalg, Rows, Columns> const & matrix, std::size_t start,
                                        std::size_t column)
{
    using Math = typename Linalg::scalar_math_type;
    typename Linalg::value_type result{0};
    for (std::size_t row = start; row < matrix.rows(); ++row)
    {
        result = stable_hypot<Math>(result, matrix(row, column));
    }
    return result;
}

} /* end namespace detail */

/**
 * Column-pivoted Givens QR: A[:,permutation] = Q*upper; transformed_rhs = Q^T*B.
 * Rank stops when the largest trailing column norm <= absolute + relative*scale,
 * where scale is the largest original column norm. Trailing rows are retained,
 * not silently zeroed/truncated. This pivot criterion is not a condition bound.
 *
 * Shape -> finite input -> tolerance domain -> finite result. Rank loss is a
 * successful factorization with a smaller reported rank. Failure preserves the
 * complete result, including aliases through its upper/transformed_rhs members.
 */
template <typename Linalg, std::size_t Rows, std::size_t Columns, std::size_t RightColumns>
[[nodiscard]] Status
qr(BoundedMatrix<Linalg, Rows, Columns> const & system, BoundedMatrix<Linalg, Rows, RightColumns> const & rhs,
   RankTolerance<typename Linalg::value_type> const & tolerance, QrResult<Linalg, Rows, Columns, RightColumns> & output)
{
    using Value = typename Linalg::value_type;
    using Math = typename Linalg::scalar_math_type;
    if (system.rows() == 0 || system.columns() == 0 || rhs.rows() != system.rows() || rhs.columns() == 0)
    {
        return Status::out_of_range;
    }
    if (!all_finite(system) || !all_finite(rhs) || !Math::is_finite(tolerance.absolute) ||
        !Math::is_finite(tolerance.relative))
    {
        return Status::non_finite_input;
    }
    if (tolerance.absolute < Value{0} || tolerance.relative < Value{0} || tolerance.relative >= Value{1})
    {
        return Status::domain_error;
    }

    QrResult<Linalg, Rows, Columns, RightColumns> candidate;
    candidate.upper = system;
    candidate.transformed_rhs = rhs;
    Value scale{0};
    for (std::size_t column = 0; column < system.columns(); ++column)
    {
        candidate.permutation[column] = column;
        Value const norm = detail::column_norm(system, 0, column);
        if (!Math::is_finite(norm))
        {
            return Status::non_finite_result;
        }
        if (norm > scale)
        {
            scale = norm;
        }
    }
    Value const threshold = tolerance.absolute + tolerance.relative * scale;
    if (!Math::is_finite(threshold))
    {
        return Status::non_finite_result;
    }
    auto & R = candidate.upper;
    auto & C = candidate.transformed_rhs;
    for (std::size_t k = 0; k < system.rows() && k < system.columns(); ++k)
    {
        std::size_t pivot = k;
        Value largest{0};
        for (std::size_t column = k; column < system.columns(); ++column)
        {
            Value const norm = detail::column_norm(R, k, column);
            if (!Math::is_finite(norm))
            {
                return Status::non_finite_result;
            }
            if (norm > largest)
            {
                largest = norm;
                pivot = column;
            }
        }
        if (largest <= threshold)
        {
            break;
        }
        if (pivot != k)
        {
            for (std::size_t row = 0; row < system.rows(); ++row)
            {
                std::swap(R(row, k), R(row, pivot));
            }
            std::swap(candidate.permutation[k], candidate.permutation[pivot]);
        }
        for (std::size_t row = k + 1; row < system.rows(); ++row)
        {
            if (R(row, k) == Value{0})
            {
                continue;
            }
            // Form c,s from scaled inputs: dividing by a rounded subnormal
            // norm can otherwise turn both coefficients into 1 instead of 1/sqrt(2).
            Value const magnitude_a = Math::absolute(R(k, k)), magnitude_b = Math::absolute(R(row, k));
            Value const scale = magnitude_a > magnitude_b ? magnitude_a : magnitude_b;
            Value const a_scaled = R(k, k) / scale, b_scaled = R(row, k) / scale;
            Value const length = Math::sqrt(a_scaled * a_scaled + b_scaled * b_scaled);
            Value const norm = scale * length;
            if (!Math::is_finite(norm))
            {
                return Status::non_finite_result;
            }
            Value const c = a_scaled / length;
            Value const s = b_scaled / length;
            R(k, k) = norm;
            R(row, k) = Value{0};
            for (std::size_t column = k + 1; column < system.columns(); ++column)
            {
                Value const a = R(k, column), b = R(row, column);
                R(k, column) = c * a + s * b;
                R(row, column) = -s * a + c * b;
            }
            for (std::size_t column = 0; column < rhs.columns(); ++column)
            {
                Value const a = C(k, column), b = C(row, column);
                C(k, column) = c * a + s * b;
                C(row, column) = -s * a + c * b;
            }
        }
        ++candidate.rank;
    }
    if (!all_finite(R) || !all_finite(C))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** Full-column-rank least squares; square input also provides a general solve. */
template <typename Linalg, std::size_t Rows, std::size_t Columns, std::size_t RightColumns>
[[nodiscard]] Status least_squares(BoundedMatrix<Linalg, Rows, Columns> const & system,
                                   BoundedMatrix<Linalg, Rows, RightColumns> const & rhs,
                                   RankTolerance<typename Linalg::value_type> const & tolerance,
                                   BoundedMatrix<Linalg, Columns, RightColumns> & output)
{
    if (system.rows() < system.columns())
    {
        return Status::out_of_range;
    }
    QrResult<Linalg, Rows, Columns, RightColumns> reduced;
    Status const status = qr(system, rhs, tolerance, reduced);
    if (!succeeded(status))
    {
        return status;
    }
    if (reduced.rank != system.columns())
    {
        return Status::ill_conditioned;
    }
    BoundedMatrix<Linalg, Columns, RightColumns> candidate;
    Status const resized = candidate.resize(system.columns(), rhs.columns());
    if (!succeeded(resized))
    {
        return resized;
    }
    for (std::size_t column = 0; column < rhs.columns(); ++column)
    {
        for (std::size_t remaining = system.columns(); remaining > 0; --remaining)
        {
            std::size_t const row = remaining - 1;
            auto value = reduced.transformed_rhs(row, column);
            for (std::size_t next = row + 1; next < system.columns(); ++next)
            {
                value -= reduced.upper(row, next) * candidate(reduced.permutation[next], column);
            }
            candidate(reduced.permutation[row], column) = value / reduced.upper(row, row);
        }
    }
    if (!all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** Full-column-rank left-nullspace action, without explicitly materializing Q. */
template <typename Linalg, std::size_t Rows, std::size_t Columns, std::size_t RightColumns>
[[nodiscard]] Status project_left_nullspace(BoundedMatrix<Linalg, Rows, Columns> const & system,
                                            BoundedMatrix<Linalg, Rows, RightColumns> const & rhs,
                                            RankTolerance<typename Linalg::value_type> const & tolerance,
                                            BoundedMatrix<Linalg, Rows, RightColumns> & output)
{
    if (system.rows() <= system.columns())
    {
        return Status::out_of_range;
    }
    QrResult<Linalg, Rows, Columns, RightColumns> reduced;
    Status const status = qr(system, rhs, tolerance, reduced);
    if (!succeeded(status))
    {
        return status;
    }
    if (reduced.rank != system.columns())
    {
        return Status::ill_conditioned;
    }
    return block(reduced.transformed_rhs, reduced.rank, 0, system.rows() - reduced.rank, rhs.columns(), output);
}

} /* end namespace formal_gvins::linalg */
