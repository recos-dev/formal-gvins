/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <array>
#include <formal_gvins/linalg/operations.hpp>

namespace formal_gvins::linalg
{

/** Fixed storage, runtime active extents. Inactive coefficients are never operands. */
template <typename Linalg, std::size_t MaxRows, std::size_t MaxColumns> class BoundedMatrix
{
public:
    using value_type = typename Linalg::value_type;
    using linalg_type = Linalg;
    static constexpr std::size_t row_capacity = MaxRows;
    static constexpr std::size_t column_capacity = MaxColumns;

    std::size_t rows() const { return m_rows; }
    std::size_t columns() const { return m_columns; }

    /** Reset the active rectangle to zero. Invalid extents preserve the object. */
    [[nodiscard]] Status resize(std::size_t rows, std::size_t columns)
    {
        if (rows > MaxRows || columns > MaxColumns)
        {
            return Status::out_of_range;
        }
        for (std::size_t row = 0; row < rows; ++row)
        {
            for (std::size_t column = 0; column < columns; ++column)
            {
                m_storage(row, column) = value_type{0};
            }
        }
        m_rows = rows;
        m_columns = columns;
        return Status::success;
    }

    /** Caller establishes row < rows(), column < columns(), as for fixed Matrix. */
    value_type operator()(std::size_t row, std::size_t column) const { return m_storage(row, column); }
    value_type & operator()(std::size_t row, std::size_t column) { return m_storage(row, column); }

    static BoundedMatrix from_fixed(Matrix<Linalg, MaxRows, MaxColumns> const & source)
    {
        BoundedMatrix result;
        result.m_storage = source;
        result.m_rows = MaxRows;
        result.m_columns = MaxColumns;
        return result;
    }

private:
    Matrix<Linalg, MaxRows, MaxColumns> m_storage;
    std::size_t m_rows = 0;
    std::size_t m_columns = 0;
};

template <typename Linalg, std::size_t Rows, std::size_t Columns>
bool all_finite(BoundedMatrix<Linalg, Rows, Columns> const & matrix)
{
    using Math = typename Linalg::scalar_math_type;
    for (std::size_t row = 0; row < matrix.rows(); ++row)
    {
        for (std::size_t column = 0; column < matrix.columns(); ++column)
        {
            if (!scalar::is_finite<Math>(matrix(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

/** Checked copies preserve coefficient values, including non-finite values. */
template <typename Linalg, std::size_t Rows, std::size_t Columns, std::size_t OutRows, std::size_t OutColumns>
[[nodiscard]] Status block(BoundedMatrix<Linalg, Rows, Columns> const & source, std::size_t start_row,
                           std::size_t start_column, std::size_t rows, std::size_t columns,
                           BoundedMatrix<Linalg, OutRows, OutColumns> & output)
{
    if (start_row > source.rows() || rows > source.rows() - start_row || start_column > source.columns() ||
        columns > source.columns() - start_column || rows > OutRows || columns > OutColumns)
    {
        return Status::out_of_range;
    }
    BoundedMatrix<Linalg, OutRows, OutColumns> candidate;
    Status const status = candidate.resize(rows, columns);
    if (!succeeded(status))
    {
        return status;
    }
    for (std::size_t row = 0; row < rows; ++row)
    {
        for (std::size_t column = 0; column < columns; ++column)
        {
            candidate(row, column) = source(start_row + row, start_column + column);
        }
    }
    output = candidate;
    return Status::success;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns, std::size_t OutRows, std::size_t OutColumns>
[[nodiscard]] Status set_block(BoundedMatrix<Linalg, Rows, Columns> const & source, std::size_t start_row,
                               std::size_t start_column, BoundedMatrix<Linalg, OutRows, OutColumns> & output)
{
    if (start_row > output.rows() || source.rows() > output.rows() - start_row || start_column > output.columns() ||
        source.columns() > output.columns() - start_column)
    {
        return Status::out_of_range;
    }
    auto candidate = output;
    for (std::size_t row = 0; row < source.rows(); ++row)
    {
        for (std::size_t column = 0; column < source.columns(); ++column)
        {
            candidate(start_row + row, start_column + column) = source(row, column);
        }
    }
    output = candidate;
    return Status::success;
}

/** Indexed selection, e.g. the same surviving indices on both covariance axes. */
template <typename Linalg, std::size_t Rows, std::size_t Columns, std::size_t OutRows, std::size_t OutColumns>
[[nodiscard]] Status gather(BoundedMatrix<Linalg, Rows, Columns> const & source,
                            std::array<std::size_t, OutRows> const & row_indices, std::size_t rows,
                            std::array<std::size_t, OutColumns> const & column_indices, std::size_t columns,
                            BoundedMatrix<Linalg, OutRows, OutColumns> & output)
{
    if (rows > OutRows || columns > OutColumns)
    {
        return Status::out_of_range;
    }
    for (std::size_t row = 0; row < rows; ++row)
    {
        if (row_indices[row] >= source.rows())
        {
            return Status::out_of_range;
        }
    }
    for (std::size_t column = 0; column < columns; ++column)
    {
        if (column_indices[column] >= source.columns())
        {
            return Status::out_of_range;
        }
    }
    BoundedMatrix<Linalg, OutRows, OutColumns> candidate;
    Status const status = candidate.resize(rows, columns);
    if (!succeeded(status))
    {
        return status;
    }
    for (std::size_t row = 0; row < rows; ++row)
    {
        for (std::size_t column = 0; column < columns; ++column)
        {
            candidate(row, column) = source(row_indices[row], column_indices[column]);
        }
    }
    output = candidate;
    return Status::success;
}

/** Shape errors precede finite-input errors; all checked arithmetic is atomic and permits aliases. */
template <typename Linalg, std::size_t Rows, std::size_t Inner, std::size_t Columns>
[[nodiscard]] Status multiply(BoundedMatrix<Linalg, Rows, Inner> const & left,
                              BoundedMatrix<Linalg, Inner, Columns> const & right,
                              BoundedMatrix<Linalg, Rows, Columns> & output)
{
    if (left.columns() != right.rows())
    {
        return Status::out_of_range;
    }
    if (!all_finite(left) || !all_finite(right))
    {
        return Status::non_finite_input;
    }
    BoundedMatrix<Linalg, Rows, Columns> candidate;
    Status const status = candidate.resize(left.rows(), right.columns());
    if (!succeeded(status))
    {
        return status;
    }
    for (std::size_t row = 0; row < left.rows(); ++row)
    {
        for (std::size_t column = 0; column < right.columns(); ++column)
        {
            typename Linalg::value_type sum{0};
            for (std::size_t inner = 0; inner < left.columns(); ++inner)
            {
                sum += left(row, inner) * right(inner, column);
            }
            candidate(row, column) = sum;
        }
    }
    if (!all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
[[nodiscard]] Status add(BoundedMatrix<Linalg, Rows, Columns> const & left,
                         BoundedMatrix<Linalg, Rows, Columns> const & right,
                         BoundedMatrix<Linalg, Rows, Columns> & output)
{
    if (left.rows() != right.rows() || left.columns() != right.columns())
    {
        return Status::out_of_range;
    }
    if (!all_finite(left) || !all_finite(right))
    {
        return Status::non_finite_input;
    }
    auto candidate = left;
    for (std::size_t row = 0; row < left.rows(); ++row)
    {
        for (std::size_t column = 0; column < left.columns(); ++column)
        {
            candidate(row, column) += right(row, column);
        }
    }
    if (!all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
[[nodiscard]] Status transpose(BoundedMatrix<Linalg, Rows, Columns> const & input,
                               BoundedMatrix<Linalg, Columns, Rows> & output)
{
    BoundedMatrix<Linalg, Columns, Rows> candidate;
    Status const status = candidate.resize(input.columns(), input.rows());
    if (!succeeded(status))
    {
        return status;
    }
    for (std::size_t row = 0; row < input.rows(); ++row)
    {
        for (std::size_t column = 0; column < input.columns(); ++column)
        {
            candidate(column, row) = input(row, column);
        }
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_gvins::linalg */
