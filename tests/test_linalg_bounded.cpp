/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <formal_gvins/linalg/linalg.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"
#include <algorithm>
#include <bit>
#include <cstdint>
#include <limits>
#if !defined(FORMAL_GVINS_TEST_CMSIS_DSP)
#include <Eigen/QR>
#endif

namespace
{
using formal_gvins::Status;
using namespace formal_gvins::linalg;
using Test = formal_gvins::test::Context;

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    using Bytes = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    if (a.rows() != b.rows() || a.columns() != b.columns())
    {
        return false;
    }
    for (std::size_t i = 0; i < a.rows(); ++i)
    {
        for (std::size_t j = 0; j < a.columns(); ++j)
        {
            if (std::bit_cast<Bytes>(a(i, j)) != std::bit_cast<Bytes>(b(i, j)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename Scalar> void run(Test & test)
{
    using Backend = formal_gvins::test::Backend<Scalar>;
    using Square = BoundedMatrix<Backend, 4, 4>;
    auto const eps = Scalar{500} * std::numeric_limits<Scalar>::epsilon();
    auto ok = [&](Status status, char const * description)
    { test.expect(status == Status::success, "bounded", description); };
    auto close = [&](Scalar a, Scalar b, char const * description)
    { test.expect(formal_gvins::test::near(a, b, eps * (Scalar{1} + std::abs(b))), "residual", description); };
    Square A, B, output;
    ok(A.resize(4, 4), "full capacity");
    for (std::size_t i = 0; i < 4; ++i)
    {
        for (std::size_t j = 0; j < 4; ++j)
        {
            A(i, j) = std::numeric_limits<Scalar>::quiet_NaN();
        }
    }
    ok(A.resize(2, 2), "shrink resets active values");
    A(0, 0) = 1;
    A(0, 1) = 2;
    A(1, 0) = 3;
    A(1, 1) = 4;
    B = A;
    test.expect(all_finite(A), "bounded", "inactive NaNs ignored");
    ok(multiply(A, B, A), "multiply aliases left");
    close(A(0, 0), 7, "active product");
    close(A(1, 1), 22, "active product");
    ok(transpose(B, B), "transpose aliases input");
    close(B(0, 1), 3, "transpose");
    ok(add(B, B, B), "add aliases both");
    close(B(0, 1), 6, "add");
    auto saved = A;
    test.expect(A.resize(5, 2) == Status::out_of_range && same_bits(A, saved), "bounded", "capacity failure atomicity");
    test.expect(block(A, std::numeric_limits<std::size_t>::max(), 0, 1, 1, output) == Status::out_of_range, "bounded",
                "offset overflow prevented");
    ok(block(A, 1, 1, 1, 1, A), "in-place block");
    close(A(0, 0), 22, "block position");
    ok(A.resize(4, 4), "regrow");
    for (std::size_t i = 0; i < 4; ++i)
    {
        for (std::size_t j = 0; j < 4; ++j)
        {
            close(A(i, j), 0, "regrow clears inactive data");
            A(i, j) = static_cast<Scalar>(i * 4 + j);
        }
    }
    std::array<std::size_t, 4> const indices{0, 2, 3, 0};
    ok(gather(A, indices, 3, indices, 3, A), "covariance row/column selection aliases source");
    close(A(1, 2), 11, "cross block retained");
    ok(B.resize(1, 1), "block source");
    B(0, 0) = 42;
    ok(set_block(B, 1, 2, A), "runtime-positioned block");
    close(A(1, 2), 42, "runtime block value");
    saved = A;
    test.expect(set_block(B, 3, 3, A) == Status::out_of_range && same_bits(A, saved), "bounded",
                "block bounds atomicity");
    ok(output.resize(1, 2), "failure sentinel");
    output(0, 0) = -Scalar{0};
    output(0, 1) = std::numeric_limits<Scalar>::quiet_NaN();
    auto const sentinel = output;
    A(0, 0) = std::numeric_limits<Scalar>::quiet_NaN();
    B = A;
    test.expect(multiply(A, B, output) == Status::non_finite_input && same_bits(output, sentinel), "bounded",
                "nonfinite atomicity");
    ok(A.resize(1, 1), "overflow matrix");
    A(0, 0) = std::numeric_limits<Scalar>::max();
    B = A;
    test.expect(multiply(A, B, output) == Status::non_finite_result && same_bits(output, sentinel), "bounded",
                "overflow atomicity");

    Square L, covariance, W;
    ok(L.resize(3, 3), "known Cholesky factor");
    L(0, 0) = 2;
    L(1, 0) = Scalar{0.5};
    L(1, 1) = Scalar{1.5};
    L(2, 0) = Scalar{-0.2};
    L(2, 1) = Scalar{0.3};
    L(2, 2) = Scalar{1.1};
    ok(transpose(L, B), "factor transpose");
    ok(multiply(L, B, covariance), "SPD fixture");
    ok(cholesky(covariance, A), "Cholesky");
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            close(A(i, j), L(i, j), "known factor residual");
        }
    }
    ok(whiten(covariance, L, W), "correlated whitening");
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            close(W(i, j), i == j ? Scalar{1} : Scalar{0}, "L inverse, not covariance inverse");
        }
    }
    B = L;
    ok(multiply(covariance, L, A), "SPD right side");
    ok(solve_spd(covariance, A, A), "SPD solve aliases RHS");
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            close(A(i, j), L(i, j), "SPD solution");
        }
    }
    ok(multiply(L, covariance, A), "right-SPD right side");
    ok(right_solve_spd(A, covariance, A), "right-SPD alias");
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            close(A(i, j), L(i, j), "right-SPD solution");
        }
    }
    covariance(0, 1) += 1;
    test.expect(cholesky(covariance, output) == Status::not_positive_definite && same_bits(output, sentinel),
                "Cholesky", "asymmetry rejected");
    covariance(1, 0) = covariance(0, 1);
    covariance(0, 0) = -1;
    test.expect(cholesky(covariance, output) == Status::not_positive_definite && same_bits(output, sentinel),
                "Cholesky", "indefinite rejected");
    L(0, 0) = 0;
    test.expect(solve_lower(L, B, output) == Status::zero_or_unsafe_divisor && same_bits(output, sentinel),
                "triangular", "zero divisor atomicity");
    L(0, 1) = 1;
    test.expect(solve_lower(L, B, output) == Status::domain_error && same_bits(output, sentinel), "triangular",
                "nontriangular rejected first");

    BoundedMatrix<Backend, 8, 5> H;
    BoundedMatrix<Backend, 8, 8> identity;
    ok(H.resize(6, 3), "rectangular active matrix");
    ok(identity.resize(6, 6), "QR identity RHS");
    for (std::size_t i = 0; i < 6; ++i)
    {
        Scalar const x = static_cast<Scalar>(i + 1);
        H(i, 0) = 1;
        H(i, 1) = x;
        H(i, 2) = x * x;
        identity(i, i) = 1;
    }
    RankTolerance<Scalar> const tolerance{Scalar{0}, Scalar{100} * std::numeric_limits<Scalar>::epsilon()};
    QrResult<Backend, 8, 5, 8> reduced;
    ok(qr(H, identity, tolerance, reduced), "pivoted QR");
    test.expect(reduced.rank == 3 && reduced.permutation[0] == 2, "QR", "rank and strongest column pivot");
    // RHS=I gives Q^T. Verify geometry, independently of the rotation sequence.
    for (std::size_t i = 0; i < 6; ++i)
    {
        for (std::size_t j = 0; j < 6; ++j)
        {
            Scalar value{0};
            for (std::size_t k = 0; k < 6; ++k)
            {
                value += reduced.transformed_rhs(i, k) * reduced.transformed_rhs(j, k);
            }
            close(value, i == j ? Scalar{1} : Scalar{0}, "Q orthogonality");
        }
        for (std::size_t j = 0; j < 3; ++j)
        {
            Scalar value{0};
            for (std::size_t k = 0; k < 6; ++k)
            {
                value += reduced.transformed_rhs(k, i) * reduced.upper(k, j);
            }
            close(value, H(i, reduced.permutation[j]), "QR reconstructs permuted input");
        }
    }
    BoundedMatrix<Backend, 8, 8> U_t;
    ok(project_left_nullspace(H, identity, tolerance, U_t), "nullspace action");
    test.expect(U_t.rows() == 3 && U_t.columns() == 6, "nullspace", "retained dimension");
    BoundedMatrix<Backend, 8, 5> annihilated;
    ok(multiply(U_t, H, annihilated), "nullspace times feature Jacobian");
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            close(annihilated(i, j), 0, "annihilation");
        }
    }
    BoundedMatrix<Backend, 8, 2> rhs;
    BoundedMatrix<Backend, 5, 2> solution;
    ok(rhs.resize(6, 2), "LS RHS");
    for (std::size_t i = 0; i < 6; ++i)
    {
        rhs(i, 0) = 2 * H(i, 0) - H(i, 1) + Scalar{0.5} * H(i, 2);
        rhs(i, 1) = static_cast<Scalar>((i * i + 3) % 7);
    }
    ok(least_squares(H, rhs, tolerance, solution), "least squares");
    close(solution(0, 0), 2, "known solution");
    close(solution(1, 0), -1, "known solution");
    close(solution(2, 0), Scalar{0.5}, "known solution");
#if !defined(FORMAL_GVINS_TEST_CMSIS_DSP)
    Eigen::Matrix<Scalar, 6, 3> reference_H;
    Eigen::Matrix<Scalar, 6, 2> reference_rhs;
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            reference_H(i, j) = H(i, j);
        }
        for (int j = 0; j < 2; ++j)
        {
            reference_rhs(i, j) = rhs(i, j);
        }
    }
    Eigen::Matrix<Scalar, 3, 2> const reference_x = reference_H.colPivHouseholderQr().solve(reference_rhs);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            close(solution(i, j), reference_x(i, j), "independent Householder QR reference");
        }
    }
#endif
    auto const saved_solution = solution;
    for (std::size_t i = 0; i < 6; ++i)
    {
        H(i, 2) = H(i, 0);
    }
    test.expect(least_squares(H, rhs, tolerance, solution) == Status::ill_conditioned &&
                    same_bits(solution, saved_solution),
                "QR", "rank-deficient solve preserves output");
    auto const saved_nullspace = U_t;
    test.expect(project_left_nullspace(H, identity, tolerance, U_t) == Status::ill_conditioned &&
                    same_bits(U_t, saved_nullspace),
                "QR", "rank-deficient feature rejected");
    auto const saved_reduced = reduced;
    H(0, 0) = std::numeric_limits<Scalar>::infinity();
    test.expect(qr(H, identity, tolerance, reduced) == Status::non_finite_input &&
                    same_bits(reduced.upper, saved_reduced.upper) &&
                    same_bits(reduced.transformed_rhs, saved_reduced.transformed_rhs) &&
                    reduced.rank == saved_reduced.rank && reduced.permutation == saved_reduced.permutation,
                "QR", "complete result preserved");
    H(0, 0) = 1;
    test.expect(qr(H, identity, RankTolerance<Scalar>{0, 1}, reduced) == Status::domain_error, "QR",
                "invalid rank tolerance");
    ok(H.resize(6, 3), "zero system");
    ok(qr(H, identity, tolerance, reduced), "rank-zero factorization is valid");
    test.expect(reduced.rank == 0, "QR", "zero rank");

    ok(H.resize(2, 1), "subnormal column");
    H(0, 0) = H(1, 0) = std::numeric_limits<Scalar>::denorm_min();
    ok(identity.resize(2, 2), "subnormal QR RHS");
    identity(0, 0) = identity(1, 1) = 1;
    ok(qr(H, identity, RankTolerance<Scalar>{0, 0}, reduced), "subnormal rotation");
    for (std::size_t i = 0; i < 2; ++i)
    {
        Scalar norm{0};
        for (std::size_t j = 0; j < 2; ++j)
        {
            norm += reduced.transformed_rhs(i, j) * reduced.transformed_rhs(i, j);
        }
        close(norm, 1, "subnormal input does not destroy orthogonality");
    }

    ok(A.resize(3, 3), "nonsymmetric square");
    A(0, 1) = 2;
    A(1, 2) = 3;
    A(2, 0) = 4;
    B = A;
    Square general;
    ok(least_squares(A, B, tolerance, A), "general solve aliases system");
    general = A;
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            close(general(i, j), i == j ? Scalar{1} : Scalar{0}, "square inverse action");
        }
    }
    ok(H.resize(3, 2), "near-rank threshold");
    ok(identity.resize(3, 3), "threshold RHS");
    H(0, 0) = 1;
    H(1, 1) = Scalar{0.001};
    ok(qr(H, identity, RankTolerance<Scalar>{0, Scalar{0.002}}, reduced), "numerically rank one");
    test.expect(reduced.rank == 1 && reduced.upper(1, 1) == H(1, 1), "QR",
                "threshold reports rank without discarding trailing coefficients");
    ok(qr(reduced.upper, reduced.transformed_rhs, RankTolerance<Scalar>{0, Scalar{0.0005}}, reduced),
       "QR aliases both result members");
    test.expect(reduced.rank == 2, "QR", "explicit smaller tolerance accepts second pivot");
    auto const before_overflow = reduced;
    H(0, 0) = H(1, 0) = std::numeric_limits<Scalar>::max();
    test.expect(qr(H, identity, tolerance, reduced) == Status::non_finite_result &&
                    same_bits(reduced.upper, before_overflow.upper) &&
                    same_bits(reduced.transformed_rhs, before_overflow.transformed_rhs) &&
                    reduced.rank == before_overflow.rank && reduced.permutation == before_overflow.permutation,
                "QR", "norm overflow preserves complete result");
    std::cout << "Scalar bytes=" << sizeof(Scalar) << "; QR result capacity 8x5 + 8x8: " << sizeof(reduced)
              << " bytes\n";
}

template <typename Scalar> void run_qr_shapes(Test & test)
{
    using Backend = formal_gvins::test::Backend<Scalar>;
    RankTolerance<Scalar> const tolerance{0, Scalar{100} * std::numeric_limits<Scalar>::epsilon()};
    Scalar const bound = Scalar{1000} * std::numeric_limits<Scalar>::epsilon();
    std::uint32_t seed = 71;
    auto sample = [&]()
    {
        seed = seed * 1664525U + 1013904223U;
        return static_cast<Scalar>(seed % 2001U) / Scalar{1000} - Scalar{1};
    };
    for (std::size_t rows = 1; rows <= 8; ++rows)
    {
        for (std::size_t columns = 1; columns <= 5; ++columns)
        {
            BoundedMatrix<Backend, 8, 5> H;
            BoundedMatrix<Backend, 8, 2> rhs;
            BoundedMatrix<Backend, 5, 2> solution;
            test.expect(H.resize(rows, columns) == Status::success && rhs.resize(rows, 2) == Status::success,
                        "QR shapes", "active extents");
            for (std::size_t i = 0; i < rows; ++i)
            {
                for (std::size_t j = 0; j < columns; ++j)
                {
                    H(i, j) = sample() + (i == j ? Scalar{4} : Scalar{0});
                }
                rhs(i, 0) = sample();
                rhs(i, 1) = sample();
            }
            QrResult<Backend, 8, 5, 2> reduced;
            test.expect(qr(H, rhs, tolerance, reduced) == Status::success && reduced.rank == std::min(rows, columns),
                        "QR shapes", "rectangular full rank");
            // Orthogonal reduction preserves the full information matrix and
            // gradient. This also checks the column permutation used by compression.
            for (std::size_t i = 0; i < columns; ++i)
            {
                for (std::size_t j = 0; j < columns; ++j)
                {
                    Scalar original{0}, transformed{0};
                    for (std::size_t row = 0; row < rows; ++row)
                    {
                        original += H(row, reduced.permutation[i]) * H(row, reduced.permutation[j]);
                        transformed += reduced.upper(row, i) * reduced.upper(row, j);
                    }
                    test.expect(std::abs(original - transformed) <= bound * (1 + std::abs(original)), "QR shapes",
                                "information matrix preserved");
                }
                for (std::size_t j = 0; j < 2; ++j)
                {
                    Scalar original{0}, transformed{0};
                    for (std::size_t row = 0; row < rows; ++row)
                    {
                        original += H(row, reduced.permutation[i]) * rhs(row, j);
                        transformed += reduced.upper(row, i) * reduced.transformed_rhs(row, j);
                    }
                    test.expect(std::abs(original - transformed) <= bound * (1 + std::abs(original)), "QR shapes",
                                "information gradient preserved");
                }
            }
            auto const status = least_squares(H, rhs, tolerance, solution);
            if (rows < columns)
            {
                test.expect(status == Status::out_of_range && solution.rows() == 0, "QR shapes",
                            "underdetermined solve rejected atomically");
                continue;
            }
            test.expect(status == Status::success, "QR shapes", "least squares");
            for (std::size_t i = 0; i < columns; ++i)
            {
                for (std::size_t j = 0; j < 2; ++j)
                {
                    Scalar normal_residual{0};
                    for (std::size_t row = 0; row < rows; ++row)
                    {
                        Scalar residual = -rhs(row, j);
                        for (std::size_t column = 0; column < columns; ++column)
                        {
                            residual += H(row, column) * solution(column, j);
                        }
                        normal_residual += H(row, i) * residual;
                    }
                    test.expect(std::abs(normal_residual) <= bound, "QR shapes", "H transpose times residual = 0");
                }
            }
        }
    }
}
} /* end anonymous namespace */

int main()
{
    formal_gvins::test::configure_backend_test();
    Test test;
    run<float>(test);
    run<double>(test);
    run_qr_shapes<float>(test);
    run_qr_shapes<double>(test);
    return test.failures() == 0 ? 0 : 1;
}
