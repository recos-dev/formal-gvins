/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/msckf/prediction.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"
#include <limits>

int main()
{
    using namespace formal_gvins;
    using L = test::Backend<double>;
    test::configure_backend_test();
    test::Context t;
    msckf::Tolerances<double> const tol{1e-12, 1e-12};
    msckf::ImuState<L> input, output;
    input.v_WB(0) = 3;
    input.b_a(1) = .2;
    msckf::ImuInterval<L> interval;
    interval.end_ns = 10'000'000;
    interval.a(0) = 4;
    linalg::Vector<L, 3> gravity;
    t.expect(succeeded(msckf::predict_nominal(input, interval, gravity, tol, output)), "prediction", "zero-rate mean");
    t.expect(test::near(output.pose.p_WB(0), .0302, 1e-16) && test::near(output.v_WB(0), 3.04, 1e-15), "prediction",
             "initial velocity and acceleration");
    t.expect(output.b_a(1) == .2, "prediction", "bias unchanged");
    auto before = output;
    interval.omega(0) = std::numeric_limits<double>::quiet_NaN();
    t.expect(msckf::predict_nominal(input, interval, gravity, tol, output) == Status::non_finite_input &&
                 output.pose.p_WB(0) == before.pose.p_WB(0),
             "prediction", "failure atomicity");
    interval.omega(0) = 1;
    t.expect(succeeded(msckf::predict_nominal(input, interval, gravity, tol, input)), "prediction", "alias supported");
    msckf::ImuState<L> start, full, split;
    interval.start_ns = 0;
    interval.end_ns = 10'000'000;
    t.expect(succeeded(msckf::predict_nominal(start, interval, gravity, tol, full)), "prediction", "whole interval");
    interval.end_ns = 5'000'000;
    t.expect(succeeded(msckf::predict_nominal(start, interval, gravity, tol, split)), "prediction", "first half");
    interval.start_ns = 5'000'000;
    interval.end_ns = 10'000'000;
    t.expect(succeeded(msckf::predict_nominal(split, interval, gravity, tol, split)), "prediction", "second half");
    t.expect(linalg::max_abs(split.pose.p_WB - full.pose.p_WB) < 1e-14 &&
                 linalg::max_abs(split.v_WB - full.v_WB) < 1e-14 &&
                 so3::same_rotation(split.pose.q_WB, full.pose.q_WB, 1e-14),
             "prediction", "held-input composition");
    interval.end_ns = 0;
    t.expect(msckf::predict_nominal(input, interval, gravity, tol, output) == Status::out_of_range, "prediction",
             "zero interval");
    return t.failures() == 0 ? 0 : 1;
}
