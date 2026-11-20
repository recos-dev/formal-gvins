/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/msckf/correction.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"
#include "msckf_support.hpp"
#include <limits>

int main()
{
    using namespace formal_gvins;
    using L = test::Backend<double>;
    test::configure_backend_test();
    test::Context t;
    msckf::Tolerances<double> const tol{1e-12, 1e-12};
    msckf::State<L, 11> state;
    msckf::CorrectionWorkspace<L, 11, 3> work;
    linalg::BoundedMatrix<L, 3, 81> H;
    (void)H.resize(1, 15);
    H(0, 0) = 1;
    linalg::BoundedMatrix<L, 3, 1> r;
    (void)r.resize(1, 1);
    r(0, 0) = 2;
    linalg::BoundedMatrix<L, 3, 3> V;
    (void)V.resize(1, 1);
    V(0, 0) = 1;
    state.P(0, 0) = 1;
    t.expect(succeeded(msckf::correct(state, r, H, V, tol, .2, work)), "correction",
             "scalar update with singular PSD prior");
    t.expect(test::near(state.imu.pose.p_WB(0), 1., 1e-14) && test::near(state.P(0, 0), .5, 1e-14) &&
                 state.first.pose.p_WB(0) == 0,
             "correction", "gain, Joseph and frozen reference");
    auto saved = state;
    V(0, 0) = -.01;
    t.expect(msckf::correct(state, r, H, V, tol, .2, work) == Status::not_positive_definite &&
                 test::equal_state(state, saved),
             "correction", "reject non-SPD V even if S positive");
    V(0, 0) = 1;
    H(0, 0) = 0;
    H(0, 6) = 1;
    state.P(6, 6) = 1;
    saved = state;
    t.expect(msckf::correct(state, r, H, V, tol, .2, work) == Status::domain_error && test::equal_state(state, saved),
             "correction", "angle bound whole-state atomicity");
    r(0, 0) = .4;
    t.expect(succeeded(msckf::correct(state, r, H, V, tol, .2, work)), "correction", "angle bound equality");
    t.expect(test::near(state.P(6, 6), .5, 1e-14) && state.first.pose.q_WB.q0() == 1, "correction",
             "identity reset and frozen quaternion");
    state = msckf::State<L, 11>{};
    state.clone_count = 11;
    state.time_ns = 20;
    (void)state.P.resize(81, 81);
    for (std::size_t i = 0; i < 11; ++i)
    {
        state.clones[i].time_ns = i;
        state.clones[i].id = i;
    }
    for (std::size_t i = 0; i < 81; ++i)
    {
        state.P(i, i) = 1;
    }
    state.P(0, 15) = state.P(15, 0) = .5;
    (void)H.resize(1, 81);
    H(0, 15) = 1;
    r(0, 0) = .1;
    t.expect(succeeded(msckf::correct(state, r, H, V, tol, .2, work)), "correction", "maximum clone count");
    t.expect(test::near(state.imu.pose.p_WB(0), .025, 1e-15) && test::near(state.clones[0].pose.p_WB(0), .05, 1e-15),
             "correction", "cross covariance updates IMU and clone");
    saved = state;
    r(0, 0) = std::numeric_limits<double>::quiet_NaN();
    t.expect(msckf::correct(state, r, H, V, tol, .2, work) == Status::non_finite_input &&
                 test::equal_state(state, saved),
             "correction", "nonfinite residual atomicity");
    r(0, 0) = 1;
    (void)H.resize(1, 81);
    H(0, 78) = 1;
    t.expect(msckf::correct(state, r, H, V, tol, .2, work) == Status::domain_error && test::equal_state(state, saved),
             "correction", "last clone angle rejects entire candidate");
    state.imu.pose.p_WB(0) = std::numeric_limits<double>::max();
    saved = state;
    (void)H.resize(1, 81);
    H(0, 0) = 1;
    r(0, 0) = std::numeric_limits<double>::max();
    t.expect(msckf::correct(state, r, H, V, tol, .2, work) == Status::non_finite_result &&
                 test::equal_state(state, saved),
             "correction", "late finite-input overflow preserves state");
    msckf::State<L, 0> imu_only;
    msckf::CorrectionWorkspace<L, 0, 1> imu_work;
    linalg::BoundedMatrix<L, 1, 15> imu_H;
    (void)imu_H.resize(1, 15);
    imu_H(0, 0) = 1;
    linalg::BoundedMatrix<L, 1, 1> imu_r, imu_V;
    (void)imu_r.resize(1, 1);
    (void)imu_V.resize(1, 1);
    imu_V(0, 0) = 1;
    t.expect(succeeded(msckf::correct(imu_only, imu_r, imu_H, imu_V, tol, .2, imu_work)), "correction",
             "zero-capacity IMU-only filter");
    (void)H.resize(0, 81);
    t.expect(msckf::correct(state, r, H, V, tol, .2, work) == Status::out_of_range, "correction",
             "empty input rejected");
    std::cout << "Correction workspace<11,3> bytes: " << sizeof(work) << '\n';
    return t.failures() == 0 ? 0 : 1;
}
