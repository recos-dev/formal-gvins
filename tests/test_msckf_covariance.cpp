/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/msckf/prediction.hpp>
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
    msckf::PredictionWorkspace<L, 11> work;
    msckf::ImuInterval<L> interval;
    interval.end_ns = 10'000'000;
    linalg::Vector<L, 3> gravity;
    linalg::Matrix<L, 12, 12> Qc;
    for (std::size_t i = 0; i < 3; ++i)
    {
        Qc(i, i) = 1;
    }
    t.expect(succeeded(msckf::predict(state, interval, gravity, Qc, tol, work)), "covariance",
             "zero rate accelerometer noise");
    t.expect(test::near(state.P(0, 0), 2.5e-7, 1e-20) && test::near(state.P(0, 3), 5e-5, 1e-18) &&
                 test::near(state.P(3, 3), .01, 1e-16),
             "covariance", "h cubed/4, h squared/2, h noise");
    t.expect(state.time_ns == interval.end_ns && test::equal_pose(state.first.pose, state.imu.pose), "covariance",
             "time and FEJ publish together");
    auto saved = state;
    t.expect(msckf::predict(state, interval, gravity, Qc, tol, work) == Status::domain_error &&
                 test::equal_state(state, saved),
             "covariance", "stale interval atomicity");
    interval.start_ns = state.time_ns;
    interval.end_ns += 10'000'000;
    Qc(0, 1) = .5;
    t.expect(msckf::predict(state, interval, gravity, Qc, tol, work) == Status::domain_error &&
                 test::equal_state(state, saved),
             "covariance", "bad noise symmetry atomicity");
    Qc(0, 1) = 0;
    Qc(0, 0) = std::numeric_limits<double>::max();
    t.expect(msckf::predict(state, interval, gravity, Qc, tol, work) == Status::non_finite_result &&
                 test::equal_state(state, saved),
             "covariance", "overflow atomicity");
    Qc = {};
    state.clone_count = 11;
    (void)state.P.resize(81, 81);
    for (std::size_t i = 0; i < 81; ++i)
    {
        state.P(i, i) = 1;
    }
    for (std::size_t i = 0; i < 11; ++i)
    {
        state.clones[i].id = i;
        state.clones[i].time_ns = i;
    }
    state.P(3, 15) = state.P(15, 3) = .2;
    t.expect(succeeded(msckf::predict(state, interval, gravity, Qc, tol, work)), "covariance", "full clone capacity");
    t.expect(test::near(state.P(0, 15), .002, 1e-17) && state.P(3, 15) == .2 && state.P(15, 15) == 1, "covariance",
             "IC propagation and CC preservation");
    t.expect(msckf::predict(work.candidate, interval, gravity, Qc, tol, work) == Status::domain_error, "covariance",
             "workspace alias rejected");
    std::cout << "Prediction workspace<11> bytes: " << sizeof(work) << '\n';
    return t.failures() == 0 ? 0 : 1;
}
