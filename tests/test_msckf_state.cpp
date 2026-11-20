/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/msckf/state.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"
#include <limits>

int main()
{
    using namespace formal_gvins;
    using L = test::Backend<double>;
    using State = msckf::State<L, 11>;
    test::configure_backend_test();
    test::Context t;
    msckf::Tolerances<double> const tol{1e-12, 1e-12};
    State state;
    t.expect(succeeded(msckf::validate(state, tol)), "state", "zero clones and PSD zero covariance");
    t.expect(state.P.rows() == 15 && State::dimension_capacity == 81, "state", "capacity and active size");
    state.clone_count = 12;
    t.expect(msckf::validate(state, tol) == Status::out_of_range, "state", "count checked before indexing");
    state.clone_count = 11;
    (void)state.P.resize(81, 81);
    state.time_ns = 20;
    for (std::size_t i = 0; i < 81; ++i)
    {
        state.P(i, i) = 1;
    }
    for (std::size_t i = 0; i < 11; ++i)
    {
        state.clones[i].id = i;
        state.clones[i].time_ns = i;
    }
    state.P(0, 80) = state.P(80, 0) = .2;
    State accepted;
    t.expect(succeeded(msckf::assign(state, tol, accepted)), "state", "full capacity accepted");
    t.expect(accepted.P(80, 0) == .2, "state", "cross block retained");
    state.clones[10].id = 0;
    t.expect(msckf::assign(state, tol, accepted) == Status::domain_error && accepted.clones[10].id == 10, "state",
             "duplicate identity atomicity");
    state.clones[10].id = 10;
    state.P(80, 0) = .3;
    t.expect(msckf::assign(state, tol, accepted) == Status::domain_error && accepted.P(80, 0) == .2, "state",
             "asymmetry atomicity");
    state.P(80, 0) = .2;
    state.imu.b_a(0) = std::numeric_limits<double>::quiet_NaN();
    t.expect(msckf::assign(state, tol, accepted) == Status::non_finite_input, "state", "nonfinite active state");
    State inactive;
    inactive.clones[0].pose.p_WB(0) = std::numeric_limits<double>::quiet_NaN();
    t.expect(succeeded(msckf::validate(inactive, tol)), "state", "inactive slots excluded");
    msckf::State<L, 0> imu_only;
    t.expect(succeeded(msckf::validate(imu_only, tol)), "state", "N=0-only configuration");
    t.expect(succeeded(msckf::assign(accepted, tol, accepted)), "state", "in-place admission");
    state = accepted;
    state.clones[1].time_ns = state.clones[0].time_ns;
    t.expect(msckf::validate(state, tol) == Status::domain_error, "state", "duplicate clone timestamp");
    state = accepted;
    state.P(0, 0) = std::numeric_limits<double>::infinity();
    t.expect(msckf::validate(state, tol) == Status::non_finite_input, "state", "nonfinite covariance");
    t.expect(msckf::validate(accepted, msckf::Tolerances<double>{-1, 1e-12}) == Status::domain_error, "state",
             "invalid tolerance");
    std::cout << "State<11> bytes: " << sizeof(State) << '\n';
    return t.failures() == 0 ? 0 : 1;
}
