/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/msckf/clones.hpp>
#include <formal_gvins/msckf/prediction.hpp>
#include <formal_gvins/msckf/correction.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"
#include "msckf_support.hpp"

int main()
{
    using namespace formal_gvins;
    using L = test::Backend<double>;
    test::configure_backend_test();
    test::Context t;
    msckf::Tolerances<double> const tol{1e-12, 1e-12};
    msckf::State<L, 11> state, candidate;
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            state.P(i, j) = (i == j ? 1. : 0) + .01 * (i + 1) * (j + 1);
        }
    }
    msckf::mirror_upper(state.P);
    auto original = state;
    state.imu.pose.p_WB(0) = 5;
    state.first.pose.p_WB(0) = 3;
    for (std::size_t c = 0; c < 11; ++c)
    {
        state.time_ns = c;
        t.expect(succeeded(msckf::augment_clone(state, c + 1, tol, candidate)), "clones",
                 "augmentation including capacity boundary");
        t.expect(state.clones[c].pose.p_WB(0) == 5 && state.clones[c].first.p_WB(0) == 3, "clones",
                 "separate current/frozen pose");
    }
    constexpr std::array<std::size_t, 6> indices{0, 1, 2, 6, 7, 8};
    for (std::size_t i = 0; i < 81; ++i)
    {
        for (std::size_t j = 0; j < 81; ++j)
        {
            t.expect(state.P(i, j) ==
                         original.P(i < 15 ? i : indices[(i - 15) % 6], j < 15 ? j : indices[(j - 15) % 6]),
                     "clones", "full congruence including clone cross blocks");
        }
    }
    auto before = state;
    t.expect(msckf::augment_clone(state, 99, tol, candidate) == Status::out_of_range &&
                 test::equal_state(state, before),
             "clones", "capacity atomicity");
    t.expect(succeeded(msckf::remove_clone(state, 5, tol, candidate)), "clones", "middle removal");
    for (std::size_t i = 0; i < 75; ++i)
    {
        for (std::size_t j = 0; j < 75; ++j)
        {
            t.expect(state.P(i, j) == before.P(i < 45 ? i : i + 6, j < 45 ? j : j + 6), "clones",
                     "principal submatrix selection");
        }
    }
    t.expect(state.clones[5].id == 7 && state.clones[5].time_ns == 6, "clones", "retained identity and timestamp");
    before = state;
    t.expect(msckf::augment_clone(state, 100, tol, candidate) == Status::domain_error &&
                 test::equal_state(state, before),
             "clones", "duplicate exposure");
    state.time_ns = 11;
    before = state;
    t.expect(msckf::augment_clone(state, 1, tol, candidate) == Status::domain_error && test::equal_state(state, before),
             "clones", "duplicate identity");
    t.expect(msckf::remove_clone(state, 10, tol, candidate) == Status::out_of_range && test::equal_state(state, before),
             "clones", "bad removal index");
    t.expect(msckf::remove_clone(state, 0, tol, state) == Status::domain_error, "clones", "scratch alias rejected");
    while (state.clone_count > 0)
    {
        t.expect(succeeded(msckf::remove_clone(state, 0, tol, candidate)), "clones", "remove all");
    }
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            t.expect(state.P(i, j) == original.P(i, j), "clones", "original marginal retained");
        }
    }

    msckf::State<L, 1> rounded, rounded_scratch;
    for (std::size_t i = 0; i < 15; ++i)
    {
        rounded.P(i, i) = 1;
    }
    rounded.P(0, 6) = .1;
    rounded.P(6, 0) = .1 + 1e-14;
    t.expect(succeeded(msckf::augment_clone(rounded, 1, tol, rounded_scratch)), "clones", "admitted roundoff symmetry");
    for (std::size_t j = 0; j < 21; ++j)
    {
        t.expect(rounded.P(0, j) == rounded.P(15, j) && rounded.P(6, j) == rounded.P(18, j), "clones",
                 "identical variable copies after canonicalization");
    }
    msckf::State<L, 0> no_clones, no_scratch;
    t.expect(msckf::augment_clone(no_clones, 1, tol, no_scratch) == Status::out_of_range, "clones",
             "zero capacity configuration");
    msckf::State<L, 1> large, large_scratch;
    large.P(0, 0) = 1e308;
    auto const large_before = large;
    t.expect(succeeded(msckf::validate(large, tol)), "clones", "large finite PSD prior is admitted");
    t.expect(msckf::augment_clone(large, 1, tol, large_scratch) == Status::non_finite_result &&
                 test::equal_state(large, large_before),
             "clones", "augmented covariance norm overflow preserves state");
    // Repeated mathematical composition; scheduling/feature ownership remain external.
    msckf::PredictionWorkspace<L, 11> prediction;
    msckf::CorrectionWorkspace<L, 11, 1> correction;
    linalg::Vector<L, 3> gravity;
    linalg::Matrix<L, 12, 12> noise;
    linalg::BoundedMatrix<L, 1, 81> H;
    linalg::BoundedMatrix<L, 1, 1> r, V;
    (void)r.resize(1, 1);
    (void)V.resize(1, 1);
    V(0, 0) = 1;
    for (std::size_t step = 0; step < 12; ++step)
    {
        msckf::ImuInterval<L> interval;
        interval.start_ns = state.time_ns;
        interval.end_ns = state.time_ns + 5'000'000;
        interval.omega(2) = .1;
        t.expect(succeeded(msckf::predict(state, interval, gravity, noise, tol, prediction)), "composition", "predict");
        t.expect(succeeded(msckf::augment_clone(state, 100 + step, tol, candidate)), "composition", "clone");
        (void)H.resize(1, state.dimension());
        H(0, 0) = 1;
        r(0, 0) = .001;
        auto first = state.first;
        t.expect(succeeded(msckf::correct(state, r, H, V, tol, .2, correction)), "composition",
                 "correct supplied position observation");
        t.expect(test::equal_pose(first.pose, state.first.pose), "composition", "FEJ held through correction");
        if (state.clone_count > 2)
        {
            t.expect(succeeded(msckf::remove_clone(state, 0, tol, candidate)), "composition", "retire");
        }
        t.expect(succeeded(msckf::validate(state, tol)), "composition", "joint validity");
    }
    return t.failures() == 0 ? 0 : 1;
}
