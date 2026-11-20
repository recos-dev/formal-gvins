/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/linalg/backend/eigen.hpp>
#include <formal_gvins/msckf/camera.hpp>
#include <formal_gvins/msckf/clones.hpp>
#include <formal_gvins/msckf/correction.hpp>
#include <formal_gvins/msckf/prediction.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string_view>
#include <vector>

using L = formal_gvins::linalg::EigenBackend<double>;
using State = formal_gvins::msckf::State<L, 11>;
using Vector = formal_gvins::linalg::Vector<L, 3>;
namespace math = formal_gvins::msckf;
using formal_gvins::Status;
using formal_gvins::succeeded;
constexpr math::Tolerances<double> tolerance{1e-12, 1e-12};

State fixture(std::size_t clones)
{
    State state;
    state.time_ns = 100;
    state.clone_count = clones;
    (void)state.P.resize(state.dimension(), state.dimension());
    for (std::size_t i = 0; i < state.dimension(); ++i)
    {
        state.P(i, i) = .1;
    }
    for (std::size_t i = 0; i < clones; ++i)
    {
        state.clones[i].time_ns = i;
        state.clones[i].id = i;
    }
    return state;
}

template <typename Operation, typename Metric>
bool measure(std::string_view name, State const & initial, Operation operation, std::string_view metric_name,
             Metric metric)
{
    constexpr std::size_t warmup = 5;
    constexpr std::size_t samples = 50;
    std::vector<double> times;
    double maximum_error = 0;
    for (std::size_t iteration = 0; iteration < warmup + samples; ++iteration)
    {
        State state = initial; // Reset fixtures outside the timed operation.
        auto const start = std::chrono::steady_clock::now();
        Status const status = operation(state);
        auto const stop = std::chrono::steady_clock::now();
        if (!succeeded(status) || !succeeded(math::validate(state, tolerance)))
        {
            std::cerr << name << ": operation or state validation failed at iteration " << iteration << '\n';
            return false;
        }
        double const error = metric(state);
        if (!std::isfinite(error) || error > 1e-12)
        {
            std::cerr << name << ": analytical residual check failed at iteration " << iteration << '\n';
            return false;
        }
        maximum_error = std::max(maximum_error, error);
        if (iteration >= warmup)
        {
            times.push_back(std::chrono::duration<double, std::micro>(stop - start).count());
        }
    }
    std::sort(times.begin(), times.end());
    std::cout << name << ',' << initial.clone_count << ',' << samples << ',' << (times[24] + times[25]) / 2 << ','
              << times[47] << ',' << times.back() << ',' << metric_name << ',' << maximum_error << '\n';
    return true;
}

int main()
{
    using namespace formal_gvins;
    Eigen::internal::set_is_malloc_allowed(false);
    std::cout << std::setprecision(10);
    std::cout << "module,active_clones,samples,median_us,p95_us,max_us,metric,max_error\n";
    math::PredictionWorkspace<L, 11> prediction;
    math::CorrectionWorkspace<L, 11, 3> correction;
    State scratch;
    Vector gravity;
    math::ImuInterval<L> interval;
    interval.start_ns = 100;
    interval.end_ns = 5'000'100;
    interval.a(0) = 2;
    interval.omega(2) = .2;
    linalg::Matrix<L, 12, 12> Qc;
    for (std::size_t i = 0; i < 12; ++i)
    {
        Qc(i, i) = 1e-6;
    }
    linalg::BoundedMatrix<L, 3, 81> H;
    linalg::BoundedMatrix<L, 3, 3> V;
    (void)V.resize(3, 3);
    linalg::BoundedMatrix<L, 3, 1> residual;
    (void)residual.resize(3, 1);
    for (std::size_t i = 0; i < 3; ++i)
    {
        V(i, i) = 1;
        residual(i, 0) = .01;
    }
    double const sine = std::sin(.2 * .005 / 2);
    double const expected_position = 4 * sine * sine / (.2 * .2);
    auto position_error = [expected_position](State const & state)
    { return std::abs(state.imu.pose.p_WB(0) - expected_position); };
    for (std::size_t count : {std::size_t{0}, std::size_t{11}})
    {
        auto initial = fixture(count);
        if (!measure(
                "nominal", initial,
                [&](State & state)
                { return math::predict_nominal(state.imu, interval, gravity, tolerance, state.imu); },
                "position_error_m", position_error))
        {
            return 1;
        }
        if (!measure(
                "prediction", initial,
                [&](State & state) { return math::predict(state, interval, gravity, Qc, tolerance, prediction); },
                "position_error_m", position_error))
        {
            return 1;
        }
        (void)H.resize(3, initial.dimension());
        for (std::size_t i = 0; i < 3; ++i)
        {
            H(i, i) = 1;
        }
        if (!measure(
                "correction", initial,
                [&](State & state) { return math::correct(state, residual, H, V, tolerance, .2, correction); },
                "position_error_m", [](State const & state) { return std::abs(state.imu.pose.p_WB(0) - .001 / 1.1); }))
        {
            return 1;
        }
    }
    if (!measure(
            "augmentation", fixture(10),
            [&](State & state) { return math::augment_clone(state, 99, tolerance, scratch); }, "copied_variance_error",
            [](State const & state) { return std::abs(state.P(75, 75) - .1); }))
    {
        return 1;
    }
    if (!measure(
            "removal", fixture(11), [&](State & state) { return math::remove_clone(state, 5, tolerance, scratch); },
            "retained_variance_error", [](State const & state) { return std::abs(state.P(69, 69) - .1); }))
    {
        return 1;
    }
    math::CameraObservation<L> observation;
    math::Radtan<double> camera{400, 450, 320, 240, 0, 0, 0, 0};
    math::CameraDomain<double> domain{.1, 60, 1.5};
    auto feature = Vector::from_row_major({.4, .2, 4.});
    linalg::Vector<L, 2> pixel = linalg::Vector<L, 2>::from_row_major({360., 262.5});
    Vector lever;
    if (!measure(
            "camera", fixture(0),
            [&](State & state)
            {
                return math::camera_observation(state.imu.pose, state.first.pose, feature, so3::UnitQuaternion<L>{},
                                                lever, pixel, camera, domain, tolerance, observation);
            },
            "pixel_residual", [&](State const &) { return linalg::max_abs(observation.residual); }))
    {
        return 1;
    }
    std::cerr << "Eigen/double, MaxClones=11, correction MaxRows=3, serial host benchmark\n"
              << "State bytes=" << sizeof(State) << ", prediction workspace bytes=" << sizeof(prediction)
              << ", correction workspace bytes=" << sizeof(correction) << '\n';
}
