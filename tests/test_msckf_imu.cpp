/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/msckf/imu.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"
#include <limits>

int main()
{
    using namespace formal_gvins;
    using L = test::Backend<double>;
    test::configure_backend_test();
    test::Context t;
    msckf::ImuSample<L> left, right, middle;
    left.time_ns = std::numeric_limits<std::uint64_t>::max() - 10'000'000;
    right.time_ns = left.time_ns + 10'000'000;
    left.specific_force(0) = 2;
    right.specific_force(0) = 6;
    left.angular_rate(1) = -1;
    right.angular_rate(1) = 3;
    auto const time = left.time_ns + 2'500'000;
    t.expect(succeeded(msckf::interpolate(left, right, time, middle)), "imu", "integer epoch interpolation");
    t.expect(middle.specific_force(0) == 3 && middle.angular_rate(1) == 0, "imu", "quarter interval value");
    t.expect(succeeded(msckf::interpolate(left, right, left.time_ns, middle)) && middle.specific_force(0) == 2, "imu",
             "exact endpoint");
    t.expect(msckf::interpolate(left, right, left.time_ns - 1, middle) == Status::out_of_range &&
                 middle.time_ns == left.time_ns,
             "imu", "extrapolation atomicity");
    linalg::Vector<L, 3> ba, bg;
    ba(0) = 1;
    bg(1) = .5;
    msckf::ImuInterval<L> interval;
    t.expect(succeeded(msckf::prepare_interval(left, right, ba, bg, interval)), "imu", "average with bias");
    t.expect(interval.a(0) == 3 && interval.omega(1) == .5 && test::near(interval.duration(), .01, 1e-17), "imu",
             "held input and seconds");
    ba(0) = 2;
    t.expect(succeeded(msckf::prepare_interval(left, right, ba, bg, interval)) && interval.a(0) == 2, "imu",
             "new bias recomputed");
    t.expect(msckf::prepare_interval(left, left, ba, bg, interval) == Status::out_of_range && interval.a(0) == 2, "imu",
             "zero interval atomicity");
    auto bad = left;
    bad.time_ns = left.time_ns - 1;
    t.expect(msckf::interpolate(bad, right, time, middle) == Status::out_of_range, "imu",
             "original raw gap cannot be bypassed");
    t.expect(succeeded(msckf::interpolate(left, right, time, left)) && left.specific_force(0) == 3, "imu",
             "input alias");
    right.angular_rate(0) = std::numeric_limits<double>::quiet_NaN();
    t.expect(msckf::prepare_interval(left, right, ba, bg, interval) == Status::non_finite_input, "imu",
             "nonfinite input");
    interval.omega(0) = 10;
    interval.omega(1) = 0;
    t.expect(succeeded(msckf::validate_interval(interval)), "imu", "rate bound equality");
    interval.omega(0) = 10.1;
    t.expect(msckf::validate_interval(interval) == Status::domain_error, "imu", "rate bound rejection");
    return t.failures() == 0 ? 0 : 1;
}
