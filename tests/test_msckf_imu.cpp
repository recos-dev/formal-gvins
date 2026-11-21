/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/msckf/imu.hpp>
#include <formal_gvins/runtime/ring_buffer.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"
#include <limits>

namespace
{
void retained_imu_interval(formal_gvins::test::Context & test)
{
    using namespace formal_gvins;
    using L = test::Backend<double>;
    runtime::RingBuffer<msckf::ImuSample<L>, 4> history;
    std::uint64_t const epoch = std::numeric_limits<std::uint64_t>::max() - 20'000'000;
    for (std::uint64_t i = 0; i < 4; ++i)
    {
        msckf::ImuSample<L> sample;
        sample.time_ns = epoch + i * 5'000'000;
        sample.specific_force(0) = static_cast<double>(2 * i);
        sample.angular_rate(1) = static_cast<double>(i);
        test.expect(history.push(sample), "imu history", "retain each raw sample");
    }

    // Two consumers still need this interval. Reading/interpolating must not retire it.
    std::uint64_t const exposure = epoch + 7'500'000;
    msckf::ImuSample<L> image_sample;
    test.expect(succeeded(msckf::interpolate(history[1], history[2], exposure, image_sample)), "imu history",
                "interpolate the image boundary from retained nanosecond samples");
    test.expect(image_sample.specific_force(0) == 3 && image_sample.angular_rate(1) == 1.5 && history.entries() == 4,
                "imu history", "interpolation preserves history for later replay");
    msckf::ImuSample<L> next;
    next.time_ns = epoch + 20'000'000;
    next.specific_force(0) = 8;
    next.angular_rate(1) = 4;
    test.expect(!history.push(next) && history[0].time_ns == epoch, "imu history",
                "full history does not overwrite pending data");

    // The owner has now released only the first sample; keep the left image bracket.
    test.expect(history.discard(1) && history.push(next), "imu history", "explicit retirement makes room");
    msckf::ImuSample<L> replay_sample;
    test.expect(succeeded(msckf::interpolate(history[0], history[1], exposure, replay_sample)), "imu history",
                "the same interpolation bracket survives physical wrap");
    test.expect(replay_sample.time_ns == exposure && replay_sample.specific_force(0) == 3 &&
                    replay_sample.angular_rate(1) == 1.5,
                "imu history", "repeat read produces the same raw boundary");

    linalg::Vector<L, 3> ba, bg;
    ba(0) = 1;
    msckf::ImuInterval<L> interval;
    test.expect(succeeded(msckf::prepare_interval(replay_sample, history[1], ba, bg, interval)) &&
                    interval.a(0) == 2.5 && interval.start_ns == exposure && interval.end_ns == epoch + 10'000'000,
                "imu history", "prepare a segment ending at the next raw sample");
    ba(0) = 2;
    test.expect(succeeded(msckf::prepare_interval(replay_sample, history[1], ba, bg, interval)) &&
                    interval.a(0) == 1.5 && history[0].specific_force(0) == 2 && history[1].specific_force(0) == 4,
                "imu history", "updated bias is applied without modifying retained measurements");
    test.expect(msckf::interpolate(history[0], history[1], epoch + 4'000'000, replay_sample) == Status::out_of_range &&
                    replay_sample.time_ns == exposure && replay_sample.specific_force(0) == 3 &&
                    replay_sample.angular_rate(1) == 1.5,
                "imu history", "retired coverage is not extrapolated");
}
} // namespace

int main()
{
    using namespace formal_gvins;
    using L = test::Backend<double>;
    test::configure_backend_test();
    test::Context t;
    retained_imu_interval(t);
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
    auto const saved = interval;
    right.angular_rate(0) = 0;
    left.specific_force(0) = std::numeric_limits<double>::max();
    right.specific_force(0) = std::numeric_limits<double>::max();
    ba(0) = -std::numeric_limits<double>::max();
    t.expect(msckf::prepare_interval(left, right, ba, bg, interval) == Status::non_finite_result &&
                 interval.start_ns == saved.start_ns && interval.end_ns == saved.end_ns &&
                 linalg::max_abs(interval.a - saved.a) == 0 && linalg::max_abs(interval.omega - saved.omega) == 0,
             "imu", "finite-input bias subtraction overflow preserves complete interval");
    return t.failures() == 0 ? 0 : 1;
}
