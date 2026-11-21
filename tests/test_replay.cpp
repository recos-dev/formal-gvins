/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 */
#include <formal_gvins/io/replay.hpp>
#include <cassert>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace
{
using namespace std::chrono_literals;
using formal_gvins::io::Publication;
using formal_gvins::io::Replay;
using formal_gvins::io::SensorEvent;

class TestSource final : public formal_gvins::io::SensorSource
{
public:
    explicit TestSource(std::vector<std::uint64_t> offsets, std::uint64_t epoch = 1606691893000000001ULL)
        : m_offsets(std::move(offsets)), m_epoch(epoch)
    {
    }
    std::uint64_t start_time_ns() const override { return m_epoch; }
    std::vector<formal_gvins::io::SensorStream> const & streams() const override { return m_streams; }
    bool next(SensorEvent & event) override
    {
        if (m_index == fail_at)
        {
            throw std::runtime_error("Injected source failure");
        }
        if (m_index == m_offsets.size())
        {
            return false;
        }
        if (read_delay.count() > 0)
        {
            std::this_thread::sleep_for(read_delay);
        }
        SensorEvent value;
        value.sequence = m_index;
        value.record_time_ns = m_epoch + m_offsets[m_index];
        value.measurement = formal_gvins::io::ImuMeasurement{123U, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
        ++m_index;
        event = std::move(value);
        return true;
    }
    std::size_t fail_at = std::numeric_limits<std::size_t>::max();
    std::chrono::milliseconds read_delay{0};

private:
    std::vector<std::uint64_t> m_offsets;
    std::uint64_t m_epoch;
    std::size_t m_index = 0U;
    std::vector<formal_gvins::io::SensorStream> m_streams;
};

template <typename Function> void expect_error(Function function)
{
    bool failed = false;
    try
    {
        function();
    }
    catch (std::exception const &)
    {
        failed = true;
    }
    assert(failed);
}

void test_order_and_epoch()
{
    for (std::size_t capacity : {1U, 2U, 32U})
    {
        // Nanosecond differences at an epoch beyond exact double representation.
        TestSource source({0U, 0U, 1U, 2000000U, 4000000U});
        Replay replay(source, {capacity, 0ns});
        Publication result;
        result.event.sequence = 99U;
        expect_error([&] { replay.next(result); });
        replay.start();
        expect_error([&] { replay.start(); });
        std::size_t count = 0U;
        while (replay.next(result))
        {
            assert(result.event.sequence == count);
            assert(std::get<formal_gvins::io::ImuMeasurement>(result.event.measurement).time_ns == 123U);
            assert(result.event.source_record.empty());
            assert(result.scheduled.count() ==
                   static_cast<std::int64_t>(result.event.record_time_ns - source.start_time_ns()));
            assert(result.elapsed >= result.scheduled && result.lateness == result.elapsed - result.scheduled);
            ++count;
        }
        assert(count == 5U && result.event.sequence == 4U);
        assert(!replay.next(result));
    }
}

void test_deadlines()
{
    using Clock = std::chrono::steady_clock;
    using formal_gvins::io::publication_deadline;
    auto const epoch = Clock::time_point(100ns);
    assert(publication_deadline(epoch, 0ns) == epoch);
    assert(publication_deadline(epoch, 7ns) == Clock::time_point(107ns));
    assert(publication_deadline(Clock::time_point::max(), 0ns) == Clock::time_point::max());
    assert(publication_deadline(Clock::time_point::max() - 1ns, 1ns) == Clock::time_point::max());
    assert(publication_deadline(Clock::time_point::min(), 1ns) == Clock::time_point::min() + 1ns);
    assert(publication_deadline(Clock::time_point(-1ns), std::chrono::nanoseconds::max()) ==
           Clock::time_point::max() - 1ns);
    expect_error([&] { publication_deadline(epoch, -1ns); });
    expect_error([&] { publication_deadline(Clock::time_point::max(), 1ns); });
}

void test_stop()
{
    TestSource source({0U, 1000000000U});
    Replay replay(source);
    replay.start();
    Publication result;
    assert(replay.next(result));
    std::thread stopper(
        [&]
        {
            std::this_thread::sleep_for(5ms);
            replay.stop();
        });
    assert(!replay.next(result));
    stopper.join();
    assert(result.event.sequence == 0U);
    assert(!replay.next(result));
    TestSource unused({0U});
    Replay stopped(unused);
    stopped.stop();
    expect_error([&] { stopped.start(); });
}

void test_error_and_lateness()
{
    TestSource broken({0U});
    broken.fail_at = 0U;
    Replay failure(broken);
    expect_error([&] { failure.start(); });
    Publication result;
    result.event.sequence = 99U;
    expect_error([&] { failure.next(result); });
    assert(result.event.sequence == 99U);

    TestSource later_failure({0U, 1000U});
    later_failure.fail_at = 1U;
    Replay failed_read(later_failure, {1U, 0ns});
    failed_read.start();
    bool saw_failure = false;
    while (!saw_failure)
    {
        auto const saved_sequence = result.event.sequence;
        try
        {
            assert(failed_read.next(result));
        }
        catch (std::exception const &)
        {
            assert(result.event.sequence == saved_sequence);
            saw_failure = true;
        }
    }
    result.event.sequence = 99U;

    TestSource source({0U});
    Replay late(source, {1U, 1ms});
    late.start();
    std::this_thread::sleep_for(3ms);
    expect_error([&] { late.next(result); });
    expect_error([&] { late.next(result); });
    assert(result.event.sequence == 99U);

    TestSource reversed({0U, 200U, 100U});
    Replay invalid(reversed);
    expect_error([&] { invalid.start(); });
    TestSource overflow({0U, std::numeric_limits<std::uint64_t>::max()}, 0U);
    Replay unrepresentable(overflow);
    expect_error([&] { unrepresentable.start(); });
    TestSource deadline_overflow({0U, static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())}, 0U);
    Replay out_of_clock_range(deadline_overflow);
    out_of_clock_range.start();
    assert(out_of_clock_range.next(result));
    expect_error([&] { out_of_clock_range.next(result); });
}

void test_empty_and_initial_loading()
{
    TestSource empty({});
    Replay replay(empty);
    replay.start();
    Publication result;
    assert(!replay.next(result));
    TestSource large_epoch({0U, 1U}, std::numeric_limits<std::uint64_t>::max() - 1U);
    Replay near_limit(large_epoch);
    near_limit.start();
    assert(near_limit.next(result) && result.scheduled == 0ns);
    assert(near_limit.next(result) && result.scheduled == 1ns);
    assert(!near_limit.next(result));
    TestSource slow({0U, 1000000000U});
    slow.read_delay = 3ms;
    Replay loaded(slow, {2U, 0ns});
    loaded.start();
    assert(loaded.next(result) && result.scheduled == 0ns);
    // No host latency upper bound is asserted; startup I/O completes before epoch capture.
    loaded.stop();
    expect_error([&] { Replay invalid(slow, {0U, 0ns}); });
    expect_error([&] { Replay invalid(slow, {4097U, 0ns}); });
    expect_error([&] { Replay invalid(slow, {1U, -1ns}); });
}
} // namespace

int main()
{
    test_order_and_epoch();
    test_deadlines();
    test_stop();
    test_error_and_lateness();
    test_empty_and_initial_loading();
    std::cout << "Local sensor replay: pass\n";
}
