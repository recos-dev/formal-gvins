/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 */
#include <formal_gvins/io/replay.hpp>
#include <limits>
#include <stdexcept>
#include <utility>
#include <type_traits>

namespace formal_gvins::io
{
namespace
{
std::size_t queue_capacity(ReplayOptions const & options)
{
    if (options.prefetch_events == 0U || options.prefetch_events > 4096U || options.maximum_lateness.count() < 0)
    {
        throw std::invalid_argument("Invalid replay capacity or lateness budget");
    }
    return options.prefetch_events;
}
} // namespace

std::chrono::steady_clock::time_point publication_deadline(std::chrono::steady_clock::time_point epoch,
                                                           std::chrono::nanoseconds offset)
{
    // Supported host clocks use nanosecond durations. Make the arithmetic
    // premise explicit instead of overflowing a duration_cast on another clock.
    static_assert(std::is_same_v<std::chrono::steady_clock::duration, std::chrono::nanoseconds>);
    if (offset.count() < 0)
    {
        throw std::invalid_argument("Negative publication offset");
    }
    auto const maximum = std::chrono::steady_clock::time_point::max().time_since_epoch();
    if (epoch.time_since_epoch() > maximum - offset)
    {
        throw std::overflow_error("Replay deadline overflow");
    }
    return epoch + offset;
}

Replay::Replay(SensorSource & source, ReplayOptions const & options)
    : m_source(source), m_options(options), m_record_origin(source.start_time_ns()), m_queue(queue_capacity(options))
{
}

Replay::~Replay()
{
    stop();
    if (m_loader.joinable())
    {
        m_loader.join();
    }
}

void Replay::start()
{
    std::unique_lock lock(m_mutex);
    if (m_started || m_stopped)
    {
        throw std::logic_error("Replay is single-use");
    }
    m_started = true;
    try
    {
        m_loader = std::thread(&Replay::load, this);
    }
    catch (...)
    {
        m_error = std::current_exception();
        throw;
    }
    m_changed.wait(lock, [this] { return m_count == m_queue.size() || m_done || m_error || m_stopped; });
    if (m_error)
    {
        std::rethrow_exception(m_error);
    }
    m_epoch = std::chrono::steady_clock::now();
}

void Replay::load()
{
    try
    {
        std::uint64_t last_time = m_record_origin;
        std::uint64_t sequence = 0U;
        while (true)
        {
            {
                std::unique_lock lock(m_mutex);
                m_changed.wait(lock, [this] { return m_count < m_queue.size() || m_stopped; });
                if (m_stopped)
                {
                    return;
                }
            }
            SensorEvent event;
            if (!m_source.next(event))
            {
                std::lock_guard lock(m_mutex);
                m_done = true;
                m_changed.notify_all();
                return;
            }
            if (event.sequence != sequence || (sequence == 0U && event.record_time_ns != m_record_origin) ||
                event.record_time_ns < last_time ||
                event.record_time_ns - m_record_origin >
                    static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
            {
                throw std::runtime_error("Invalid replay event order or time range");
            }
            last_time = event.record_time_ns;
            ++sequence;
            {
                std::lock_guard lock(m_mutex);
                if (m_stopped)
                {
                    return;
                }
                auto const tail = (m_head + m_count) % m_queue.size();
                std::swap(m_queue[tail], event);
                ++m_count;
                m_changed.notify_all();
            }
        }
    }
    catch (...)
    {
        std::lock_guard lock(m_mutex);
        m_error = std::current_exception();
        m_changed.notify_all();
    }
}

bool Replay::next(Publication & output)
{
    Publication result;
    std::unique_lock lock(m_mutex);
    if (!m_started)
    {
        throw std::logic_error("Call start() before next()");
    }
    m_changed.wait(lock, [this] { return m_count != 0U || m_done || m_error || m_stopped; });
    if (m_error)
    {
        std::rethrow_exception(m_error);
    }
    if (m_stopped || m_count == 0U)
    {
        return false;
    }
    std::swap(result.event, m_queue[m_head]);
    m_head = (m_head + 1U) % m_queue.size();
    --m_count;
    m_changed.notify_all();
    auto const offset = result.event.record_time_ns - m_record_origin;
    result.scheduled = std::chrono::nanoseconds(static_cast<std::int64_t>(offset));
    std::chrono::steady_clock::time_point deadline;
    try
    {
        deadline = publication_deadline(m_epoch, result.scheduled);
    }
    catch (...)
    {
        m_error = std::current_exception();
        m_stopped = true;
        m_changed.notify_all();
        throw;
    }
    m_changed.wait_until(lock, deadline, [this] { return m_stopped || m_error; });
    if (m_error)
    {
        std::rethrow_exception(m_error);
    }
    if (m_stopped)
    {
        return false;
    }
    result.elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - m_epoch);
    result.lateness = result.elapsed - result.scheduled;
    if (m_options.maximum_lateness.count() > 0 && result.lateness > m_options.maximum_lateness)
    {
        m_error = std::make_exception_ptr(std::runtime_error("Replay lateness budget exceeded"));
        m_stopped = true;
        m_changed.notify_all();
        std::rethrow_exception(m_error);
    }
    lock.unlock();
    output = std::move(result);
    return true;
}

void Replay::stop()
{
    std::lock_guard lock(m_mutex);
    m_stopped = true;
    m_changed.notify_all();
}
} // namespace formal_gvins::io
