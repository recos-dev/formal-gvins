/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 */
#pragma once
#include <formal_gvins/io/source.hpp>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>

namespace formal_gvins::io
{

/** Map an already computed nonnegative offset to an absolute deadline.
 * No clock reads or waits; throws on a negative offset or clock-range overflow. */
std::chrono::steady_clock::time_point publication_deadline(std::chrono::steady_clock::time_point epoch,
                                                           std::chrono::nanoseconds offset);

struct ReplayOptions
{
    std::size_t prefetch_events = 32U;
    // Zero reports lateness without enforcing a budget. Positive values stop
    // before delivering an overdue event; they never alter timestamps or drop it.
    std::chrono::nanoseconds maximum_lateness{0};
};

struct Publication
{
    SensorEvent event;
    std::chrono::nanoseconds scheduled{0}; // relative to start(), from recorded arrival
    std::chrono::nanoseconds elapsed{0};   // local delivery time relative to start()
    std::chrono::nanoseconds lateness{0};
};

/** Local 1x replay. One loader worker, caller-owned publication loop.
 * Source is borrowed exclusively and must outlive this object. start()/next()
 * have one caller; stop() may be called concurrently. No restart or seeking.
 * start() prefills the bounded queue before capturing the simulated boot epoch.
 * next() returns false on EOF/stop and throws on source/schedule failure, preserving
 * output in either case. Stop can interrupt waits but not an in-flight filesystem
 * read. Destruction joins the loader. This is host pacing, not a hard RT guarantee. */
class Replay
{
public:
    explicit Replay(SensorSource & source, ReplayOptions const & options = {});
    ~Replay();
    Replay(Replay const &) = delete;
    Replay & operator=(Replay const &) = delete;

    void start();
    bool next(Publication & output);
    void stop();

private:
    void load();

    SensorSource & m_source;
    ReplayOptions m_options;
    std::uint64_t m_record_origin;
    std::vector<SensorEvent> m_queue;
    std::size_t m_head = 0U;
    std::size_t m_count = 0U;
    bool m_started = false;
    bool m_done = false;
    bool m_stopped = false;
    std::exception_ptr m_error;
    std::chrono::steady_clock::time_point m_epoch{};
    std::mutex m_mutex;
    std::condition_variable m_changed;
    std::thread m_loader;
};
} // namespace formal_gvins::io
