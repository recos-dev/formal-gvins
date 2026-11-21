/* SPDX-License-Identifier: Apache-2.0 */
#include <formal_gvins/runtime/ring_buffer.hpp>
#include "test_support.hpp"
#include <cstdint>
#include <deque>
#include <limits>
#include <type_traits>
#include <utility>

namespace
{
using formal_gvins::runtime::RingBuffer;
using formal_gvins::test::Context;

struct Sample
{
    std::uint64_t time_ns{};
    std::uint64_t value{};
    bool operator==(Sample const &) const = default;
};

void boundaries(Context & test)
{
    using Buffer = RingBuffer<Sample, 3>;
    static_assert(!std::is_copy_constructible_v<Buffer> && !std::is_copy_assignable_v<Buffer>);
    static_assert(!std::is_move_constructible_v<Buffer> && !std::is_move_assignable_v<Buffer>);
    static_assert(std::is_same_v<decltype(std::declval<Buffer &>()[0]), Sample const &>);
    static_assert(sizeof(Buffer) >= Buffer::capacity() * sizeof(Sample));

    Buffer buffer;
    test.expect(buffer.empty() && !buffer.full() && buffer.entries() == 0 && buffer.capacity() == 3, "ring",
                "construction is ready and empty");
    test.expect(buffer.discard(0) && !buffer.discard(1), "ring", "empty discard bounds");
    auto const maximum = std::numeric_limits<std::uint64_t>::max();
    Sample const a{maximum - 2, 11}, b{maximum - 1, 22}, c{maximum, 33}, d{0, 44};
    test.expect(buffer.push(a) && buffer.push(b) && buffer.push(c), "ring", "fill every slot");
    test.expect(buffer.full() && !buffer.push(d), "ring", "full insertion rejected");
    test.expect(!buffer.discard(4) && !buffer.discard(std::numeric_limits<std::size_t>::max()), "ring",
                "invalid counts rejected before arithmetic");
    test.expect(buffer.entries() == 3 && buffer[0] == a && buffer[1] == b && buffer[2] == c, "ring",
                "failures preserve all retained samples and nanoseconds");
    auto const * first = &buffer[0];
    test.expect(buffer.discard(0) && &buffer[0] == first, "ring", "zero discard is a no-op");
    test.expect(buffer.discard(1) && buffer.push(d), "ring", "retire then wrap");
    test.expect(buffer[0] == b && buffer[1] == c && buffer[2] == d, "ring",
                "logical order survives physical wrap; timestamp policy belongs to owner");
    test.expect(buffer.discard(2) && buffer.push(buffer[0]), "ring", "a retained sample may be copied on push");
    test.expect(buffer.entries() == 2 && buffer[0] == d && buffer[1] == d, "ring", "borrowed input not overwritten");
    test.expect(buffer.discard(2) && buffer.empty() && buffer.push(a) && buffer[0] == a, "ring",
                "drained nonzero ring position can refill");
    buffer.reset();
    test.expect(buffer.empty() && buffer.push(c) && buffer[0] == c, "ring", "reset starts a new history");
}

template <std::size_t Capacity> void sequences(Context & test)
{
    RingBuffer<std::uint64_t, Capacity> buffer;
    std::deque<std::uint64_t> oracle;
    auto verify = [&]()
    {
        test.expect(buffer.entries() == oracle.size() && buffer.empty() == oracle.empty() &&
                        buffer.full() == (oracle.size() == Capacity),
                    "sequence", "occupancy matches independent linear sequence");
        for (std::size_t i = 0; i < oracle.size() && i < buffer.entries(); ++i)
        {
            test.expect(buffer[i] == oracle[i], "sequence", "every retained value matches the sequence");
        }
    };
    auto append = [&](std::uint64_t value)
    {
        bool const expected = oracle.size() < Capacity;
        test.expect(buffer.push(value) == expected, "sequence", "push result");
        if (expected)
        {
            oracle.push_back(value);
        }
        verify();
    };
    auto discard = [&](std::size_t count)
    {
        bool const expected = count <= oracle.size();
        test.expect(buffer.discard(count) == expected, "sequence", "discard result");
        if (expected)
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                oracle.pop_front();
            }
        }
        verify();
    };

    verify();
    for (std::size_t i = 0; i <= Capacity; ++i)
    {
        append(i);
    }
    for (std::size_t i = 0; i < 3 * Capacity; ++i)
    {
        discard(1);
        append(Capacity + i);
    }
    discard(Capacity);
    discard(Capacity + 1);
    std::uint32_t seed = 271828;
    for (std::size_t step = 0; step < 5000; ++step)
    {
        seed = 1664525U * seed + 1013904223U;
        if (step % 127 == 0)
        {
            buffer.reset();
            oracle.clear();
            verify();
        }
        else if (seed % 5 < 3)
        {
            append(seed);
        }
        else
        {
            discard((seed >> 8) % (Capacity + 2));
        }
    }
}

struct CountedSample
{
    std::uint64_t value{};
    static inline std::size_t assignments{};
    static inline std::size_t cleared{};

    CountedSample & operator=(CountedSample const & other)
    {
        value = other.value;
        ++assignments;
        cleared += value == 0 ? 1 : 0;
        return *this;
    }
};

void bounded_work(Context & test)
{
    RingBuffer<CountedSample, 3> buffer;
    CountedSample::assignments = 0;
    CountedSample::cleared = 0;
    test.expect(buffer.push({1}) && buffer.push({2}) && buffer.push({3}), "work", "fill counted payloads");
    test.expect(CountedSample::assignments == 3, "work", "one payload assignment per successful push");
    test.expect(!buffer.push({4}) && !buffer.discard(4), "work", "reject invalid operations");
    test.expect(buffer.discard(2) && buffer[0].value == 3 && CountedSample::assignments == 3, "work",
                "reads, rejected operations and prefix discard do not copy samples");
    buffer.reset();
    test.expect(buffer.empty() && CountedSample::assignments == 6 && CountedSample::cleared == 3, "work",
                "reset clears the entire backing array including retired slots");
}
} // namespace

int main()
{
    Context test;
    boundaries(test);
    sequences<1>(test);
    sequences<3>(test);
    sequences<8>(test);
    sequences<257>(test);
    bounded_work(test);
    return test.failures() == 0 ? 0 : 1;
}
