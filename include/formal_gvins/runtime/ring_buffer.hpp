/*
 * formal-gvins is licensed under the Apache License, Version 2.0.
 * See LICENSE for usage and redistribution terms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <cstddef>

namespace formal_gvins::runtime
{

/**
 * Single-owner, fixed-capacity storage. Full pushes fail without overwriting.
 * Samples are bounded values whose construction/copy does not allocate, throw
 * or block. The owner admits inputs and decides when readers no longer need a
 * prefix. No timestamp convention, synchronization or sensor policy is owned.
 * See R-RING-BUFFER for access lifetime and bounded-work contracts.
 */
template <typename Sample, std::size_t Capacity> class RingBuffer
{
    static_assert(Capacity > 0);

public:
    RingBuffer() = default;
    RingBuffer(RingBuffer const &) = delete;
    RingBuffer & operator=(RingBuffer const &) = delete;
    RingBuffer(RingBuffer &&) = delete;
    RingBuffer & operator=(RingBuffer &&) = delete;

    /** Append one sample; false preserves the entire buffer. */
    [[nodiscard]] bool push(Sample const & sample);

    /** Remove count oldest samples. A count greater than entries() changes nothing. */
    [[nodiscard]] bool discard(std::size_t count);

    /** Logical oldest-to-newest access; requires index < entries(). No consumption. */
    Sample const & operator[](std::size_t index) const { return m_samples[slot(index)]; }

    std::size_t entries() const { return m_count; }
    static constexpr std::size_t capacity() { return Capacity; }
    bool empty() const { return m_count == 0; }
    bool full() const { return m_count == Capacity; }

    /** Clear every slot, position and count. No heap allocation. */
    void reset();

private:
    /** Requires offset <= Capacity. Subtract before adding to avoid index overflow. */
    std::size_t slot(std::size_t offset) const;

    std::array<Sample, Capacity> m_samples{};
    std::size_t m_oldest = 0;
    std::size_t m_count = 0;
}; /* end class RingBuffer */

template <typename Sample, std::size_t Capacity>
std::size_t RingBuffer<Sample, Capacity>::slot(std::size_t offset) const
{
    std::size_t const remaining = Capacity - m_oldest;
    return offset < remaining ? m_oldest + offset : offset - remaining;
}

template <typename Sample, std::size_t Capacity> bool RingBuffer<Sample, Capacity>::push(Sample const & sample)
{
    if (full())
    {
        return false;
    }
    m_samples[slot(m_count)] = sample;
    ++m_count;
    return true;
}

template <typename Sample, std::size_t Capacity> bool RingBuffer<Sample, Capacity>::discard(std::size_t count)
{
    if (count > m_count)
    {
        return false;
    }
    m_oldest = slot(count);
    m_count -= count;
    return true;
}

template <typename Sample, std::size_t Capacity> void RingBuffer<Sample, Capacity>::reset()
{
    m_samples.fill(Sample{});
    m_oldest = 0;
    m_count = 0;
}

} /* end namespace formal_gvins::runtime */
