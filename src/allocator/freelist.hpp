#pragma once

#include <bit>
#include <format>
#include <iostream>
#include <queue>
#include <vector>

#ifndef SVO_FREELIST_MAX_ALLOC_COUNT
#define SVO_FREELIST_MAX_ALLOC_COUNT 64
#endif

template <typename OffsetT, size_t max_alloc_size>
class FreeList
{
public:
    using offset_t = OffsetT;

    constexpr size_t get_max_alloc_size() { return max_alloc_size; }

    struct OverflowBlock
    {
        offset_t offset;
        offset_t size;
    };

    inline static constexpr offset_t null = std::numeric_limits<offset_t>::max();
    offset_t alloc(size_t size)
    {
        size_t free_index = m_bitmask.find_lsb(size - 1);
        if (free_index == m_bitmask.find_lsb_none)
            return get_overflow_free(size);

        offset_t fast_free_offset = m_fast_free[free_index][m_fast_free_counts[free_index] - 1];
        m_fast_free_counts[free_index]--;

        if (m_fast_free_counts[free_index] == 0)
            m_bitmask.reset(free_index);

        size_t free_size = free_index + 1;
        if (free_size > size)
            free(fast_free_offset + size, free_size - size);

        return fast_free_offset;
    }

    void free(offset_t offset, size_t size)
    {
        if (m_fast_free_counts[size - 1] >= SVO_FREELIST_MAX_ALLOC_COUNT)
        {
            m_overflow.emplace_back(offset, size);
            return;
        }
        if (m_fast_free_counts[size - 1] == 0)
            m_bitmask.set(size - 1);
        m_fast_free[size - 1][m_fast_free_counts[size - 1]] = offset;
        m_fast_free_counts[size - 1]++;
    }

private:
    offset_t get_overflow_free(size_t size)
    {
        for (auto it = m_overflow.begin(); it != m_overflow.end(); ++it)
        {
            OverflowBlock block = *it;
            if (block.size < size)
                continue;

            if (block.size == size)
            {
                *it = m_overflow.back();
                m_overflow.pop_back();
                return block.offset;
            }

            it->offset += size;
            it->size -= size;

            return block.offset;
        }
        return null;
    }

    offset_t m_fast_free[max_alloc_size][SVO_FREELIST_MAX_ALLOC_COUNT];
    size_t m_fast_free_counts[max_alloc_size] = {};

    static constexpr size_t c_word_count = (max_alloc_size + 63) / 64;
    svo::bitmask<uint64_t, c_word_count> m_bitmask;

    std::vector<OverflowBlock> m_overflow;
};
