#pragma once

#include <bit>

namespace svo
{
template <typename T, size_t N>
struct bitmask
{
    T mask[N];

    constexpr void reset(size_t i) { mask[elem_index(i)] &= ~(T(1) << bit_index(i)); }

    constexpr void set(size_t i) { mask[elem_index(i)] |= T(1) << bit_index(i); }

    constexpr bool get(size_t i) const
    {
        return 0 != (mask[elem_index(i)] & (T(1) << bit_index(i)));
    }

    constexpr size_t popcount() const
    {
        size_t count = 0;
        for (size_t i = 0; i < N; i++)
            count += std::popcount(mask[i]);
        return count;
    }

    constexpr void clear()
    {
        for (size_t i = 0; i < N; i++)
            mask[i] = T(0);
    }

    constexpr size_t elem_index(size_t i) const { return i / bits_per_elem(); }

    constexpr static size_t bit_index(size_t i) { return i % bits_per_elem(); }

    constexpr static size_t bits_per_elem() { return (sizeof(T) * 8); }

    constexpr static size_t find_lsb_none = N * bits_per_elem();
    constexpr static size_t max_elems = N * bits_per_elem();
    constexpr size_t find_lsb(size_t start_i) const
    {
        if (start_i >= max_elems)
            return find_lsb_none;

        size_t elem_idx = elem_index(start_i);
        size_t bit_offset = bit_index(start_i);

        T current_mask = mask[elem_idx];
        if (bit_offset > 0)
        {
            current_mask &= (~T(0) << bit_offset);
        }

        if (current_mask != T(0))
        {
            return (elem_idx * bits_per_elem()) + std::countr_zero(current_mask);
        }

        for (size_t i = elem_idx + 1; i < N; ++i)
        {
            if (mask[i] != T(0))
            {
                return (i * bits_per_elem()) + std::countr_zero(mask[i]);
            }
        }

        return find_lsb_none;
    }

    constexpr size_t find_zero_lsb(size_t start_i) const
    {
        size_t elem_idx = elem_index(start_i);
        size_t bit_offset = bit_index(start_i);

        T current_mask = ~mask[elem_idx];
        if (bit_offset > 0)
        {
            current_mask &= (~T(0) << bit_offset);
        }

        if (current_mask != T(0))
        {
            return (elem_idx * bits_per_elem()) + std::countr_zero(current_mask);
        }

        for (size_t i = elem_idx + 1; i < N; ++i)
        {
            T current_elem_mask = ~mask[i];
            if (current_elem_mask != T(0))
            {
                return (i * bits_per_elem()) + std::countr_zero(current_elem_mask);
            }
        }

        return find_lsb_none;
    }

    constexpr size_t count_before(size_t i) const
    {
        const size_t elem_idx = elem_index(i);
        const size_t bit_offset = bit_index(i);

        size_t count = 0;

        for (size_t i = 0; i < elem_idx; ++i)
            count += glm::bitCount(mask[i]);

        if (bit_offset != 0)
            count += glm::bitCount(mask[elem_idx] & ((T(1) << bit_offset) - 1));

        return count;
    }

    constexpr void fill_mask(size_t begin, size_t end)
    {
        size_t begin_elem = elem_index(begin);
        size_t begin_bit = bit_index(begin);
        size_t end_elem = elem_index(end);
        size_t end_bit = bit_index(end);

        for (size_t i = begin_elem; i <= end_elem; i++)
        {
            T fill_mask = ~T(0);
            if (i == begin_elem)
            {
                fill_mask &= (~T(0)) << begin_bit;
            }
            if (i == end_elem)
            {
                T upper_mask = (end_bit == bits_per_elem() - 1) ? ~T(0)
                                                                : ((T(1) << (end_bit + 1)) - 1);

                fill_mask &= upper_mask;
            }
            mask[i] |= fill_mask;
        }
    }

    constexpr void fill_mask_zero(size_t begin, size_t end)
    {
        size_t begin_elem = elem_index(begin);
        size_t begin_bit = bit_index(begin);
        size_t end_elem = elem_index(end);
        size_t end_bit = bit_index(end);

        for (size_t i = begin_elem; i <= end_elem; i++)
        {
            T fill_mask = ~T(0);
            if (i == begin_elem)
            {
                fill_mask &= (~T(0)) << begin_bit;
            }
            if (i == end_elem)
            {
                T upper_mask = (end_bit == bits_per_elem() - 1) ? ~T(0)
                                                                : ((T(1) << (end_bit + 1)) - 1);

                fill_mask &= upper_mask;
            }
            mask[i] &= ~fill_mask;
        }
    }
};
} // namespace svo