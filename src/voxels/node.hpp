#pragma once

#include <cstdint>
#include <limits>

#include "bitmask.hpp"

namespace svo
{

/**
 *
 * @brief a GPU-friendly generic tree node structure.
 *
 * memory layout:
 *
 * [DataT data, ChildmaskT childmask, OffsetT first_child, OffsetT parent]
 *
 *  * members:
 *
 * - childmask: bitmask where each bit indicates presence of a child
 *
 * - firstchild: offset into flat node array
 *               (0xFFFFFFFF if no children or invalid)
 *
 * - parent: offset into flat node array
 *               (0xFFFFFFFF if no children or invalid)
 *
 * @tparam DataT - your custom data (POD recommended)
 * @tparam ChildmaskT - underlying type for children mask
 * @tparam OffsetT - underlying type children and parent offset
 *
 */
template <typename DataT, typename ChildmaskT, size_t ChildmaskN, typename OffsetT,
          size_t MaxChildren>
struct sparse_tree_node
{
    using child_offset_t = OffsetT;
    using child_mask_t = ChildmaskT;
    using bitmask_t = bitmask<child_mask_t, ChildmaskN>;
    using data_t = DataT;

    constexpr static size_t max_children = MaxChildren;
    constexpr static size_t morton_length = std::bit_width(max_children - 1);
    constexpr static child_offset_t invalid_offset = std::numeric_limits<child_offset_t>::max();

    data_t data;               /// your custom data, supports any POD
    bitmask_t childmask;       /// nth bit represents nth child presence
    child_offset_t firstchild; /// offset into the first child in an array
    child_offset_t parent;     /// offset into the parent in an array

    inline void reset_bit(size_t i) { childmask.reset(i); }

    inline void set_bit(size_t i) { childmask.set(i); }

    inline bool get_bit(size_t i) { return childmask.get(i); }

    inline void clear_bits() { childmask.clear(); }

    inline size_t child_count() { return childmask.popcount(); }

    inline size_t find_lsb(size_t start_i) { return childmask.find_lsb(start_i); }
};

template <typename NodeT>
typename NodeT::child_offset_t get_child_offset(NodeT& node, size_t bit_index)
{
    return node.childmask.count_before(bit_index);
}

template <typename NodeT>
typename NodeT::child_offset_t get_child_ptr(NodeT& node, size_t bit_index)
{
    return node.firstchild + get_child_offset(node, bit_index);
}

/**
 * @brief tree_node variant with 2x32-bit mask and 32-bit index.
 *
 * Equivalent to:
 * sparse_tree_node<DataT, uint32_t, 2, uint32_t, 64>
 *
 * @tparam DataT payload type (POD recommended)
 */
template <typename DataT>
using contree_node = sparse_tree_node<DataT, uint32_t, 2, uint32_t, 64>;

/// 64 bit firstchild offset variant
template <typename DataT>
using contree_node_o64u = sparse_tree_node<DataT, uint64_t, 1, uint64_t, 64>;
/// 32 bit firstchild offset variant
template <typename DataT>
using contree_node_o32u = sparse_tree_node<DataT, uint64_t, 1, uint32_t, 64>;
/// 16 bit firstchild offset variant
template <typename DataT>
using contree_node_o16u = sparse_tree_node<DataT, uint64_t, 1, uint16_t, 64>;
/// 8 bit firstchild offset variant
template <typename DataT>
using contree_node_o8u = sparse_tree_node<DataT, uint64_t, 1, uint8_t, 64>;

/// 64 bit firstchild offset variant
template <typename DataT>
using octree_node_o64u = sparse_tree_node<DataT, uint8_t, 1, uint64_t, 8>;
/// 32 bit firstchild offset variant
template <typename DataT>
using octree_node_o32u = sparse_tree_node<DataT, uint8_t, 1, uint32_t, 8>;
/// 16 bit firstchild offset variant
template <typename DataT>
using octree_node_o16u = sparse_tree_node<DataT, uint8_t, 1, uint16_t, 8>;
/// 8 bit firstchild offset variant
template <typename DataT>
using octree_node_o8u = sparse_tree_node<DataT, uint8_t, 1, uint8_t, 8>;

template <typename DataT>
using quadtree_node = sparse_tree_node<DataT, uint8_t, 1, uint32_t, 4>;

/// 64 bit firstchild offset variant
template <typename DataT>
using quadtree_node_o64u = sparse_tree_node<DataT, uint8_t, 1, uint64_t, 4>;
/// 32 bit firstchild offset variant
template <typename DataT>
using quadtree_node_o32u = sparse_tree_node<DataT, uint8_t, 1, uint32_t, 4>;
/// 16 bit firstchild offset variant
template <typename DataT>
using quadtree_node_o16u = sparse_tree_node<DataT, uint8_t, 1, uint16_t, 4>;
/// 8 bit firstchild offset variant
template <typename DataT>
using quadtree_node_o8u = sparse_tree_node<DataT, uint8_t, 1, uint8_t, 4>;
} // namespace svo

using NodeT = svo::contree_node<uint32_t>;
