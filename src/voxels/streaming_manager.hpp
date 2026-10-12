#pragma once

#include "allocator/page_allocator.hpp"
#include "voxels/builder.hpp"
#include "voxels/node.hpp"

#include "core.hpp"

#include <mutex>
#include <vector>

using NodeT = svo::contree_node<uint32_t>;
class VoxelStreamingManager
{
public:
    void init(uint32_t num_nodes, uint32_t page_size)
    {
        m_timestamp = 0;
        m_page_size = page_size;

        uint32_t num_pages = (num_nodes + page_size - 1) / page_size;
        m_allocator.set_page_count(num_pages);
        m_page_lru.resize(num_pages, m_timestamp);
        m_page_parent_index.resize(num_pages, UINT32_MAX);
        m_nodes.resize(num_nodes);

        NodeT root_node;
        root_node.childmask.clear();
        root_node.data = 0xff00ff;
        root_node.firstchild = UINT32_MAX;
        root_node.parent = UINT32_MAX;

        m_nodes[0] = root_node;
        allocated_pages++;

        GLbitfield flags = GL_DYNAMIC_STORAGE_BIT | GL_MAP_WRITE_BIT;
        m_nodes_ssbo.allocate(m_nodes.data(), sizeof(NodeT) * num_nodes, flags);
    }
    int allocated_pages = 0;
    void update_lru(uint32_t timestamp, const std::vector<uint32_t>& nodes)
    {
        for (uint32_t node : nodes)
        {
            while (node != UINT32_MAX)
            {
                m_page_lru[page_of_node(node)] = timestamp;
                node = m_nodes[node].parent;
            }
        }
    }

    uint32_t page_of_node(uint32_t node) { return node / m_page_size; }
    void update(uint32_t timestamp) { m_timestamp = timestamp; }
    int32_t page_age(uint32_t page_id)
    {
        return static_cast<int32_t>(m_timestamp) - static_cast<int32_t>(m_page_lru[page_id]);
    }
    uint32_t allocate_page()
    {
        for (uint32_t i = 0; i < 32; i++)
        {
            uint32_t page_id = m_victim_cursor;
            int32_t age = page_age(page_id);
            if (age > 32)
            {
                unlink_page(page_id);

                m_victim_cursor = (m_victim_cursor + 1) % m_page_lru.size();
                m_page_lru[page_id] = m_timestamp;

                return page_id;
            }
            m_victim_cursor = (m_victim_cursor + 1) % m_page_lru.size();
        }

        return UINT32_MAX;
    }

    template <typename PFNCreate, typename PFNHomogeneous>
    void process_subdivision_requests(const GenerateInfo<PFNCreate, PFNHomogeneous>& gen_info,
                                      uint32_t* node_indices, uint32_t num_node_indices)
    {
        printf("REQ");
        bool should_allocate = true;
        uint32_t page_id = UINT32_MAX;
        uint32_t num_success_requests = 0;
        for (uint32_t i = 0; i < num_node_indices; i++)
        {
            uint32_t node_idx = node_indices[i];

            if (m_nodes[node_idx].child_count() > 0)
                continue;

            BuildInfo build_info = get_build_info(node_idx);

            if (build_info.depth >= 4)
                continue;

            if (should_allocate)
            {
                page_id = allocate_page();
                if (page_id == UINT32_MAX)
                {
                    should_allocate = true;
                    continue;
                }
                should_allocate = false;
            }

            uint32_t child_idx = page_id * m_page_size;
            if (subdivide_node(build_info, gen_info, node_idx, child_idx, m_nodes.data(), 4))
            {
                num_success_requests++;
                m_page_parent_index[page_id] = node_idx;
                m_page_lru[page_id] = m_timestamp;
                should_allocate = true;
                allocated_pages++;
            }
        }

        upload_to_gpu();
        printf("UESTS\n");
    }

    void bind_buffer(uint32_t nodes_bind_idx) { m_nodes_ssbo.bind_base(nodes_bind_idx); }
    uint32_t page_count() { return m_page_lru.size(); }

    glm::ivec4 coord_of_node(uint32_t node_idx)
    {
        glm::ivec4 coord(0);

        uint32_t shift = 0;
        while (node_idx != UINT32_MAX)
        {
            uint32_t parent_idx = m_nodes[node_idx].parent;

            if (parent_idx == UINT32_MAX)
                return coord;

            uint32_t parent_first_child = m_nodes[parent_idx].firstchild;
            uint32_t dense_offset = node_idx - parent_first_child;
            auto bitmask = m_nodes[parent_idx].childmask;

            uint32_t bit_index = 0;
            for (int i = 0; i <= dense_offset; i++)
            {
                bit_index = bitmask.find_lsb(bit_index);
                if (bit_index == bitmask.find_lsb_none)
                    break;
                if (i < dense_offset)
                {
                    bitmask.reset(bit_index);
                }
            }

            coord.x |= ((bit_index / 1) % 4) << shift;
            coord.y |= ((bit_index / 4) % 4) << shift;
            coord.z |= ((bit_index / 16) % 4) << shift;
            coord.w++;
            shift += 2;

            node_idx = parent_idx;
        }
        return coord;
    }

    void upload_to_gpu()
    {
        m_nodes_ssbo.set_sub_data(m_nodes.data(), m_nodes.size() * sizeof(NodeT), 0);
    }

    BuildInfo get_build_info(uint32_t node_idx)
    {
        BuildInfo info;

        glm::ivec4 coord = coord_of_node(node_idx);
        info.chunk_size = 64.f / glm::pow(4.f, coord.w);
        info.chunk_origin = glm::vec3(coord) * info.chunk_size;
        info.depth = coord.w;

        return info;
    }

    void unlink_page(uint32_t page_id)
    {
        uint32_t parent_node = m_page_parent_index[page_id];
        if (parent_node != NodeT::invalid_offset)
        {
            allocated_pages--;
            m_nodes[parent_node].childmask.clear();
            m_nodes[parent_node].firstchild = NodeT::invalid_offset;

            uint32_t start_node_idx = page_id * m_page_size;
            uint32_t end_node_idx = std::min(start_node_idx + m_page_size,
                                             static_cast<uint32_t>(m_nodes.size()));

            for (uint32_t i = start_node_idx; i < end_node_idx; i++)
            {

                m_nodes[i].parent = UINT32_MAX;
                m_nodes[i].firstchild = NodeT::invalid_offset;
                m_nodes[i].childmask.clear();
            }

            m_page_parent_index[page_id] = UINT32_MAX;
        }
    }
    uint32_t m_timestamp = 0;
    uint32_t m_page_size;

    uint32_t m_victim_cursor = 0;
    PageAllocator m_allocator;

    std::vector<uint32_t> m_page_lru;
    std::vector<uint32_t> m_page_parent_index;

    std::vector<NodeT> m_nodes;

    Ssbo m_nodes_ssbo;
};