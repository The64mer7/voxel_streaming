#pragma once

#include "allocator/page_allocator.hpp"
#include "graphics/camera.hpp"
#include "voxels/builder.hpp"
#include "voxels/node.hpp"

#include "core.hpp"

#include <mutex>
#include <vector>

static bool is_aabb_in_frustum(const glm::mat4& view_matrix, const glm::mat4& projection_matrix,
                               const glm::vec3& aabb_origin, float aabb_size)
{
    glm::mat4 vp = projection_matrix * view_matrix;

    std::array<glm::vec4, 6> planes;

    planes[0] = vp[3] + vp[0]; // Left
    planes[1] = vp[3] - vp[0]; // Right
    planes[2] = vp[3] + vp[1]; // Bottom
    planes[3] = vp[3] - vp[1]; // Top
    planes[4] = vp[3] + vp[2]; // Near
    planes[5] = vp[3] - vp[2]; // Far

    glm::vec3 aabb_max = aabb_origin + glm::vec3(aabb_size);

    for (int i = 0; i < 6; i++)
    {
        glm::vec3 normal = glm::vec3(planes[i]);
        float d = planes[i].w;

        glm::vec3 p;
        p.x = (normal.x >= 0.0f) ? aabb_max.x : aabb_origin.x;
        p.y = (normal.y >= 0.0f) ? aabb_max.y : aabb_origin.y;
        p.z = (normal.z >= 0.0f) ? aabb_max.z : aabb_origin.z;

        if (glm::dot(normal, p) + d < 0.0f)
        {
            return false;
        }
    }

    return true;
}

class VoxelStreamingManager
{
public:
    FPSCamera* cam;
    uint32_t retire_age = 32;
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
    void update_lru(uint32_t timestamp, uint32_t* nodes, uint32_t num_nodes)
    {
        m_timestamp = timestamp;
        for (uint32_t i = 0; i < num_nodes; i++)
        {
            uint32_t node = nodes[i];
            while (node != UINT32_MAX)
            {
                reset_age(page_of_node(node));
                node = m_nodes[node].parent;
            }
        }

        if (true)
            for (uint32_t i = 0; i < 16; i++)
            {
                uint32_t page_id = m_victim_cursor;
                if (m_page_lru[page_id] == UINT32_MAX) // already cached
                    continue;

                int32_t age = page_age(page_id);
                if (age > retire_age)
                {
                    unlink_page_and_cache(page_id);
                    m_victim_cursor = (m_victim_cursor + 1) % m_page_lru.size();
                }
                m_victim_cursor = (m_victim_cursor + 1) % m_page_lru.size();
            }
    }

    // bool can_discard(uint32_t page_id)
    //{
    //     if (m_page_parent_index[page_id] == UINT32_MAX)
    //         return true;
    //     BuildInfo info = get_build_info(m_page_parent_index[page_id]);
    //     return !is_aabb_in_frustum(cam->get_view_matrix(), cam->get_projection_matrix(),
    //                                info.chunk_origin, info.chunk_size);
    // }

    uint32_t page_of_node(uint32_t node) { return node / m_page_size; }
    int32_t page_age(uint32_t page_id)
    {
        return static_cast<int32_t>(m_timestamp) - static_cast<int32_t>(m_page_lru[page_id]);
    }
    void reset_age(uint32_t page_id) { m_page_lru[page_id] = m_timestamp; }

    uint32_t allocate_page()
    {
        if (!m_free_pages.empty())
        {
            uint32_t page_id = m_free_pages.back();
            m_free_pages.pop_back();
            reset_age(page_id);
            return page_id;
        }
        for (uint32_t i = 0; i < 32; i++)
        {
            uint32_t page_id = m_victim_cursor;
            int32_t age = page_age(page_id);
            if (age > retire_age)
            {
                reset_age(page_id);
                unlink_page(page_id);
                m_victim_cursor = (m_victim_cursor + 1) % m_page_lru.size();

                return page_id;
            }
            m_victim_cursor = (m_victim_cursor + 1) % m_page_lru.size();
        }

        return UINT32_MAX;
    }

    template <typename PFNCreate, typename PFNHomogeneous>
    void process_subdivision_requests(const GenerateInfo<PFNCreate, PFNHomogeneous>& gen_info,
                                      uint32_t* nodes, uint32_t num_nodes)
    {
        uint32_t min_page = m_page_lru.size();
        uint32_t max_page = 0;
        bool should_allocate = true;
        uint32_t page_id = UINT32_MAX;
        uint32_t num_success_requests = 0;
        for (uint32_t i = 0; i < num_nodes; i++)
        {
            uint32_t node_idx = nodes[i];
            if (node_idx == UINT32_MAX)
                continue;

            if (m_nodes[node_idx].child_count() > 0)
                continue;

            BuildInfo build_info = get_build_info(node_idx);

            if (build_info.depth >= 5)
                continue;
            min_page = glm::min(min_page, page_of_node(node_idx));
            max_page = glm::max(max_page, page_of_node(node_idx));
            if (should_allocate)
            {
                page_id = allocate_page();
                min_page = glm::min(min_page, page_id);
                max_page = glm::max(max_page, page_id);
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
        printf("page %u - %u \n", min_page, max_page);
        upload_to_gpu(min_page * m_page_size, max_page * m_page_size + m_page_size);
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

    inline static constexpr bool use_world_nodes = true;
    void upload_to_gpu()
    {
        if (use_world_nodes)
            m_nodes_ssbo.set_sub_data(
                WorldContree::s_nodes.data(),
                glm::min(WorldContree::s_nodes.size(), m_nodes.size()) * sizeof(NodeT), 0);
        else
            m_nodes_ssbo.set_sub_data(m_nodes.data(), m_nodes.size() * sizeof(NodeT), 0);
    }

    void upload_to_gpu(uint32_t begin, uint32_t end)
    {
        if (end < begin)
            return;
        if (use_world_nodes)
        {
            m_nodes_ssbo.set_sub_data(WorldContree::s_nodes.data() + begin,
                                      (end - begin) * sizeof(NodeT), begin * sizeof(NodeT));
        }
        else
        {
            m_nodes_ssbo.set_sub_data(m_nodes.data() + begin, (end - begin) * sizeof(NodeT),
                                      begin * sizeof(NodeT));
        }
    }

    BuildInfo get_build_info(uint32_t node_idx, float chunk_size = 128.f)
    {
        BuildInfo info;

        glm::ivec4 coord = coord_of_node(node_idx);
        info.coord = coord;
        info.chunk_size = chunk_size / glm::pow(4.f, coord.w);
        info.chunk_origin = glm::vec3(coord) * info.chunk_size;
        info.depth = coord.w;

        return info;
    }

    void unlink_page(uint32_t page_id)
    {
        uint32_t parent_node = m_page_parent_index[page_id];
        if (parent_node == UINT32_MAX)
            return;

        uint32_t start = page_id * m_page_size;
        uint32_t end = std::min(start + m_page_size, (uint32_t)m_nodes.size());

        for (uint32_t i = start; i < end; i++)
            if (m_nodes[i].firstchild != NodeT::invalid_offset)
                unlink_page(page_of_node(m_nodes[i].firstchild));

        if (m_nodes[parent_node].firstchild == start)
        {
            m_nodes[parent_node].childmask.clear();
            m_nodes[parent_node].firstchild = NodeT::invalid_offset;
        }
        for (uint32_t i = start; i < end; i++)
        {
            m_nodes[i].parent = UINT32_MAX;
            m_nodes[i].firstchild = NodeT::invalid_offset;
            m_nodes[i].childmask.clear();
        }
        m_page_parent_index[page_id] = UINT32_MAX;
        allocated_pages--;
    }

    void unlink_page_and_cache(uint32_t page_id)
    {
        uint32_t parent_node = m_page_parent_index[page_id];
        if (parent_node == UINT32_MAX)
            return;

        uint32_t start = page_id * m_page_size;
        uint32_t end = std::min(start + m_page_size, (uint32_t)m_nodes.size());

        for (uint32_t i = start; i < end; i++)
            if (m_nodes[i].firstchild != NodeT::invalid_offset)
                unlink_page(page_of_node(m_nodes[i].firstchild));

        if (m_nodes[parent_node].firstchild == start)
        {
            m_nodes[parent_node].childmask.clear();
            m_nodes[parent_node].firstchild = NodeT::invalid_offset;
        }
        for (uint32_t i = start; i < end; i++)
        {
            m_nodes[i].parent = UINT32_MAX;
            m_nodes[i].firstchild = NodeT::invalid_offset;
            m_nodes[i].childmask.clear();
        }
        m_page_lru[page_id] = UINT32_MAX;
        m_page_parent_index[page_id] = UINT32_MAX;
        m_free_pages.push_back(page_id);
        allocated_pages--;
    }

    uint32_t m_timestamp = 0;
    uint32_t m_page_size;

    uint32_t m_victim_cursor = 0;
    PageAllocator m_allocator;

    std::vector<uint32_t> m_page_lru;
    std::vector<uint32_t> m_page_parent_index;
    std::vector<uint32_t> m_free_pages;

    std::vector<NodeT> m_nodes;

    Ssbo m_nodes_ssbo;
};