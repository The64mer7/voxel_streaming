#pragma once

#include "allocator/node_allocator.hpp"
#include "node.hpp"
#include <glm/glm.hpp>

struct BuildInfo
{
    glm::ivec3 coord;
    glm::vec3 chunk_origin;
    float chunk_size;
    uint32_t depth;
};

template <typename PFNCreate, typename PFNHomogeneous>
struct GenerateInfo
{
    PFNCreate pfn_create;
    PFNHomogeneous pfn_homogeneous;
};

template <typename PFNCreate, typename PFNHomogeneous>
bool subdivide_node(const BuildInfo& info, const GenerateInfo<PFNCreate, PFNHomogeneous>& gen_info,
                    uint32_t node_idx, uint32_t first_child_idx, NodeT* nodes,
                    uint32_t axis_branching_factor)

{
    if (gen_info.pfn_homogeneous(info.chunk_origin, info.chunk_size))
        return false;

    NodeT& node = nodes[node_idx];
    uint32_t write_idx = first_child_idx;
    uint32_t child_bit = 0;

    for (uint32_t z = 0; z < axis_branching_factor; z++)
        for (uint32_t y = 0; y < axis_branching_factor; y++)
            for (uint32_t x = 0; x < axis_branching_factor; x++)
            {
                float child_size = info.chunk_size / axis_branching_factor;
                glm::vec3 child_origin = info.chunk_origin + child_size * glm::vec3(x, y, z);

                NodeT& child_node = nodes[write_idx];

                bool created = gen_info.pfn_create(child_origin, child_size, &child_node.data);
                if (created)
                {
                    child_node.firstchild = NodeT::invalid_offset;
                    child_node.childmask.clear();
                    child_node.parent = node_idx;

                    node.set_bit(child_bit);
                    write_idx++;
                }
                child_bit++;
            }
    if (write_idx == first_child_idx)
    {
        printf("subdivided into 0\n");
        return false;
    }

    node.firstchild = first_child_idx;

    return true;
}

static void free_children_3d_fixed_page(NodeAllocator* allocator, uint32_t node_idx, NodeT* nodes)
{
    uint32_t first_child = nodes[node_idx].firstchild;
    if (first_child == UINT32_MAX)
        return;
    nodes[node_idx].firstchild = UINT32_MAX;

    uint32_t children_page_id = first_child / 64;
    for (int i = 0; i < 64; i++)
    {
        if (nodes[first_child + i].firstchild != UINT32_MAX)
        {
            free_children_3d_fixed_page(allocator, first_child + i, nodes);
        }
    }
    allocator->free(children_page_id);
}

template <typename CreateFunc, typename HomogeneousFunc>
static bool modify_3d_dfs_fixed_page(NodeAllocator* allocator, uint32_t root_idx, NodeT* nodes,
                                     CreateFunc modify_fn, HomogeneousFunc modify_homogeneous_fn,
                                     const glm::vec3& chunk_origin, float chunk_size,
                                     uint32_t max_depth, std::vector<uint32_t>& out_pages,
                                     std::vector<glm::vec4>& aabbs)

{
    struct StackFrame
    {
        uint32_t node_idx;
        glm::vec3 origin;
        float size;
        int depth;
    };
    thread_local std::vector<StackFrame> stack;
    stack.push_back({root_idx, chunk_origin, chunk_size, 0});

    while (!stack.empty())
    {
        StackFrame frame = stack.back();
        stack.pop_back();

        NodeT& node = nodes[frame.node_idx];

        uint32_t first_child = node.firstchild;
        if (modify_homogeneous_fn(frame.origin, frame.size))
        {
            if (first_child != UINT32_MAX)
            {
                free_children_3d_fixed_page(allocator, frame.node_idx, nodes);
            }
            node.clear_bits();
            node.firstchild = UINT32_MAX;
            continue;
        }
        else
        {
            if (frame.depth >= max_depth)
                continue;
            if (first_child == UINT32_MAX)
            {
                aabbs.push_back(glm::vec4(frame.origin, frame.size));
                first_child = allocator->allocate();
                node.firstchild = first_child;
                node.childmask.fill_mask(0, 63);
                for (int i = 0; i < 64; i++)
                {
                    nodes[first_child + i].data = node.data;
                    nodes[first_child + i].parent = frame.node_idx;
                    nodes[first_child + i].firstchild = UINT32_MAX;
                    nodes[first_child + i].clear_bits();
                }
            }
            for (int i = 0; i < 64; i++)
            {
                StackFrame child;
                child.depth = frame.depth + 1;
                child.size = frame.size / 4;
                child.origin = frame.origin + child.size * glm::vec3(i % 4, (i / 4) % 4, i / 16);
                child.node_idx = first_child + i;

                uint32_t data = 0;
                bool modify_child = modify_fn(child.origin, child.size, &data);
                if (modify_child)
                {
                    if (!node.get_bit(i))
                    {
                        nodes[child.node_idx].parent = frame.node_idx;
                    }
                    nodes[child.node_idx].data = data;
                    stack.push_back(child);

                    node.childmask.set(i);
                }
            }
        }
    }
    return true;
}

template <typename CreateFunc, typename HomogeneousFunc>
static bool create_3d_dfs_fixed_page(NodeAllocator* allocator, uint32_t root_idx, NodeT* nodes,
                                     CreateFunc create_fn, HomogeneousFunc homogeneous_fn,
                                     const glm::vec3& chunk_origin, float chunk_size,
                                     uint32_t max_depth, std::vector<uint32_t>& out_pages)
{
    struct StackFrame
    {
        uint32_t node_idx;
        glm::vec3 origin;
        float size;
        int depth;
    };

    NodeT root_node;
    root_node.clear_bits();
    root_node.parent = UINT32_MAX;
    root_node.firstchild = UINT32_MAX;

    if (!create_fn(chunk_origin, chunk_size, &root_node.data))
    {
        return false;
    }

    auto allocate_nodes = [&]()
    {
        uint32_t absolute_index = allocator->allocate();
        if (absolute_index == UINT32_MAX)
        {
            printf("out of mem");
            exit(1);
        }
        out_pages.push_back(absolute_index / 64);
        return absolute_index;
    };

    nodes[root_idx] = root_node;
    allocate_nodes();

    auto free_nodes = [&](uint32_t page_offset) { allocator->free(page_offset); };

    thread_local std::vector<StackFrame> stack;
    stack.clear();
    stack.push_back({root_idx, chunk_origin, chunk_size, 0});

    while (!stack.empty())
    {
        StackFrame frame = stack.back();
        stack.pop_back();
        NodeT node = nodes[frame.node_idx];

        if (homogeneous_fn(frame.origin, frame.size) || frame.depth >= max_depth)
        {
            continue;
        }
        uint32_t first_child_idx = allocate_nodes();

        uint32_t num_nodes = 0;
        for (int z = 0; z < 4; z++)
            for (int y = 0; y < 4; y++)
                for (int x = 0; x < 4; x++)
                {
                    float child_size = frame.size * 0.25f;
                    glm::vec3 child_origin = frame.origin + child_size * glm::vec3(x, y, z);

                    NodeT child_node;

                    bool created = create_fn(child_origin, child_size, &child_node.data);
                    if (created)
                    {
                        child_node.clear_bits();
                        child_node.firstchild = UINT32_MAX;
                        child_node.parent = frame.node_idx;
                        uint32_t flat_idx = x + 4 * y + 16 * z;
                        nodes[frame.node_idx].set_bit(flat_idx);

                        nodes[first_child_idx + flat_idx] = child_node;
                        stack.emplace_back(first_child_idx + flat_idx, child_origin, child_size,
                                           frame.depth + 1);
                        num_nodes++;
                    }
                }
        if (num_nodes != 0)
        {
            nodes[frame.node_idx].firstchild = first_child_idx;
        }
        else
        {
            free_nodes(first_child_idx);
        }
    }
    printf("pages %u\n", out_pages.size());
    return true;
}