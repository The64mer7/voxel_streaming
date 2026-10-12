#pragma once

#include "node.hpp"
#include <glm/glm.hpp>

struct BuildInfo
{
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

template <typename NodeT, typename PFNCreate, typename PFNHomogeneous>
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
    if (write_idx == 0)
    {
        printf("subdivided into 0\n");
        return false;
    }

    node.firstchild = first_child_idx;

    return true;
}