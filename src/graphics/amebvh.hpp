#ifndef AME_BVH_H
#define AME_BVH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

    struct ame_mesh
    {
        const float* vertices;
        uint32_t num_vertices;
        uint32_t vertex_stride;

        const uint32_t* indices;
        uint32_t num_indices;
    };

    struct ame_vec3
    {
        union
        {
            float raw[3];
            struct
            {
                float x, y, z;
            };
        };
    };

    ame_vec3 ame_vec3_add(ame_vec3* a, ame_vec3* b);
    ame_vec3 ame_vec3_sub(ame_vec3* a, ame_vec3* b);
    ame_vec3 ame_vec3_mul(ame_vec3* a, ame_vec3* b);
    ame_vec3 ame_vec3_div(ame_vec3* a, ame_vec3* b);

    struct ame_bvh_aabb
    {
        ame_vec3 min, max;
    };

    inline ame_vec3 ame_bvh_aabb_size(const ame_bvh_aabb* aabb)
    {
        ame_vec3 v;
        v.x = aabb->max.x - aabb->min.x;
        v.y = aabb->max.y - aabb->min.y;
        v.z = aabb->max.z - aabb->min.z;
        return v;
    }

    struct ame_bvh_node
    {
        ame_bvh_aabb aabb;
        union
        {
            struct
            {
                int32_t left;
                int32_t right;
            } node;
            struct
            {
                uint32_t begin;
                uint32_t end;
            } leaf;
        };
    };

    inline uint32_t ame_bvh_node_count(const ame_bvh_node* node)
    {
        uint32_t begin = node->leaf.begin & 0x7fffffffu;
        return node->leaf.end - begin;
    }

    struct ame_bvh
    {
        const ame_mesh* mesh;

        const ame_bvh_node* nodes;
        uint32_t num_nodes;
        const uint32_t* sorted_triangles;
    };

    bool ame_create_bvh(ame_bvh* bvh, const ame_mesh* mesh, uint32_t max_triangles_per_leaf);
    void ame_destroy_bvh(const ame_bvh* bvh);

    bool ame_is_leaf(const ame_bvh_node* node);
    void ame_set_leaf(ame_bvh_node* node);

    void ame_reset_leaf(ame_bvh_node* node);

    uint32_t ame_bvh_leaf_begin(const ame_bvh_node* node);

    uint32_t ame_bvh_leaf_end(const ame_bvh_node* node);

#ifdef __cplusplus
}
#endif

#endif // AME_BVH_H

#ifdef AME_BVH_IMPLEMENTATION

#ifdef __cplusplus
extern "C"
{
#endif

    ame_vec3 ame_vec3_add(ame_vec3* a, ame_vec3* b)
    {
        ame_vec3 c;
        c.x = a->x + b->x;
        c.y = a->y + b->y;
        c.z = a->z + b->z;
        return c;
    }

    ame_vec3 ame_vec3_sub(ame_vec3* a, ame_vec3* b)
    {
        ame_vec3 c;
        c.x = a->x - b->x;
        c.y = a->y - b->y;
        c.z = a->z - b->z;
        return c;
    }

    ame_vec3 ame_vec3_mul(ame_vec3* a, ame_vec3* b)
    {
        ame_vec3 c;
        c.x = a->x * b->x;
        c.y = a->y * b->y;
        c.z = a->z * b->z;
        return c;
    }

    ame_vec3 ame_vec3_div(ame_vec3* a, ame_vec3* b)
    {
        ame_vec3 c;
        c.x = a->x / b->x;
        c.y = a->y / b->y;
        c.z = a->z / b->z;
        return c;
    }

    bool ame_is_leaf(const ame_bvh_node* node) { return (node->node.left & 0x80000000u) != 0; }

    void ame_set_leaf(ame_bvh_node* node) { node->node.left |= 0x80000000u; }

    void ame_reset_leaf(ame_bvh_node* node) { node->node.left &= 0x7fffffffu; }

    uint32_t ame_bvh_leaf_begin(const ame_bvh_node* node) { return node->leaf.begin & 0x7fffffff; }

    uint32_t ame_bvh_leaf_end(const ame_bvh_node* node) { return node->leaf.end; }

    struct ame_create_bvh_stack_item
    {
        ame_bvh_node node;
        int32_t parent;
        bool is_left;
        int32_t depth;
    };

    // returns index for indices
    uint32_t ame_triangle_to_index(uint32_t triangle) { return triangle * 3; }

    ame_bvh_aabb ame_mesh_bounds(const ame_mesh* mesh, const uint32_t* triangles, uint32_t begin,
                                 uint32_t end)
    {
        ame_bvh_aabb bounds;

        for (int i = 0; i < 3; i++)
            bounds.min.raw[i] = FLT_MAX;
        for (int i = 0; i < 3; i++)
            bounds.max.raw[i] = -FLT_MAX;

        for (uint32_t t = begin; t < end; t++)
        {
            uint32_t i = ame_triangle_to_index(triangles[t]);
            for (int corner = 0; corner < 3; corner++)
            {
                uint32_t v = mesh->indices[i + corner] * mesh->vertex_stride;
                for (int axis = 0; axis < 3; axis++)
                {
                    bounds.min.raw[axis] = fminf(mesh->vertices[v + axis], bounds.min.raw[axis]);
                    bounds.max.raw[axis] = fmaxf(mesh->vertices[v + axis], bounds.max.raw[axis]);
                }
            }
        }

        return bounds;
    }

#define AME_MAX_STACK_DEPTH 64

    struct ame_vector
    {
        ame_bvh_node* data;
        uint32_t size;
        uint32_t capacity;
    };

    void ame_vector_init(ame_vector* vec, uint32_t initial_capacity)
    {
        vec->size = 0;
        vec->capacity = initial_capacity;
        vec->data = vec->capacity > 0 ? (ame_bvh_node*)malloc(vec->capacity * sizeof(ame_bvh_node))
                                      : NULL;
    }

    void ame_vector_free(ame_vector* vec)
    {
        free(vec->data);
        vec->data = NULL;
        vec->size = 0;
        vec->capacity = 0;
    }

    void ame_vector_push_back(ame_vector* vec, const ame_bvh_node* value)
    {
        if (vec->size >= vec->capacity)
        {
            vec->capacity = vec->capacity == 0 ? 4 : vec->capacity * 2;
            vec->data = (ame_bvh_node*)realloc(vec->data, vec->capacity * sizeof(ame_bvh_node));
        }
        vec->data[vec->size++] = *value;
    }

    void ame_vector_shrink_to_fit(ame_vector* vec)
    {
        if (!vec)
            return;

        if (vec->size == 0)
        {
            free(vec->data);
            vec->data = NULL;
            vec->capacity = 0;
        }
        else if (vec->size < vec->capacity)
        {
            vec->capacity = vec->size;
            vec->data = (ame_bvh_node*)realloc(vec->data, vec->capacity * sizeof(ame_bvh_node));
        }
    }

    int ame_partition_axis(const ame_bvh_aabb* aabb)
    {
        ame_vec3 size = ame_bvh_aabb_size(aabb);
        float max_axis_size = 0;
        int axis = 0;
        for (int i = 0; i < 3; i++)
        {
            float axis_size = size.raw[i];
            if (axis_size > max_axis_size)
            {
                max_axis_size = axis_size;
                axis = i;
            }
        }
        return axis;
    }

    ame_vec3 ame_partition_average_position(const ame_mesh* mesh, const uint32_t* triangles,
                                            uint32_t begin, uint32_t end)
    {
        ame_vec3 average = {0};
        if (end <= begin)
            return average;
        uint32_t count = end - begin;

        for (uint32_t t = begin; t < end; t++)
        {
            uint32_t i = ame_triangle_to_index(triangles[t]);
            uint32_t v0 = mesh->indices[i + 0] * mesh->vertex_stride;
            uint32_t v1 = mesh->indices[i + 1] * mesh->vertex_stride;
            uint32_t v2 = mesh->indices[i + 2] * mesh->vertex_stride;

            for (int axis = 0; axis < 3; axis++)
            {
                average.raw[axis] += (mesh->vertices[v0 + axis] + mesh->vertices[v1 + axis] +
                                      mesh->vertices[v2 + axis]) /
                                     3.0f;
            }
        }

        for (int axis = 0; axis < 3; axis++)
            average.raw[axis] /= (float)count;

        return average;
    }

    void ame_swap(uint32_t* a, uint32_t* b)
    {
        uint32_t t = *a;
        *a = *b;
        *b = t;
    }

    void ame_partition_by_average(const ame_mesh* mesh, uint32_t* triangles,
                                  const ame_bvh_node* parent, ame_bvh_node* left,
                                  ame_bvh_node* right)
    {
        int axis = ame_partition_axis(&parent->aabb);
        uint32_t begin = ame_bvh_leaf_begin(parent);
        uint32_t end = ame_bvh_leaf_end(parent);
        float partition_position = ame_partition_average_position(mesh, triangles, begin, end)
                                       .raw[axis];

        uint32_t left_ptr = begin;
        uint32_t right_ptr = end;

        while (left_ptr < right_ptr)
        {
            uint32_t i = ame_triangle_to_index(triangles[left_ptr]);
            uint32_t v0 = mesh->indices[i + 0] * mesh->vertex_stride;
            uint32_t v1 = mesh->indices[i + 1] * mesh->vertex_stride;
            uint32_t v2 = mesh->indices[i + 2] * mesh->vertex_stride;

            float centroid = 0.f;
            centroid += mesh->vertices[v0 + axis];
            centroid += mesh->vertices[v1 + axis];
            centroid += mesh->vertices[v2 + axis];
            centroid /= 3.0f;

            if (centroid < partition_position)
                left_ptr++;
            else
            {
                right_ptr--;
                ame_swap(&triangles[left_ptr], &triangles[right_ptr]);
            }
        }

        if (left_ptr == begin || left_ptr == end)
            left_ptr = begin + (end - begin) / 2;

        left->leaf.begin = parent->leaf.begin;
        left->leaf.end = left_ptr;
        ame_set_leaf(left);

        right->leaf.begin = left_ptr;
        right->leaf.end = parent->leaf.end;
        ame_set_leaf(right);
    }

    bool ame_create_bvh(ame_bvh* bvh, const ame_mesh* mesh, uint32_t max_triangles_per_leaf)
    {
        if (bvh == NULL || mesh == NULL)
            return false;

        ame_create_bvh_stack_item stack[AME_MAX_STACK_DEPTH];
        int32_t stack_ptr = 0;

        uint32_t num_triangles = mesh->num_indices / 3;

        uint32_t* triangles = (uint32_t*)malloc(num_triangles * sizeof(uint32_t));
        if (triangles == NULL)
            return false;

        for (uint32_t i = 0; i < num_triangles; i++)
            triangles[i] = i;

        ame_vector nodes;
        ame_vector_init(&nodes, 16);

        stack[0].is_left = false;
        stack[0].parent = -1;
        stack[0].depth = 0;

        stack[0].node.leaf.begin = 0;
        stack[0].node.leaf.end = num_triangles;
        ame_set_leaf(&stack[0].node);
        stack_ptr++;

        while (stack_ptr > 0)
        {
            ame_create_bvh_stack_item item = stack[--stack_ptr];
            uint32_t curr_node_idx = nodes.size;

            if (ame_is_leaf(&item.node))
            {
                item.node.aabb = ame_mesh_bounds(mesh, triangles, ame_bvh_leaf_begin(&item.node),
                                                 ame_bvh_leaf_end(&item.node));
            }

            uint32_t num_node_triangles = ame_bvh_node_count(&item.node);
            if (num_node_triangles > max_triangles_per_leaf && num_node_triangles > 1)
            {
                ame_create_bvh_stack_item left;
                left.is_left = true;
                left.parent = curr_node_idx;
                left.depth = item.depth + 1;
                ame_create_bvh_stack_item right;
                right.is_left = false;
                right.parent = curr_node_idx;
                right.depth = item.depth + 1;

                ame_partition_by_average(mesh, triangles, &item.node, &left.node, &right.node);

                if (ame_bvh_node_count(&right.node))
                {
                    stack[stack_ptr++] = right;
                    ame_reset_leaf(&item.node);
                }
                if (ame_bvh_node_count(&left.node))
                {
                    stack[stack_ptr++] = left;
                    ame_reset_leaf(&item.node);
                }
            }

            if (item.parent != -1)
            {
                if (item.is_left)
                    nodes.data[item.parent].node.left = curr_node_idx;
                else
                    nodes.data[item.parent].node.right = curr_node_idx;
            }
            ame_vector_push_back(&nodes, &item.node);
        }
        ame_vector_shrink_to_fit(&nodes);

        bvh->nodes = nodes.data;
        bvh->num_nodes = nodes.size;
        bvh->sorted_triangles = triangles;
        bvh->mesh = mesh;
        return true;
    }

    void ame_destroy_bvh(const ame_bvh* bvh)
    {
        free((void*)bvh->nodes);
        free((void*)bvh->sorted_triangles);
    }

#ifdef __cplusplus
}
#endif

#endif
