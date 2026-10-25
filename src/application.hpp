#pragma once
#include <glad/gl.h>

#include <GLFW/glfw3.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/vec_swizzle.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <bitset>
#include <format>
#include <iostream>
#include <set>
#include <thread>
#include <unordered_set>

#include "core.hpp"

#include "terrain_generator/contree_gen.hpp"
#include "voxels/probe.hpp"
#include "voxels/streaming_manager.hpp"

static glm::vec2 imgui_to_glm_vec2(const ImVec2& v) { return glm::vec2{v.x, v.y}; }

inline void debug_print_tree(NodeT* nodes, uint32_t idx = 0, uint32_t depth = 0)
{
    auto& node = nodes[idx];
    uint64_t combined_mask = static_cast<uint64_t>(node.childmask.mask[0]) |
                             (static_cast<uint64_t>(node.childmask.mask[1]) << 32);

    std::bitset<64> mask_bits(combined_mask);
    std::string separator = "        ";
    separator[depth] = '\0';
    ImGui::Text("%sNode [%d] | data: %u | parent: %u | firstchild: %u | mask: %s",
                separator.c_str(), idx, node.data, node.parent, node.firstchild,
                mask_bits.to_string().c_str());

    if (node.firstchild != UINT32_MAX)
        for (int i = 0; i < 64; i++)
        {
            if (node.get_bit(i))
                debug_print_tree(nodes, node.firstchild + i, depth + 1);
        }
}

class Application
{
public:
    Application(uint32_t width, uint32_t height);

    ~Application() {}

    // GLFW, GLAD, ImGui init
    void init();

    // Start the app
    void run();

    // Your application logic here
    void update(float dt);

    // GLFW, GLAD, ImGui cleanup
    void cleanup();

    void start()
    {
        constexpr uint32_t num_nodes = 1024 * 1024 * 128;
        streaming_manager.init(num_nodes, 64);
        streaming_manager.cam = &camera;
        // TODO:
        if (!sp_raymarch.load_compute(
                "C:/Users/jaros/Desktop/voxel_streaming/shaders/raymarch.comp"))
            exit(1);
        if (!sp_irradiance_raymarch.load_compute(
                "C:/Users/jaros/Desktop/voxel_streaming/shaders/raymarch_irradiance.comp"))
            exit(1);
        if (!sp_aabb.load_graphics("C:/Users/jaros/Desktop/voxel_streaming/shaders/aabb.vert",
                                   "C:/Users/jaros/Desktop/voxel_streaming/shaders/aabb.frag"))
            exit(1);
        if (!sp_probes_debug.load_graphics(
                "C:/Users/jaros/Desktop/voxel_streaming/shaders/probe_debug.vert",
                "C:/Users/jaros/Desktop/voxel_streaming/shaders/probe_debug.frag"))
            exit(1);
        if (!sp_probes_debug_ray.load_graphics(
                "C:/Users/jaros/Desktop/voxel_streaming/shaders/probe_debug_ray.vert",
                "C:/Users/jaros/Desktop/voxel_streaming/shaders/probe_debug_ray.frag"))
            exit(1);
        if (!sp_request_bitmask_gen.load_compute(
                "C:/Users/jaros/Desktop/voxel_streaming/shaders/request_bitmask_gen.comp"))
            exit(1);
        if (!sp_compact_requests.load_compute(
                "C:/Users/jaros/Desktop/voxel_streaming/shaders/compact_requests_fix.comp"))
            exit(1);
        if (!sp_composite.load_compute(
                "C:/Users/jaros/Desktop/voxel_streaming/shaders/composite.comp"))
            exit(1);
        if (!sp_ssao.load_compute("C:/Users/jaros/Desktop/voxel_streaming/shaders/ssao.comp"))
            exit(1);

        uint32_t w = 800;
        uint32_t h = 600;
        albedo_texture.create(GL_RGBA8);
        albedo_texture.allocate(w, h);

        final_texture.create(GL_RGBA8);
        final_texture.allocate(w, h);

        debug_texture.create(GL_RGBA16F);
        debug_texture.allocate(w, h);

        depth_texture.create(GL_R16F);
        depth_texture.allocate(w, h);

        ao_texture.create(GL_R16F);
        ao_texture.allocate(w / 2, h / 2);

        normals_texture.create(GL_RGBA16_SNORM);
        normals_texture.allocate(w, h);

        for (int i = 0; i < 3; i++)
        {
            request_imgs[i].create(GL_RGBA32UI);
            request_imgs[i].allocate(w, h);
        }

        request_node_bitmask.callocate<uint32_t>(nullptr, w * h + 1);
        request_node_bitmask.clear(0);

        for (int i = 0; i < 2; i++)
        {
            compact_nodes[i].callocate<uint32_t>(nullptr, 12 * w * h + 1,
                                                 GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT);
            compact_nodes[i].clear(0);
        }

        DispatchIndirectCommand cmd = {0, 0, 0};
        compact_requests_indirect.callocate<DispatchIndirectCommand>(&cmd, 1);

        probes.callocate<Probe>(nullptr, glm::compMul(probe_counts), GL_DYNAMIC_STORAGE_BIT);
        probes.clear(0);

        probes_colors.create(GL_RGBA8, GL_TEXTURE_2D_ARRAY);
        probes_colors.allocate_3d(probe_resolution * probe_counts.x,
                                  probe_resolution * probe_counts.y, probe_counts.z, GL_LINEAR);

        glm::vec4 initial_color(0.f);
        glClearTexImage(probes_colors.get_handle(), 0, GL_RGBA, GL_FLOAT, &initial_color);

        probes_depths.create(GL_RG16F, GL_TEXTURE_2D_ARRAY);
        probes_depths.allocate_3d(probe_resolution * probe_counts.x,
                                  probe_resolution * probe_counts.y, probe_counts.z, GL_LINEAR);
        float initial_depth = 0.f;
        glClearTexImage(probes_depths.get_handle(), 0, GL_RED, GL_FLOAT, &initial_depth);
        for (int i = 0; i < 2; i++)
        {
            request_buffers[i].allocate(nullptr, (1024 * 1024 + 1) * sizeof(uint32_t),
                                        GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT);
            uint32_t zero = 0;
            request_buffers[i].set_sub_data(&zero, sizeof(uint32_t), 0);
            fences[i] = nullptr;

            request_bitmasks[i].allocate(nullptr, sizeof(uint32_t) * (num_nodes + 31) / 32,
                                         GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT);
            request_bitmasks[i].clear(0);
        }

        if (!fbo)
            glCreateFramebuffers(1, &fbo);
        if (!rbo)
            glCreateRenderbuffers(1, &rbo);

        glNamedRenderbufferStorage(rbo, GL_DEPTH24_STENCIL8, width, height);

        glNamedFramebufferTexture(fbo, GL_COLOR_ATTACHMENT0, final_texture.get_handle(), 0);
        glNamedFramebufferRenderbuffer(fbo, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

        if (glCheckNamedFramebufferStatus(fbo, GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            printf("error creating framebuffer\n");
            exit(1);
        }

        glCreateVertexArrays(1, &empty_vao);

        setup_camera();
        WorldContree::s_nodes.resize(1024 * 1024 * 128);
        WorldContree::s_allocator.create(WorldContree::s_nodes.size() / 64, 64);
        // MeshContree::create("meshes/satellite/satellite.obj");

        std::vector<uint32_t> pages;

        if (false)
        {
            MeshContree::create("meshes/sponza/sponza.obj");

            auto root_aabb = MeshContree::get_bvh().nodes[0].aabb;
            ame_vec3 bvh_size = ame_vec3_sub(&root_aabb.max, &root_aabb.min);
            glm::vec3 tree_origin = glm::make_vec3(&root_aabb.min.x);
            float tree_scale = glm::max(bvh_size.x, glm::max(bvh_size.y, bvh_size.z));
            printf("voxelize...\n");
            uint32_t root_idx = create_3d_dfs_fixed_page(
                &WorldContree::s_allocator, WorldContree::s_nodes.data(), MeshContree::create3d,
                MeshContree::homogeneous3d, tree_origin, tree_scale, 4, pages);
        }
        else
        {
            printf("voxelize...\n");
            create_3d_dfs_fixed_page(&WorldContree::s_allocator, WorldContree::s_nodes.data(),
                                     TerrainContree::create3d, TerrainContree::homogeneous3d,
                                     glm::vec3(0), 128.f, 4, pages);
        }
        printf("done\n");

        // WorldContree::create_3d_dfs(glm::vec3(0), chunk_size, 4);
        streaming_manager.upload_to_gpu();
    }
    glm::uvec4 data[3] = {glm::uvec4(UINT32_MAX), glm::uvec4(UINT32_MAX), glm::uvec4(UINT32_MAX)};

    static constexpr auto gen_info = GenerateInfo{TerrainContree::create3d,
                                                  TerrainContree::homogeneous3d};

    void old_stream(uint32_t slot)
    {
        if (!stream)
            return;
        stream = false;
        if (frame_index == 0)
            return;
        uint32_t* requests = (uint32_t*)request_buffers[slot].map(GL_MAP_READ_BIT);

        uint32_t num_requests = requests[0];
        // printf("requests from gpu %u\n", num_requests);

        static std::vector<uint32_t> request_nodes;
        if (num_requests)
        {
            request_nodes.resize(num_requests);
            double t0 = glfwGetTime();
            // printf("memcpybegin: ");
            memcpy(request_nodes.data(), requests + 1, sizeof(uint32_t) * num_requests);
            double t1 = glfwGetTime();
            // printf("time: %f\n", t1 - t0);
            static std::set<uint32_t> unique_requests;
            unique_requests.clear();
            compact_requests.clear();
            for (uint32_t i = 0; i < num_requests; i++)
            {
                if (!unique_requests.contains(request_nodes[i]))
                {
                    unique_requests.insert(request_nodes[i]);
                }
            }
            while (!unique_requests.empty())
            {
                uint32_t request_val = *std::prev(unique_requests.end());
                unique_requests.erase(std::prev(unique_requests.end()));
                compact_requests.push_back(request_val & 0x1fffffffu);
            }
            printf("requests after compaction %u (from %u)\n", compact_requests.size(),
                   request_nodes.size());

            streaming_manager.update_lru(frame_index, compact_requests.data(),
                                         compact_requests.size());
            streaming_manager.process_subdivision_requests(gen_info, compact_requests.data(),
                                                           compact_requests.size());
        }
        request_buffers[slot].unmap();
        uint32_t zero = 0;
        request_buffers[slot].set_sub_data(&zero, sizeof(uint32_t), 0);
    }
    std::vector<uint32_t> streamed_nodes;
    std::vector<uint32_t> streamed_compact_nodes;
    std::vector<BuildInfo> streamed_compact_aabbs;
    void stream_nodes()
    {
        if (frame_index == 0)
            return;
        uint32_t slot = frame_index % 2;
        if (stream)
        {
            double t_total_start = glfwGetTime();

            double t0 = glfwGetTime();
            uint32_t* compact = (uint32_t*)compact_nodes[slot].map(GL_MAP_READ_BIT);
            streamed_nodes.resize(compact[0]);
            if (streamed_nodes.size())
            {
                memcpy(streamed_nodes.data(), compact + 1,
                       sizeof(uint32_t) * streamed_nodes.size());
            }
            compact_nodes[slot].unmap();
            double t_map_end = glfwGetTime();

            static std::unordered_set<uint32_t> nodes_set;
            nodes_set.clear();
            streamed_compact_nodes.clear();
            // streamed_compact_aabbs.clear();

            for (int i = 0; i < streamed_nodes.size(); i++)
            {
                uint32_t node = streamed_nodes[i];
                if (!nodes_set.contains(node))
                {
                    nodes_set.insert(node);
                    if (node != UINT32_MAX)
                    {
                        streamed_compact_nodes.push_back(node);
                        // streamed_compact_aabbs.push_back(streaming_manager.get_build_info(node));
                    }
                }
            }
            double t_dedup_end = glfwGetTime();

            if (!streamed_nodes.empty())
            {
                printf("%u -> %u (%f)\n", streamed_nodes.size(), streamed_compact_nodes.size(),
                       float(streamed_compact_nodes.size()) / streamed_nodes.size());
            }

#if 0
        for (int i = 0; i < 3; i++)
            glGetTextureSubImage(request_imgs[i].get_handle(), 0, request_imgs[i].width() / 2,
                                 request_imgs[i].height() / 2, 0, 1, 1, 1, GL_RGBA_INTEGER,
                                 GL_UNSIGNED_INT, sizeof(glm::uvec4), glm::value_ptr(data[i]));
#endif

            // 3. Measure Manager Updates (LRU & Subdivisions)
            double t_mgr_start = glfwGetTime();
            streaming_manager.update_lru(frame_index, streamed_compact_nodes.data(),
                                         streamed_compact_nodes.size());
            streaming_manager.process_subdivision_requests(gen_info, streamed_compact_nodes.data(),
                                                           streamed_compact_nodes.size());
            double t_mgr_end = glfwGetTime();

            printf("time:\n");
            printf("  - buffer readback: %fms\n", 1000 * (t_map_end - t0));
            printf("  - deduplication: %fms\n", 1000 * (t_dedup_end - t_map_end));
            printf("  - manager :  %fms\n", 1000 * (t_mgr_end - t_mgr_start));
            printf("  - total stream time:   %fms\n", 1000 * (t_mgr_end - t_total_start));
        }
        stream = false;
    }

    std::vector<std::string> profiling_names;
    std::vector<uint64_t> profiling_times;
    std::vector<uint32_t> profiling_depths;

    void tick(float dt)
    {
        handle_input(dt);

        uint32_t slot = frame_index % 2;

        if (fences[slot])
        {
            glClientWaitSync(fences[slot], GL_SYNC_FLUSH_COMMANDS_BIT, 1000000000);
            glDeleteSync(fences[slot]);
            fences[slot] = nullptr;

            // old_stream(slot);
            stream_nodes();
        }

        sp_irradiance_raymarch.bind();

        sp_irradiance_raymarch.set_uint("u_frame_index", frame_index);
        sp_irradiance_raymarch.set_float("u_chunk_size", chunk_size);

        sp_irradiance_raymarch.set_uvec3("u_probe_counts", probe_counts);
        sp_irradiance_raymarch.set_int("u_probe_resolution", probe_resolution);
        sp_irradiance_raymarch.set_float("u_probe_spacing", probe_spacing);
        sp_irradiance_raymarch.set_float("u_probe_hysteresis", probe_hysteresis);
        sp_irradiance_raymarch.set_sampler("u_probe_depths_tex", probes_depths.get_handle(), 0,
                                           GL_TEXTURE_2D_ARRAY);
        sp_irradiance_raymarch.set_sampler("u_probe_colors_tex", probes_colors.get_handle(), 1,
                                           GL_TEXTURE_2D_ARRAY);

        sp_irradiance_raymarch.set_image("u_probe_depths", probes_depths.get_handle(), 0,
                                         GL_READ_WRITE, probes_depths.get_internal_format());
        sp_irradiance_raymarch.set_image("u_probe_colors", probes_colors.get_handle(), 1,
                                         GL_READ_WRITE, probes_colors.get_internal_format());

        streaming_manager.bind_buffer(0);
        probes.bind_base(1);

        profiling_timer.begin_gpu("global_illumination_ray");
        sp_irradiance_raymarch.dispatch_compute(
            glm::uvec3(2, 2, 2), GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
        profiling_timer.end_gpu("global_illumination_ray");

        sp_raymarch.bind();

        streaming_manager.bind_buffer(1);
        request_buffers[slot].bind_base(2);
        probes.bind_base(3);
        request_node_bitmask.bind_base(4);

        sp_raymarch.set_image("u_albedo", albedo_texture.get_handle(), 0, GL_READ_WRITE,
                              albedo_texture.get_internal_format());

        sp_raymarch.set_image("u_depth", depth_texture.get_handle(), 1, GL_READ_WRITE,
                              depth_texture.get_internal_format());

        sp_raymarch.set_image("u_normal", normals_texture.get_handle(), 2, GL_READ_WRITE,
                              normals_texture.get_internal_format());

        sp_raymarch.set_image("u_debug_texture", debug_texture.get_handle(), 3, GL_READ_WRITE,
                              debug_texture.get_internal_format());

        for (int i = 0; i < 3; i++)
        {
            sp_raymarch.set_image((std::string("u_img_requests") + std::to_string(i)).c_str(),
                                  request_imgs[i].get_handle(), 4 + i, GL_READ_WRITE,
                                  request_imgs[i].get_internal_format());
        }

        sp_raymarch.set_mat4("u_inv_view", glm::inverse(camera.get_view_matrix()));
        sp_raymarch.set_mat4("u_inv_proj", glm::inverse(camera.get_projection_matrix()));
        sp_raymarch.set_vec3("u_cam_pos", camera.get_position());
        sp_raymarch.set_uint("u_frame_index", frame_index);
        sp_raymarch.set_float("u_chunk_size", chunk_size);

        glm::uvec3 raymarch_groups;
        raymarch_groups.x = (albedo_texture.width() + 15) / 16;
        raymarch_groups.y = (albedo_texture.height() + 15) / 16;
        raymarch_groups.z = 1;

        profiling_timer.begin_gpu("primary_ray");
        sp_raymarch.dispatch_compute(
            raymarch_groups, GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT |
                                 GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
        profiling_timer.end_gpu("primary_ray");

        {

            glm::uvec4 groups;
            groups.x = (request_imgs[0].width() + 15) / 16;
            groups.y = (request_imgs[0].height() + 15) / 16;
            groups.z = 1;

            uint32_t zero = 0;
            request_node_bitmask.set_sub_data(&zero, sizeof(zero));
            sp_request_bitmask_gen.bind();
            sp_request_bitmask_gen.set_sampler("u_tex_requests0", request_imgs[0].get_handle(), 0,
                                               GL_TEXTURE_2D);
            sp_request_bitmask_gen.set_sampler("u_tex_requests1", request_imgs[1].get_handle(), 1,
                                               GL_TEXTURE_2D);
            sp_request_bitmask_gen.set_sampler("u_tex_requests2", request_imgs[2].get_handle(), 2,
                                               GL_TEXTURE_2D);

            request_node_bitmask.bind_base(0);
            compact_requests_indirect.bind_base(1);

            profiling_timer.begin_gpu("requests_bitmask");
            sp_request_bitmask_gen.dispatch_compute(
                groups, GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT |
                            GL_TEXTURE_FETCH_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
            profiling_timer.end_gpu("requests_bitmask");
        }

        {
            uint32_t zero = 0;
            compact_nodes[slot].set_sub_data(&zero, sizeof(zero));

            sp_compact_requests.bind();
            sp_compact_requests.set_sampler("u_tex_requests0", request_imgs[0].get_handle(), 0,
                                            GL_TEXTURE_2D);
            sp_compact_requests.set_sampler("u_tex_requests1", request_imgs[1].get_handle(), 1,
                                            GL_TEXTURE_2D);
            sp_compact_requests.set_sampler("u_tex_requests2", request_imgs[2].get_handle(), 2,
                                            GL_TEXTURE_2D);

            request_node_bitmask.bind_base(0);
            compact_nodes[slot].bind_base(1);
            profiling_timer.begin_gpu("requests_compact");
            sp_compact_requests.dispatch_compute_indirect(compact_requests_indirect,
                                                          GL_SHADER_STORAGE_BARRIER_BIT |
                                                              GL_BUFFER_UPDATE_BARRIER_BIT |
                                                              GL_TEXTURE_FETCH_BARRIER_BIT);
            profiling_timer.end_gpu("requests_compact");
        }

        {
            sp_ssao.bind();

            sp_ssao.set_sampler("u_depth", depth_texture.get_handle(), 0, GL_TEXTURE_2D);
            sp_ssao.set_sampler("u_normal", normals_texture.get_handle(), 1, GL_TEXTURE_2D);
            sp_ssao.set_mat4("u_inv_proj", glm::inverse(camera.get_projection_matrix()));
            sp_ssao.set_mat4("u_inv_view", glm::inverse(camera.get_view_matrix()));
            sp_ssao.set_vec3("u_cam_pos", camera.get_position());

            sp_ssao.set_float("u_radius", ssao_radius);
            sp_ssao.set_float("u_bias", ssao_bias);

            sp_ssao.set_image("u_ao", ao_texture.get_handle(), 0, GL_WRITE_ONLY, GL_R16F);

            sp_ssao.dispatch_compute_grid(
                {ao_texture.width(), ao_texture.height(), 1}, glm::uvec3(16, 16, 1),
                GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
        }

        {
            sp_composite.bind();
            sp_composite.set_uint("u_display_texture", display_texture);
            sp_composite.set_mat4("u_inv_view", glm::inverse(camera.get_view_matrix()));
            sp_composite.set_mat4("u_inv_proj", glm::inverse(camera.get_projection_matrix()));
            sp_composite.set_vec3("u_cam_pos", camera.get_position());
            sp_composite.set_uint("u_frame_index", frame_index);
            sp_composite.set_float("u_brightness", display_brightness);
            sp_composite.set_float("u_probe_spacing", probe_spacing);
            sp_composite.set_uvec3("u_probe_counts", probe_counts);
            sp_composite.set_int("u_probe_resolution", probe_resolution);

            probes.bind_base(0);

            sp_composite.set_image("u_albedo", albedo_texture.get_handle(), 0, GL_READ_ONLY,
                                   albedo_texture.get_internal_format());

            sp_composite.set_image("u_depth", depth_texture.get_handle(), 1, GL_READ_ONLY,
                                   depth_texture.get_internal_format());

            sp_composite.set_image("u_normal", normals_texture.get_handle(), 2, GL_READ_ONLY,
                                   normals_texture.get_internal_format());

            sp_composite.set_image("u_final", final_texture.get_handle(), 3, GL_READ_WRITE,
                                   final_texture.get_internal_format());

            sp_composite.set_image("u_probe_depths", probes_depths.get_handle(), 4, GL_READ_ONLY,
                                   probes_depths.get_internal_format());
            sp_composite.set_image("u_probe_colors", probes_colors.get_handle(), 5, GL_READ_ONLY,
                                   probes_colors.get_internal_format());

            sp_composite.set_sampler("u_debug_texture", debug_texture.get_handle(), 0,
                                     GL_TEXTURE_2D);

            sp_composite.set_sampler("u_probe_depths_tex", probes_depths.get_handle(), 1,
                                     GL_TEXTURE_2D_ARRAY);
            sp_composite.set_sampler("u_probe_colors_tex", probes_colors.get_handle(), 2,
                                     GL_TEXTURE_2D_ARRAY);
            sp_composite.set_sampler("u_ao", ao_texture.get_handle(), 3, GL_TEXTURE_2D);

            sp_composite.dispatch_compute_grid(
                glm::uvec3(final_texture.width(), final_texture.height(), 1), glm::uvec3(16, 16, 1),
                GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
        }

        fences[slot] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);

        frame_index++;
    }
    std::vector<uint32_t> compact_requests;

    bool stream = false;
    bool display_aabbs = false;
    void render(float dt)
    {
        glViewport(0, 0, final_texture.width(), final_texture.height());
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        if (false)
            for (int i = 0; i < 12; i++)
            {
                int vector_idx = i / 4;
                int elem_idx = i % 4;
                if (data[vector_idx][elem_idx] != UINT32_MAX)
                {
                    BuildInfo info = streaming_manager.get_build_info(data[vector_idx][elem_idx]);
                    // draw_aabb(info.chunk_origin, info.chunk_size);
                }
            }

        for (int i = 0; i < aabbs.size(); i++)
        {
            draw_aabb(glm::xyz(aabbs[i]), aabbs[i].w);
        }
        if (display_aabbs)
        {

            for (int i = 0; i < streamed_compact_aabbs.size(); i++)
            {
                draw_aabb(streamed_compact_aabbs[i].chunk_origin,
                          streamed_compact_aabbs[i].chunk_size);
            }
            const auto& bvh = MeshContree::get_bvh();
            for (uint32_t i = 0; i < bvh.num_nodes; i++)
            {
                auto& min = bvh.nodes[i].aabb.min;
                auto& max = bvh.nodes[i].aabb.max;
                if (bvh.nodes[i].leaf.begin & 0x80000000u)
                    draw_aabb(glm::vec3(min.x, min.y, min.z),
                              glm::vec3(max.x - min.x, max.y - min.y, max.z - min.z));
            }
        }
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        // draw_probes();

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        if (ImGui::IsKeyDown(ImGuiKey_P))
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (ImGui::Begin("graphics settings"))
        {
            ImGui::SliderFloat("probe_spacing", &probe_spacing, 1.f, 16.f);
            ImGui::SliderFloat("probe_hysteresis", &probe_hysteresis, 0.f, 1.f);

            ImGui::SliderFloat("ssao_radius", &ssao_radius, 0, 1);
            ImGui::SliderFloat("ssao_bias", &ssao_bias, 0, 1);
        }
        ImGui::End();

        if (ImGui::Begin("g-buffer"))
        {
            ImGui::SliderFloat("brightness", &display_brightness, 0, 1);
            for (int i = 0; i < sizeof(g_buffer_names) / sizeof(const char*); i++)
            {
                if (ImGui::Button(g_buffer_names[i]))
                {
                    display_texture = i;
                }
            }
        }
        ImGui::End();

        bool is_viewport_hovered = false;
        glm::vec2 region_screen = glm::vec2(0);
        if (ImGui::Begin("display"))
        {
            region_screen = imgui_to_glm_vec2(ImGui::GetCursorScreenPos());
            ImGui::Image(final_texture.get_handle(),
                         ImVec2(viewport_scale * final_texture.width(),
                                viewport_scale * final_texture.height()),
                         ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
            is_viewport_hovered = ImGui::IsItemHovered();
        }
        ImGui::End();
        if (ImGui::Begin("tree"))
        {
            // debug_print_tree(WorldContree::s_nodes.data());
            static int start_idx = 0;
            ImGui::SliderInt("start_idx", &start_idx, 0, WorldContree::s_nodes.size() - 256);
            ImGui::Separator();
            for (int i = 0; i < 256; i++)
            {
                auto& node = WorldContree::s_nodes[i + start_idx];

                uint64_t combined_mask = static_cast<uint64_t>(node.childmask.mask[0]) |
                                         (static_cast<uint64_t>(node.childmask.mask[1]) << 32);

                std::bitset<64> mask_bits(combined_mask);

                ImGui::Text("Node [%d] | data: %u | parent: %u | firstchild: %u | mask: %s",
                            i + start_idx, node.data, node.parent, node.firstchild,
                            mask_bits.to_string().c_str());
            }
        }
        ImGui::End();

        if (ImGui::Begin("frame_time"))
        {
            for (uint32_t i = 0; i < profiling_names.size(); i++)
            {
                ImGui::SetCursorPosX(profiling_depths[i] * ImGui::CalcTextSize(" ").x * 4.f);
                ImGui::Text("%s: %.3fms", profiling_names[i].c_str(),
                            double(profiling_times[i]) / 1000000.0);
            }
        }
        ImGui::End();

        if (ImGui::Begin("debug"))
        {
            if (ImGui::Button("reload shaders"))
            {
                sp_raymarch.reload();
                sp_aabb.reload();
                sp_request_bitmask_gen.reload();
                sp_compact_requests.reload();
                sp_irradiance_raymarch.reload();
                sp_composite.reload();
                sp_probes_debug.reload();
                sp_probes_debug_ray.reload();
                sp_ssao.reload();
            }
            ImGui::SliderFloat2("minmax debug draw aabb", glm::value_ptr(draw_aabb_minmax_size),
                                0.f, 32.f);

            ImGui::SliderFloat("radius", &BallContree::ball.w, 1, 32);

            glm::vec2 mouse_screen = imgui_to_glm_vec2(ImGui::GetMousePos());
            glm::vec2 mouse_pos = (mouse_screen - region_screen) / viewport_scale;
            ImGui::Text("mouse_pos %.2f %.2f", mouse_pos.x, mouse_pos.y);

            static float d = 0.f;
            static uint32_t brush_color = 0xfff;
            if (is_viewport_hovered && ImGui::IsMouseDown(ImGuiMouseButton_Left))
            {

                if (glm::all(glm::greaterThanEqual(mouse_pos, glm::vec2(0))) &&
                    glm::all(
                        glm::lessThan(mouse_pos, {depth_texture.width(), depth_texture.height()})))
                {
                    glGetTextureSubImage(depth_texture.get_handle(), 0, mouse_pos.x,
                                         depth_texture.height() - mouse_pos.y, 0, 1, 1, 1, GL_RED,
                                         GL_FLOAT, sizeof(d), &d);
                    ImGui::Text("depth: %f", d);
                }

                uint32_t r = rand() % 15 + 1;
                uint32_t g = rand() % 15 + 1;
                uint32_t b = rand() % 15 + 1;
                brush_color = r | (g << 4) | (b << 8);
                auto create_ray = [&](glm::vec3& ro, glm::vec3& rd)
                {
                    glm::vec2 img_size = {depth_texture.width(), depth_texture.height()};

                    glm::vec2 ndc = (mouse_pos + 0.5f) / img_size * 2.0f - 1.0f;
                    ndc.y *= -1;
                    float aspect = float(img_size.x) / float(img_size.y);

                    glm::vec4 clip = glm::inverse(camera.get_projection_matrix()) *
                                     glm::vec4(ndc, -1.0f, 1.0f);
                    clip /= clip.w;
                    glm::vec4 world_pos = glm::inverse(camera.get_view_matrix()) * clip;

                    ro = camera.get_position();
                    rd = glm::normalize(glm::xyz(world_pos) - ro);
                };

                glm::vec3 ro, rd;
                create_ray(ro, rd);
                glm::vec3 wp = ro + rd * d;

                std::vector<uint32_t> pages;
                BallContree::ball.x = wp.x;
                BallContree::ball.y = wp.y;
                BallContree::ball.z = wp.z;

                aabbs.clear();

                BallContree::color = 0x00fu;
                BallContree::clear = input.is_key_held(GLFW_KEY_E);

                uint32_t before_pages = WorldContree::s_allocator.free_pages_count();

                Contree3DModifier modifier;
                modifier.out_minmax = glm::uvec2(UINT32_MAX, 0);
                modifier.allocator = &WorldContree::s_allocator;
                modifier.nodes = WorldContree::s_nodes.data();
                modifier.num_nodes = WorldContree::s_nodes.size();
                modifier.modify_fn = BallContree::create3d;
                modifier.modify_homogeneous_fn = BallContree::homogeneous3d;
                modifier.max_depth = 4;
                modifier.aabbs = &aabbs;
                modifier.erase = BallContree::clear;
                modifier.modify_3d_dfs_iter3(glm::vec3(0), 128, 0, 0);

                uint32_t after_pages = WorldContree::s_allocator.free_pages_count();

                if (after_pages > before_pages)
                    printf("edit freed: %u pages\n", after_pages - before_pages);
                else
                    printf("edit alloc: %u pages\n", before_pages - after_pages);

                profiling_timer.begin_gpu("upload_edit");
                streaming_manager.upload_to_gpu(modifier.out_minmax.x, modifier.out_minmax.y);
                profiling_timer.end_gpu("upload_edit");
            }

            ImGui::Checkbox("display_aabbs", &display_aabbs);
            // uint32_t* data_requested = &data[0][0];
            // for (int i = 0; i < 12; i++)
            //{
            //     if (i < 11 && data_requested[i] == data_requested[i + 1])
            //         ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 0, 0, 1));
            //     ImGui::Text("[%u]", data_requested[i]);
            //     if (i < 11 && data_requested[i] == data_requested[i + 1])
            //         ImGui::PopStyleColor(1);
            // }

            ImGui::Text("cached pages: %u", streaming_manager.m_free_pages.size());
            ImGui::Text("dt: %fms", 1000 * dt);
            ImGui::Text("allocated: %i/%i", streaming_manager.allocated_pages,
                        streaming_manager.page_count());
            static bool stream_continue = false;
            if (ImGui::Button("stream once"))
            {
                stream = true;
            }
            ImGui::Checkbox("continue_streaming", &stream_continue);
            if (stream_continue)
                stream = true;

            if (false)
                for (uint32_t i = 0; i < streaming_manager.page_count(); i++)
                {
                    uint32_t depth = streaming_manager
                                         .get_build_info(streaming_manager.m_page_parent_index[i])
                                         .depth;
                    std::string str = "        ";
                    str[depth] = '\0';
                    ImGui::Text("%s %i %u", str.c_str(),
                                int(streaming_manager.m_page_lru[i]) - int(frame_index),
                                streaming_manager.m_page_parent_index[i]);
                }
        }
        ImGui::End();
    }

    void handle_input(float dt)
    {
        input.update(window);
        glm::vec3 right_vector = glm::normalize(
            glm::cross(camera.get_forward_vector(), world_direction::up));

        if (input.get_key(GLFW_KEY_W))
            camera.translate(dt * speed * camera.get_forward_vector());
        if (input.get_key(GLFW_KEY_S))
            camera.translate(-dt * speed * camera.get_forward_vector());
        if (input.get_key(GLFW_KEY_A))
            camera.translate(-dt * speed * right_vector);
        if (input.get_key(GLFW_KEY_D))
            camera.translate(dt * speed * right_vector);

        if (input.is_key_held(GLFW_KEY_F))
            speed *= glm::pow(8.f, dt);
        if (input.is_key_held(GLFW_KEY_R))
            speed /= glm::pow(8.f, dt);

        if (input.is_button_held(GLFW_MOUSE_BUTTON_RIGHT))
            camera.rotate(input.get_mouse_dx() * sensitivity, input.get_mouse_dy() * sensitivity);
        if (input.is_key_clicked(GLFW_KEY_Q))
            streaming_manager.upload_to_gpu();
    }

    void setup_camera()
    {
        FPSCameraSettings settings;

        settings.position = {0, 0, 0};
        settings.direction = glm::normalize(glm::vec3(1, 0, 1));
        settings.near_plane = 0.125f;
        settings.far_plane = 2048.f;
        settings.fov_degrees = 45.f;
        settings.width = width;
        settings.height = height;
        camera.init(settings);

        camera.set_projection(settings);
    }

    void draw_aabb(const glm::vec3& min, const glm::vec3& size)
    {
        if (glm::any(glm::greaterThanEqual(size, glm::vec3(draw_aabb_minmax_size.y))))
            return;
        if (glm::any(glm::lessThan(size, glm::vec3(draw_aabb_minmax_size.x))))
            return;

        sp_aabb.bind();

        sp_aabb.set_mat4("u_mvp", camera.get_vp_matrix());
        sp_aabb.set_vec3("u_min", min);
        sp_aabb.set_vec3("u_size", size);

        glBindVertexArray(empty_vao);
        glDrawArrays(GL_LINES, 0, 24);
        glBindVertexArray(0);

        sp_aabb.unbind();
    }

    void draw_probes()
    {
        sp_probes_debug.bind();

        probes.bind_base(0);

        sp_probes_debug.set_sampler("u_depth", depth_texture.get_handle(), 0, GL_TEXTURE_2D);
        sp_probes_debug.set_mat4("u_view", camera.get_view_matrix());
        sp_probes_debug.set_mat4("u_proj", camera.get_projection_matrix());
        sp_probes_debug.set_float("u_near", 0.125f);
        sp_probes_debug.set_float("u_far", 2048.f);
        sp_probes_debug.set_float("u_probe_spacing", probe_spacing);
        sp_probes_debug.set_float("u_point_size", probe_point_size);
        sp_probes_debug.set_vec2("u_screen_size",
                                 glm::vec2(final_texture.width(), final_texture.height()));

        glBindVertexArray(empty_vao);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, 4096);
    }

    void draw_probe_rays()
    {
        sp_probes_debug_ray.bind();
        probes.bind_base(0);

        sp_probes_debug_ray.set_vec2("u_screen_size",
                                     glm::vec2(final_texture.width(), final_texture.height()));
        sp_probes_debug_ray.set_sampler("u_depth", depth_texture.get_handle(), 0, GL_TEXTURE_2D);
        sp_probes_debug_ray.set_mat4("u_view", camera.get_view_matrix());
        sp_probes_debug_ray.set_mat4("u_proj", camera.get_projection_matrix());
        sp_probes_debug_ray.set_float("u_probe_spacing", probe_spacing);
        sp_probes_debug_ray.set_float("u_near", 0.125f);
        sp_probes_debug_ray.set_float("u_far", 2048.f);

        glBindVertexArray(empty_vao);
        glDrawArraysInstanced(GL_LINES, 0, 2, 4096);
    }
    glm::vec2 draw_aabb_minmax_size = glm::vec2(0, 100);
    void draw_aabb(const glm::vec3& min, float size) { draw_aabb(min, glm::vec3(size)); }

    GLFWwindow* window;
    InputState input;

    uint32_t width;
    uint32_t height;
    bool is_running;

    // Your application variables here:
    uint32_t frame_index = 0;
    float chunk_size = 128.f;
    VoxelStreamingManager streaming_manager;

    Ssbo request_buffers[2];
    Ssbo request_bitmasks[2];
    GLsync fences[2];

    FPSCamera camera;
    float speed = 16.f;
    float sensitivity = 0.01f;

    Shader sp_raymarch;
    Shader sp_aabb;
    Shader sp_request_bitmask_gen;
    Shader sp_compact_requests;
    Shader sp_irradiance_raymarch;
    Shader sp_composite;
    Shader sp_probes_debug;
    Shader sp_probes_debug_ray;
    Shader sp_ssao;

    Texture request_imgs[3];
    Ssbo request_node_bitmask;
    Ssbo compact_nodes[2];
    Ssbo compact_requests_indirect;

    inline static const char* g_buffer_names[] = {
        "final",        "albedo",     "depth",       "normal",      "indirect",
        "subdiv_depth", "iterations", "probe_depth", "probe_color", "ao"};
    Texture albedo_texture;
    Texture depth_texture;
    Texture normals_texture;
    Texture final_texture;
    Texture debug_texture;
    Texture ao_texture;

    Ssbo probes;
    Texture probes_depths;
    Texture probes_colors;

    uint32_t fbo = 0;
    uint32_t rbo = 0;
    uint32_t empty_vao = 0;

    Timer profiling_timer;

    std::vector<glm::vec4> aabbs;

    uint32_t display_texture = 0;
    float display_brightness = 1.f;
    float viewport_scale = 1.5f;

    float ssao_radius = 0.1f;
    float ssao_bias = 0.1f;

    float probe_spacing = 16.f;
    float probe_point_size = 3.f;
    float probe_hysteresis = 0.95f;
    glm::ivec3 probe_counts = {32, 32, 32};
    int probe_resolution = 16;
};