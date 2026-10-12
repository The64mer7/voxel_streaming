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

#include <format>
#include <iostream>
#include <unordered_set>

#include "core.hpp"

#include "terrain_generator/contree_gen.hpp"
#include "voxels/streaming_manager.hpp"

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
        constexpr uint32_t num_nodes = 1u << 18u;
        streaming_manager.init(num_nodes, 64);
        if (!sp_raymarch.load_compute("shaders/raymarch.comp"))
            exit(1);
        if (!sp_aabb.load_graphics("shaders/aabb.vert", "shaders/aabb.frag"))
            exit(1);

        uint32_t w = 1000;
        uint32_t h = 700;
        albedo.create(GL_RGBA8);
        albedo.allocate(w, h);

        final_texture.create(GL_RGBA8);
        final_texture.allocate(w, h);

        requests_img.create(GL_R32UI);
        requests_img.allocate(w, h);

        for (int i = 0; i < 2; i++)
        {
            request_buffers[i].allocate(nullptr, (1024 * 1024 + 1) * sizeof(uint32_t),
                                        GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT);
            uint32_t zero = 0;
            request_buffers[i].set_sub_data(&zero, sizeof(uint32_t), 0);
            fences[i] = nullptr;
        }

        if (!fbo)
            glCreateFramebuffers(1, &fbo);
        if (!rbo)
            glCreateRenderbuffers(1, &rbo);

        glNamedRenderbufferStorage(rbo, GL_DEPTH24_STENCIL8, width, height);

        glNamedFramebufferTexture(fbo, GL_COLOR_ATTACHMENT0, albedo.get_handle(), 0);
        glNamedFramebufferRenderbuffer(fbo, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

        if (glCheckNamedFramebufferStatus(fbo, GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            printf("error creating framebuffer\n");
            exit(1);
        }

        glCreateVertexArrays(1, &empty_vao);

        setup_camera();
    }

    void tick(float dt)
    {
        printf("frame %u\n", frame_index);
        handle_input(dt);
        streaming_manager.update(frame_index);

        uint32_t slot = frame_index % 2;

        if (fences[slot])
        {
            glClientWaitSync(fences[slot], GL_SYNC_FLUSH_COMMANDS_BIT, 1000000000);
            glDeleteSync(fences[slot]);
            fences[slot] = nullptr;

            uint32_t* requests = (uint32_t*)request_buffers[slot].map(GL_MAP_READ_BIT);

            uint32_t num_requests = requests[0];
            printf("requests from gpu %u\n", num_requests);

            static std::vector<uint32_t> request_nodes;
            if (num_requests)
            {
                request_nodes.resize(num_requests);
                memcpy(request_nodes.data(), requests + 1, sizeof(uint32_t) * num_requests);

                static std::unordered_set<uint32_t> unique_requests;
                unique_requests.clear();
                compact_requests.clear();
                for (uint32_t i = 0; i < num_requests; i++)
                {
                    if (!unique_requests.contains(request_nodes[i]))
                    {
                        unique_requests.insert(request_nodes[i]);
                        compact_requests.push_back(request_nodes[i]);
                    }
                }
                printf("requests after compaction %u\n", compact_requests.size());
                static constexpr auto gen_info = GenerateInfo{SphereContree<NodeT>::create3d,
                                                              SphereContree<NodeT>::homogeneous3d};

                if (stream)
                {
                    streaming_manager.update_lru(frame_index, compact_requests);
                    streaming_manager.process_subdivision_requests(
                        gen_info, compact_requests.data(), compact_requests.size());
                }
                stream = false;
            }
            request_buffers[slot].unmap();

            uint32_t zero = 0;
            request_buffers[slot].set_sub_data(&zero, sizeof(uint32_t), 0);
        }

        sp_raymarch.bind();

        streaming_manager.bind_buffer(1);
        request_buffers[slot].bind_base(2);
        sp_raymarch.set_image("u_albedo", albedo.get_handle(), 0, GL_READ_WRITE,
                              albedo.get_internal_format());
        sp_raymarch.set_mat4("u_inv_view", glm::inverse(camera.get_view_matrix()));
        sp_raymarch.set_mat4("u_inv_proj", glm::inverse(camera.get_projection_matrix()));
        sp_raymarch.set_vec3("u_cam_pos", camera.get_position());

        glm::uvec3 groups;
        groups.x = (albedo.width() + 15) / 16;
        groups.y = (albedo.height() + 15) / 16;
        groups.z = 1;
        sp_raymarch.dispatch_compute(groups.x, groups.y, groups.z);

        fences[slot] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);

        frame_index++;
    }
    std::vector<uint32_t> compact_requests;

    bool stream = false;
    void render(float dt)
    {
        glViewport(0, 0, albedo.width(), albedo.height());
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        for (uint32_t i = 0; i < compact_requests.size(); i++)
        {
            // BuildInfo info = streaming_manager.get_build_info(compact_requests[i]);
            // draw_aabb(info.chunk_origin, glm::vec3(info.chunk_size));
        }

        for (uint32_t i = 0; i < streaming_manager.page_count(); i++)
        {
            if (streaming_manager.m_page_parent_index[i] != UINT32_MAX &&
                streaming_manager.page_age(i) > 16)
            {
                uint32_t node_idx = streaming_manager.m_page_parent_index[i];

                BuildInfo info = streaming_manager.get_build_info(node_idx);
                // draw_aabb(info.chunk_origin, glm::vec3(info.chunk_size));
            }
        }

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        if (ImGui::Begin("debug"))
        {
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

        if (ImGui::Begin("display"))
        {
            ImGui::Image(albedo.get_handle(), ImVec2(albedo.width(), albedo.height()),
                         ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
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
            speed *= glm::pow(2.f, dt);
        if (input.is_key_held(GLFW_KEY_R))
            speed /= glm::pow(2.f, dt);

        if (input.is_button_held(GLFW_MOUSE_BUTTON_RIGHT))
            camera.rotate(input.get_mouse_dx() * sensitivity, input.get_mouse_dy() * sensitivity);
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
        sp_aabb.bind();

        sp_aabb.set_mat4("u_mvp", camera.get_vp_matrix());
        sp_aabb.set_vec3("u_min", min);
        sp_aabb.set_vec3("u_size", size);

        glBindVertexArray(empty_vao);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);

        sp_aabb.unbind();
    }

    GLFWwindow* window;
    InputState input;

    uint32_t width;
    uint32_t height;
    bool is_running;

    // Your application variables here:
    uint32_t frame_index = 0;
    VoxelStreamingManager streaming_manager;

    Ssbo request_buffers[2];
    GLsync fences[2];

    FPSCamera camera;
    float speed = 16.f;
    float sensitivity = 0.01f;

    Shader sp_raymarch;
    Shader sp_aabb;

    Texture albedo;
    Texture requests_img;
    Texture final_texture;

    uint32_t fbo = 0;
    uint32_t rbo = 0;
    uint32_t empty_vao = 0;
};