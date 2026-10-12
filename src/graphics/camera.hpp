#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace world_direction
{
constexpr glm::vec3 up = glm::vec3(0.f, 1.f, 0.f);
constexpr glm::vec3 right = glm::vec3(1.f, 0.f, 0.f);
constexpr glm::vec3 forward = glm::vec3(0.f, 0.f, -1.f);
} // namespace world_direction

struct FPSCameraSettings
{
    glm::vec3 position;
    glm::vec3 direction;
    float fov_degrees;
    float near_plane;
    float far_plane;
    float width;
    float height;
    float walk_speed;
    int key_forward;
    int key_backward;
    int key_left;
    int key_right;
    glm::vec2 sensitivity;
};

class FPSCamera
{
public:
    FPSCamera() = default;
    FPSCamera(const FPSCameraSettings& settings) { init(settings); }

    void init(const FPSCameraSettings& settings)
    {
        m_position = settings.position;
        m_pitch = asin(settings.direction.y);
        m_yaw = atan2(settings.direction.z, settings.direction.x);
        set_projection(settings);
    }

    void set_projection(const FPSCameraSettings& settings)
    {
        float aspect = settings.width / settings.height;
        m_projection_matrix = glm::perspective(glm::radians(settings.fov_degrees), aspect,
                                               settings.near_plane, settings.far_plane);

        float ortho_height = 20.f;
        float ortho_width = aspect * ortho_height;
        m_ortho_projection_matrix = glm::ortho(-ortho_width, ortho_width, -ortho_height,
                                               ortho_height, -settings.far_plane,
                                               settings.far_plane);
    }

    void set_projection(float far_plane, float near_plane, float fov, float width, float height)
    {
        float aspect = width / height;
        m_projection_matrix = glm::perspective(glm::radians(fov), aspect, near_plane, far_plane);

        float ortho_height = 20.f;
        float ortho_width = aspect * ortho_height;
        m_ortho_projection_matrix = glm::ortho(-ortho_width, ortho_width, -ortho_height,
                                               ortho_height, -far_plane, far_plane);
    }

    void translate(glm::vec3 v) { m_position += v; }

    void set_position(glm::vec3 v) { m_position = v; }

    glm::vec3 get_forward_vector() const
    {
        glm::vec3 forward_vec;
        forward_vec.x = cos(m_yaw) * cos(m_pitch);
        forward_vec.y = sin(m_pitch);
        forward_vec.z = sin(m_yaw) * cos(m_pitch);
        return glm::normalize(forward_vec);
    }

    glm::vec3 get_position() const { return m_position; }

    glm::mat4 get_view_matrix() const
    {
        return glm::lookAt(get_position(), get_position() + get_forward_vector(),
                           world_direction::up);
    }

    glm::mat4 get_projection_matrix() const { return m_projection_matrix; }

    glm::mat4 get_ortho_projection_matrix() const { return m_ortho_projection_matrix; }

    glm::mat4 get_vp_matrix() const { return m_projection_matrix * get_view_matrix(); }

    void rotate(float delta_yaw, float delta_pitch)
    {
        m_yaw += delta_yaw;
        m_pitch -= delta_pitch;
        m_pitch = glm::clamp(m_pitch, glm::radians(-89.f), glm::radians(89.f));
    }

private:
    glm::vec3 m_position;
    float m_yaw;
    float m_pitch;
    glm::mat4 m_projection_matrix;
    glm::mat4 m_ortho_projection_matrix;
};

struct chunk_relative_first_person_camera
{
    FPSCamera camera;
    glm::ivec3 chunk_offset;

    glm::vec3 get_world_pos()
    {
        return glm::vec3(chunk_offset) * m_chunk_size + camera.get_position();
    }

    void update()
    {
        if (glm::any(
                glm::greaterThanEqual(glm::abs(camera.get_position()), glm::vec3(m_chunk_size))))
        {
            chunk_offset += (camera.get_position() / m_chunk_size);
            camera.translate(-glm::vec3(chunk_offset) * m_chunk_size);
        }
    }

private:
    float m_chunk_size = 8.f;
};