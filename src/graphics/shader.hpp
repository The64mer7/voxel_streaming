#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

#include <glad/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "graphics/buffer.hpp"

struct DispatchIndirectCommand
{
    uint32_t num_groups_x;
    uint32_t num_groups_y;
    uint32_t num_groups_z;
};

class Shader
{
public:
    unsigned int program_id = 0;

    Shader() = default;

    void destroy()
    {
        if (program_id != 0)
            glDeleteProgram(program_id);
        program_id = 0;
    }

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    Shader(Shader&& other) noexcept
    {
        program_id = other.program_id;
        other.program_id = 0;
        m_vertex_path = std::move(other.m_vertex_path);
        m_fragment_path = std::move(other.m_fragment_path);
        m_compute_path = std::move(other.m_compute_path);
        m_is_compute = other.m_is_compute;
        uniform_location_cache = std::move(other.uniform_location_cache);
    }

    Shader& operator=(Shader&& other) noexcept
    {
        if (this != &other)
        {
            if (program_id != 0)
                glDeleteProgram(program_id);
            program_id = other.program_id;
            other.program_id = 0;
            m_vertex_path = std::move(other.m_vertex_path);
            m_fragment_path = std::move(other.m_fragment_path);
            m_compute_path = std::move(other.m_compute_path);
            m_is_compute = other.m_is_compute;
            uniform_location_cache = std::move(other.uniform_location_cache);
        }
        return *this;
    }

    bool reload()
    {
        if (m_is_compute)
        {
            return reload_compute();
        }
        return reload_graphics();
    }

    bool load_graphics(const std::string& vertex_path, const std::string& fragment_path)
    {
        uniform_location_cache.clear();
        m_vertex_path = vertex_path;
        m_fragment_path = fragment_path;
        m_is_compute = false;

        std::string vertex_code = read_file(vertex_path);
        std::string fragment_code = read_file(fragment_path);

        if (vertex_code.empty() || fragment_code.empty())
            return false;

        unsigned int vertex = compile_shader(vertex_code, GL_VERTEX_SHADER);
        unsigned int fragment = compile_shader(fragment_code, GL_FRAGMENT_SHADER);

        if (!vertex || !fragment)
        {
            if (vertex)
                glDeleteShader(vertex);
            if (fragment)
                glDeleteShader(fragment);
            return false;
        }

        unsigned int new_program = glCreateProgram();
        glAttachShader(new_program, vertex);
        glAttachShader(new_program, fragment);
        glLinkProgram(new_program);

        bool success = check_compile_errors(new_program, "PROGRAM");

        glDeleteShader(vertex);
        glDeleteShader(fragment);

        if (success)
        {
            if (program_id != 0)
                glDeleteProgram(program_id);
            program_id = new_program;
            uniform_location_cache.clear();
        }
        else
        {
            glDeleteProgram(new_program);
        }

        return success;
    }

    bool load_compute(const std::string& compute_path)
    {
        uniform_location_cache.clear();
        m_compute_path = compute_path;
        m_is_compute = true;

        std::string compute_code = read_file(compute_path);
        if (compute_code.empty())
            return false;

        unsigned int compute = compile_shader(compute_code, GL_COMPUTE_SHADER);
        if (!compute)
            return false;

        unsigned int new_program = glCreateProgram();
        glAttachShader(new_program, compute);
        glLinkProgram(new_program);

        bool success = check_compile_errors(new_program, "PROGRAM");
        glDeleteShader(compute);

        if (success)
        {
            if (program_id != 0)
                glDeleteProgram(program_id);
            program_id = new_program;
            uniform_location_cache.clear();
        }
        else
        {
            glDeleteProgram(new_program);
        }

        return success;
    }

    void bind() const { glUseProgram(program_id); }

    void unbind() const { glUseProgram(0); }

    void dispatch_compute(unsigned int group_x, unsigned int group_y, unsigned int group_z,
                          GLenum barrier_flags = GL_SHADER_STORAGE_BARRIER_BIT) const
    {
        bind();
        glDispatchCompute(group_x, group_y, group_z);
        glMemoryBarrier(barrier_flags);
    }

    void dispatch_compute_grid(const glm::uvec3& total_size, const glm::uvec3& group_size,
                               GLenum barrier_flags)
    {
        bind();
        glm::uvec3 groups = (total_size + group_size - 1u) / group_size;
        glDispatchCompute(groups.x, groups.y, groups.z);
        glMemoryBarrier(barrier_flags);
    }

    void dispatch_compute(const glm::uvec3& groups,
                          GLenum barrier_flags = GL_SHADER_STORAGE_BARRIER_BIT) const
    {
        bind();
        glDispatchCompute(groups.x, groups.y, groups.z);
        glMemoryBarrier(barrier_flags);
    }

    void dispatch_compute_indirect(const Ssbo& ssbo, GLenum barrier_flags)
    {
        glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, ssbo.buffer_id);
        glDispatchComputeIndirect(0);
        glMemoryBarrier(barrier_flags);
    }

    void set_int(const std::string& name, int value) const
    {
        glUniform1i(get_uniform_location(name), value);
    }

    void set_uint(const std::string& name, unsigned int value) const
    {
        glUniform1ui(get_uniform_location(name), value);
    }

    void set_float(const std::string& name, float value) const
    {
        glUniform1f(get_uniform_location(name), value);
    }

    void set_vec2(const std::string& name, const glm::vec2& value) const
    {
        glUniform2fv(get_uniform_location(name), 1, glm::value_ptr(value));
    }

    void set_vec3(const std::string& name, const glm::vec3& value) const
    {
        glUniform3fv(get_uniform_location(name), 1, glm::value_ptr(value));
    }

    void set_vec4(const std::string& name, const glm::vec4& value) const
    {
        glUniform4fv(get_uniform_location(name), 1, glm::value_ptr(value));
    }

    void set_ivec2(const std::string& name, const glm::ivec2& value) const
    {
        glUniform2iv(get_uniform_location(name), 1, glm::value_ptr(value));
    }

    void set_ivec3(const std::string& name, const glm::ivec3& value) const
    {
        glUniform3iv(get_uniform_location(name), 1, glm::value_ptr(value));
    }

    void set_ivec4(const std::string& name, const glm::ivec4& value) const
    {
        glUniform4iv(get_uniform_location(name), 1, glm::value_ptr(value));
    }

    void set_uvec2(const std::string& name, const glm::uvec2& value) const
    {
        glUniform2uiv(get_uniform_location(name), 1, glm::value_ptr(value));
    }

    void set_uvec3(const std::string& name, const glm::uvec3& value) const
    {
        glUniform3uiv(get_uniform_location(name), 1, glm::value_ptr(value));
    }

    void set_uvec4(const std::string& name, const glm::uvec4& value) const
    {
        glUniform4uiv(get_uniform_location(name), 1, glm::value_ptr(value));
    }

    void set_mat2(const std::string& name, const glm::mat2& mat) const
    {
        glUniformMatrix2fv(get_uniform_location(name), 1, GL_FALSE, glm::value_ptr(mat));
    }

    void set_mat3(const std::string& name, const glm::mat3& mat) const
    {
        glUniformMatrix3fv(get_uniform_location(name), 1, GL_FALSE, glm::value_ptr(mat));
    }

    void set_mat4(const std::string& name, const glm::mat4& mat) const
    {
        glUniformMatrix4fv(get_uniform_location(name), 1, GL_FALSE, glm::value_ptr(mat));
    }

    void set_sampler(const std::string& name, GLuint texture, GLuint texture_unit,
                     GLenum target = GL_TEXTURE_2D) const
    {
        glActiveTexture(GL_TEXTURE0 + texture_unit);
        glBindTexture(target, texture);
        glUniform1i(get_uniform_location(name), static_cast<int>(texture_unit));
    }

    void set_image(const std::string& name, GLuint texture, uint32_t image_unit, GLenum access,
                   GLenum format = GL_RGBA8) const
    {
        glBindImageTexture(image_unit, texture, 0, GL_FALSE, 0, access, format);
        glUniform1i(get_uniform_location(name), static_cast<int>(image_unit));
    }

private:
    bool reload_compute() { return load_compute(m_compute_path); }

    bool reload_graphics() { return load_graphics(m_vertex_path, m_fragment_path); }

    mutable std::unordered_map<std::string, int> uniform_location_cache;
    std::string m_vertex_path;
    std::string m_fragment_path;
    std::string m_compute_path;
    bool m_is_compute = false;

    int get_uniform_location(const std::string& name) const
    {
        if (uniform_location_cache.find(name) != uniform_location_cache.end())
        {
            return uniform_location_cache[name];
        }
        int location = glGetUniformLocation(program_id, name.c_str());
        uniform_location_cache[name] = location;
        return location;
    }

    std::string read_file(const std::string& filepath) const
    {
        std::ifstream file;
        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        try
        {
            file.open(filepath);
            std::stringstream stream;
            stream << file.rdbuf();
            file.close();
            return stream.str();
        }
        catch (std::ifstream::failure&)
        {
            std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ: " << filepath << std::endl;
            return "";
        }
    }

    unsigned int compile_shader(const std::string& code, GLenum type) const
    {
        const char* code_cstr = code.c_str();
        unsigned int shader = glCreateShader(type);
        glShaderSource(shader, 1, &code_cstr, nullptr);
        glCompileShader(shader);

        if (!check_compile_errors(shader, type == GL_VERTEX_SHADER     ? "VERTEX"
                                          : type == GL_FRAGMENT_SHADER ? "FRAGMENT"
                                                                       : "COMPUTE"))
        {
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    }

    bool check_compile_errors(unsigned int shader, const std::string& type) const
    {
        int success;
        char info_log[1024];
        if (type != "PROGRAM")
        {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success)
            {
                glGetShaderInfoLog(shader, 1024, nullptr, info_log);
                std::cerr << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
                          << info_log
                          << "\n -- --------------------------------------------------- -- "
                          << std::endl;
                return false;
            }
        }
        else
        {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success)
            {
                glGetProgramInfoLog(shader, 1024, nullptr, info_log);
                std::cerr << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                          << info_log
                          << "\n -- --------------------------------------------------- -- "
                          << std::endl;
                return false;
            }
        }
        return true;
    }
};