#pragma once

#include <cstddef>
#include <glad/gl.h>

class Ubo
{
public:
    unsigned int buffer_id = 0;
    size_t size_in_bytes = 0;

    Ubo() = default;

    template <typename T>
    Ubo(const T& data, GLbitfield flags = GL_DYNAMIC_STORAGE_BIT)
    {
        allocate(data, flags);
    }

    Ubo(const void* data, size_t size, GLbitfield flags = GL_DYNAMIC_STORAGE_BIT)
    {
        allocate(data, size, flags);
    }

    ~Ubo()
    {
        if (buffer_id != 0)
        {
            glDeleteBuffers(1, &buffer_id);
        }
    }

    Ubo(const Ubo&) = delete;
    Ubo& operator=(const Ubo&) = delete;

    Ubo(Ubo&& other) noexcept
    {
        buffer_id = other.buffer_id;
        size_in_bytes = other.size_in_bytes;
        other.buffer_id = 0;
        other.size_in_bytes = 0;
    }

    Ubo& operator=(Ubo&& other) noexcept
    {
        if (this != &other)
        {
            if (buffer_id != 0)
                glDeleteBuffers(1, &buffer_id);
            buffer_id = other.buffer_id;
            size_in_bytes = other.size_in_bytes;
            other.buffer_id = 0;
            other.size_in_bytes = 0;
        }
        return *this;
    }

    template <typename T>
    void allocate(const T& data, GLbitfield flags = GL_DYNAMIC_STORAGE_BIT)
    {
        allocate(&data, sizeof(T), flags);
    }

    void allocate(const void* data, size_t size, GLbitfield flags = GL_DYNAMIC_STORAGE_BIT)
    {
        if (buffer_id == 0)
        {
            glCreateBuffers(1, &buffer_id);
        }
        size_in_bytes = size;
        glNamedBufferStorage(buffer_id, static_cast<GLsizeiptr>(size), data, flags);
    }

    template <typename T>
    void set_data(const T& data) const
    {
        glNamedBufferSubData(buffer_id, 0, sizeof(T), &data);
    }

    void set_sub_data(const void* data, size_t size, size_t offset = 0) const
    {
        glNamedBufferSubData(buffer_id, static_cast<GLintptr>(offset),
                             static_cast<GLsizeiptr>(size), data);
    }

    void bind_base(unsigned int binding_index) const
    {
        glBindBufferBase(GL_UNIFORM_BUFFER, binding_index, buffer_id);
    }

    void bind_range(unsigned int binding_index, size_t offset, size_t size) const
    {
        glBindBufferRange(GL_UNIFORM_BUFFER, binding_index, buffer_id,
                          static_cast<GLintptr>(offset), static_cast<GLsizeiptr>(size));
    }
};