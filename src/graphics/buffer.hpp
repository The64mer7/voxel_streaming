#pragma once

#include <cstddef>
#include <glad/gl.h>

class Ssbo
{
public:
    unsigned int buffer_id = 0;
    size_t size_in_bytes = 0;

    Ssbo() = default;

    Ssbo(const void* data, size_t size, GLbitfield flags = GL_DYNAMIC_STORAGE_BIT)
    {
        allocate(data, size, flags);
    }

    void destroy()
    {
        if (buffer_id != 0)
            glDeleteBuffers(1, &buffer_id);
        buffer_id = 0;
    }

    Ssbo(const Ssbo&) = delete;
    Ssbo& operator=(const Ssbo&) = delete;

    Ssbo(Ssbo&& other) noexcept
    {
        buffer_id = other.buffer_id;
        size_in_bytes = other.size_in_bytes;
        other.buffer_id = 0;
        other.size_in_bytes = 0;
    }

    Ssbo& operator=(Ssbo&& other) noexcept
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
    void callocate(const void* data, size_t count, GLbitfield flags = GL_DYNAMIC_STORAGE_BIT)
    {
        if (buffer_id == 0)
        {
            glCreateBuffers(1, &buffer_id);
        }
        size_in_bytes = count * sizeof(T);
        glNamedBufferStorage(buffer_id, static_cast<GLsizeiptr>(size_in_bytes), data, flags);
    }

    void set_sub_data(const void* data, size_t size, size_t offset = 0) const
    {
        glNamedBufferSubData(buffer_id, static_cast<GLintptr>(offset),
                             static_cast<GLsizeiptr>(size), data);
    }

    void clear(GLubyte data) const
    {
        if (buffer_id != 0 && size_in_bytes > 0)
        {
            glClearNamedBufferData(buffer_id, GL_R8UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, &data);
        }
    }

    void bind_base(unsigned int binding_index) const
    {
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding_index, buffer_id);
    }

    void* map(GLenum access = GL_MAP_READ_BIT | GL_MAP_WRITE_BIT) const
    {
        return map_range(0, size_in_bytes, access);
    }

    void* map_range(size_t offset, size_t length, GLbitfield access_flags) const
    {
        return glMapNamedBufferRange(buffer_id, static_cast<GLintptr>(offset),
                                     static_cast<GLsizeiptr>(length), access_flags);
    }

    void unmap() const { glUnmapNamedBuffer(buffer_id); }
};