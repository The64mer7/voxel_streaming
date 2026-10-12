#pragma once
#include <cstdint>
#include <glad/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

struct Texture
{
    void create(uint32_t internal_format)
    {
        m_internal_format = internal_format;
        glCreateTextures(GL_TEXTURE_2D, 1, &m_handle);
    }
    void allocate(uint64_t width, uint64_t height)
    {
        m_width = width;
        m_height = height;

        glTextureStorage2D(m_handle, 1, m_internal_format, width, height);

        glTextureParameteri(m_handle, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTextureParameteri(m_handle, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    }
    void upload(GLenum format, GLenum type, void* data)
    {
        glTextureSubImage2D(m_handle, 0, 0, 0, width(), height(), format, type, data);
    }
    void destroy()
    {
        glDeleteTextures(1, &m_handle);
        m_handle = 0;
    }

    GLenum get_internal_format() { return m_internal_format; }

    GLuint get_handle() { return m_handle; }
    uint64_t width() { return m_width; }
    uint64_t height() { return m_height; }

private:
    GLenum m_internal_format;

    GLuint m_handle = 0;
    uint64_t m_width = 0;
    uint64_t m_height = 0;
};