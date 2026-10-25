#pragma once
#include <cstdint>
#include <glad/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

struct Texture
{
    void create(uint32_t internal_format, GLenum target = GL_TEXTURE_2D)
    {
        m_target = target;
        m_internal_format = internal_format;
        glCreateTextures(target, 1, &m_handle);
    }
    void allocate(GLsizei width, GLsizei height, GLint filter = GL_NEAREST)
    {
        m_width = width;
        m_height = height;
        m_depth = 1;

        glTextureStorage2D(m_handle, 1, m_internal_format, width, height);

        glTextureParameteri(m_handle, GL_TEXTURE_MAG_FILTER, filter);
        glTextureParameteri(m_handle, GL_TEXTURE_MIN_FILTER, filter);
    }
    void allocate_3d(GLsizei width, GLsizei height, GLsizei depth, GLint filter = GL_NEAREST)
    {
        m_width = width;
        m_height = height;
        m_depth = depth;

        glTextureStorage3D(m_handle, 1, m_internal_format, width, height, depth);

        glTextureParameteri(m_handle, GL_TEXTURE_MAG_FILTER, filter);
        glTextureParameteri(m_handle, GL_TEXTURE_MIN_FILTER, filter);
    }
    void upload(GLenum format, GLenum type, const void* data)
    {
        glTextureSubImage2D(m_handle, 0, 0, 0, width(), height(), format, type, data);
    }

    void upload_3d(GLenum format, GLenum type, const void* data)
    {
        glTextureSubImage3D(m_handle, 0, 0, 0, 0, m_width, m_height, m_depth, format, type, data);
    }

    void destroy()
    {
        if (m_handle != 0)
            glDeleteTextures(1, &m_handle);
        m_handle = 0;
    }

    GLenum get_target() const { return m_target; }
    GLenum get_internal_format() const { return m_internal_format; }
    GLuint get_handle() const { return m_handle; }
    GLsizei width() const { return m_width; }
    GLsizei height() const { return m_height; }
    GLsizei depth() const { return m_depth; }

private:
    GLenum m_internal_format;

    GLuint m_handle = 0;
    GLsizei m_width = 0;
    GLsizei m_height = 0;
    GLsizei m_depth = 0;
    GLenum m_target = 0;
};
