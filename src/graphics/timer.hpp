// gpu_timer.hpp
#pragma once

#include <glad/gl.h>

#include <cstddef>
#include <string>
#include <vector>

class GpuTimer
{
public:
    explicit GpuTimer(std::size_t num_queries, std::size_t ring_size = 4)
        : m_num_queries(num_queries), m_ring_size(ring_size), m_queries(num_queries * ring_size, 0),
          m_active_queries(num_queries, false), m_initialized(false), m_current_frame(0)
    {
    }

    ~GpuTimer()
    {
        if (m_initialized)
            glDeleteQueries(static_cast<GLsizei>(m_queries.size()), m_queries.data());
    }

    GpuTimer(const GpuTimer&) = delete;
    GpuTimer& operator=(const GpuTimer&) = delete;

    void begin(std::size_t query_index)
    {
        initialize();

        if (m_active_queries[query_index])
            return;

        glBeginQuery(GL_TIME_ELAPSED, get_query(query_index));
        m_active_queries[query_index] = true;
    }

    void end(std::size_t query_index)
    {
        if (!m_active_queries[query_index])
            return;

        glEndQuery(GL_TIME_ELAPSED);
        m_active_queries[query_index] = false;
    }

    void update() { m_current_frame = (m_current_frame + 1) % m_ring_size; }

    bool get_result_ms(std::size_t query_index, double& result_ms) const
    {
        if (!m_initialized)
            return false;

        const GLuint query = get_query(query_index);

        GLint available = GL_FALSE;
        glGetQueryObjectiv(query, GL_QUERY_RESULT_AVAILABLE, &available);

        if (!available)
            return false;

        GLuint64 elapsed_ns = 0;
        glGetQueryObjectui64v(query, GL_QUERY_RESULT, &elapsed_ns);

        result_ms = static_cast<double>(elapsed_ns) / 1'000'000.0;
        return true;
    }

private:
    std::size_t m_num_queries;
    std::size_t m_ring_size;
    std::vector<GLuint> m_queries;
    std::vector<bool> m_active_queries;
    bool m_initialized;
    std::size_t m_current_frame;

    void initialize()
    {
        if (m_initialized)
            return;

        glGenQueries(static_cast<GLsizei>(m_queries.size()), m_queries.data());

        m_initialized = true;
    }

    GLuint get_query(std::size_t query_index) const
    {
        return m_queries[m_current_frame * m_num_queries + query_index];
    }
};