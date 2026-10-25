#pragma once

#include <glad/gl.h>

#include <cstddef>
#include <string>
#include <vector>

class Timer
{
public:
    void destroy()
    {
        for (auto& node : queries)
        {
            glDeleteQueries(node.num_queries, node.begin_queries);
            glDeleteQueries(node.num_queries, node.end_queries);
        }
    }

    void update()
    {
        for (auto& node : queries)
        {
            node.current_query_idx = (node.current_query_idx + 1) % node.num_queries;
        }
        curr_nested_depth = 0;
        frame_index++;
    }

    template <typename ForEachPFN>
    void for_each_gpu_time(ForEachPFN fn_for_each)
    {
        for (int i = 0; i < queries.size(); i++)
        {
            auto& node = queries[i];
            uint32_t read_idx = (node.current_query_idx + 1) % node.num_queries;

            int avail = 0;
            glGetQueryObjectiv(node.end_queries[read_idx], GL_QUERY_RESULT_AVAILABLE, &avail);

            if (avail && int32_t(frame_index) - int32_t(lru[i]) < node.num_queries)
            {
                uint64_t begin = 0;
                uint64_t end = 0;

                glGetQueryObjectui64v(node.begin_queries[read_idx], GL_QUERY_RESULT, &begin);
                glGetQueryObjectui64v(node.end_queries[read_idx], GL_QUERY_RESULT, &end);

                uint64_t elapsed = end - begin;

                fn_for_each(names[i], elapsed, node.depth);
            }
        }
    }

    void begin_gpu(const std::string& name)
    {
        auto it = name_to_query.find(name);
        if (it == name_to_query.end())
        {
            ProfilingNode node;
            glCreateQueries(GL_TIMESTAMP, ProfilingNode::num_queries, node.begin_queries);
            glCreateQueries(GL_TIMESTAMP, ProfilingNode::num_queries, node.end_queries);

            uint32_t query_idx = static_cast<uint32_t>(queries.size());
            name_to_query[name] = query_idx;
            queries.push_back(node);
            names.push_back(name);
            lru.push_back(frame_index);
            it = name_to_query.find(name);
        }

        uint32_t query_idx = it->second;
        ProfilingNode& node = queries[query_idx];
        lru[query_idx] = frame_index;
        uint32_t query_id = node.begin_queries[node.current_query_idx];
        glQueryCounter(query_id, GL_TIMESTAMP);
        node.depth = curr_nested_depth;
        curr_nested_depth++;
    }

    void end_gpu(const std::string& name)
    {
        auto it = name_to_query.find(name);
        if (it == name_to_query.end())
            return;

        uint32_t query_idx = it->second;
        ProfilingNode& node = queries[query_idx];
        uint32_t query_id = node.end_queries[node.current_query_idx];
        glQueryCounter(query_id, GL_TIMESTAMP);
        curr_nested_depth--;
    }

    uint32_t frame_index = 0;

private:
    struct ProfilingNode
    {
        inline static constexpr uint32_t num_queries = 3;
        uint32_t begin_queries[num_queries];
        uint32_t end_queries[num_queries];
        uint32_t current_query_idx = 0;
        uint32_t depth = 0;
    };

    uint32_t curr_nested_depth = 0;
    std::unordered_map<std::string, uint32_t> name_to_query;
    std::vector<ProfilingNode> queries;
    std::vector<std::string> names;
    std::vector<uint32_t> lru;
};