#pragma once

#include <cstdint>
#include <stack>
#include <vector>

class PageAllocator
{
public:
    void set_page_count(uint32_t max_num_pages, uint32_t num_freelists)
    {
        m_max_num_pages = max_num_pages;
        m_freelists.resize(num_freelists);
    }
    uint32_t allocate_page()
    {
        FreelistStack* freelist = find_nonempty_freelist();
        if (freelist)
        {
            uint32_t page = freelist->top();
            freelist->pop();
            return page;
        }

        if (m_next_page < m_max_num_pages)
        {
            return m_next_page++;
        }

        return UINT32_MAX;
    }
    void free_page(uint32_t page) { get_freelist(page)->push(page); }
    uint32_t free_pages_count() { return 0; }

private:
    using FreelistStack = std::stack<uint32_t, std::vector<uint32_t>>;

    FreelistStack* find_nonempty_freelist()
    {
        for (size_t i = 0; i < m_freelists.size(); i++)
        {
            if (m_freelists[i].size() > 0)
            {
                return &m_freelists[i];
            }
        }

        return nullptr;
    }
    FreelistStack* get_freelist(uint32_t page)
    {
        uint32_t chunk_size = (m_max_num_pages + m_freelists.size() - 1) / m_freelists.size();
        size_t index = page / chunk_size;
        if (index >= m_freelists.size())
        {
            index = m_freelists.size() - 1;
        }

        return &m_freelists[index];
    }

    uint32_t m_next_page = 0;
    uint32_t m_max_num_pages = 0;

    std::vector<FreelistStack> m_freelists;
    std::vector<uint64_t> m_avail_freelists_bitmask;
};

template <typename T, size_t PageSize>
class PagedPool
{
public:
    struct Page
    {
        T data[PageSize];
    };
    void create(size_t num_pages)
    {
        m_allocator.set_page_count(num_pages, 1);
        m_pages.resize(num_pages);
    }

    Page* get_page(uint32_t handle) { return &m_pages[handle]; }

    uint32_t allocate_page() { return m_allocator.allocate_page(); }

    void free_page(uint32_t page) { m_allocator.free_page(page); }

    Page* get_pages_buffer() { return m_pages.data(); }

    size_t get_pages_count() { return m_pages.size(); }

private:
    PageAllocator m_allocator;
    std::vector<Page> m_pages;
};
