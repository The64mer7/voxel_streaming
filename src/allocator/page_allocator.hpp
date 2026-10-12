#pragma once

#include <cstdint>
#include <stack>
#include <vector>

class PageAllocator
{
public:
    void set_page_count(uint32_t max_num_pages) { m_max_num_pages = max_num_pages; }
    uint32_t allocate_page()
    {
        if (!m_freelist.empty())
        {
            uint32_t page = m_freelist.top();
            m_freelist.pop();
            return page;
        }

        if (m_next_page < m_max_num_pages)
        {
            return m_next_page++;
        }

        return UINT32_MAX;
    }
    void free_page(uint32_t page) { m_freelist.push(page); }

private:
    uint32_t m_next_page = 0;
    uint32_t m_max_num_pages = 0;
    std::stack<uint32_t, std::vector<uint32_t>> m_freelist;
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
        m_allocator.set_page_count(num_pages);
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
