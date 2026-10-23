#pragma once

#include "allocator/page_allocator.hpp"
#include <mutex>

class NodeAllocator
{
public:
    void create(uint32_t num_pages, uint32_t alloc_size)
    {
        m_allocator.set_page_count(num_pages);
        m_alloc_size = alloc_size;
    }

    uint32_t allocate()
    {
        std::lock_guard lock(m_mtx);
        uint32_t page = m_allocator.allocate_page();
        if (page == UINT32_MAX)
            return UINT32_MAX;
        return page * m_alloc_size;
    }

    void free(uint32_t offset)
    {
        std::lock_guard lock(m_mtx);
        m_allocator.free_page(offset / m_alloc_size);
    }
    uint32_t free_pages_count() { return m_allocator.free_pages_count(); }

private:
    std::mutex m_mtx;
    uint32_t m_alloc_size;
    PageAllocator m_allocator;
};