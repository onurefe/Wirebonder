#include "page_renderer.hpp"
#include <cstring>

PageRenderer::PageRenderer(LcdControllerModule *lcdController)
    : m_lcdController(lcdController)
    , m_pages{}
    , m_pageCount(0U)
    , m_activePage(nullptr)
    , m_windowStart(0U)
    , m_pointerRow(kNoPointer)
    , m_dirty(false)
    , m_running(false)
{
}

void PageRenderer::start()
{
    m_dirty = true;
    m_running = true;
}

void PageRenderer::stop()
{
    m_running = false;
    m_dirty = false;
}

bool PageRenderer::registerPage(Page *page)
{
    if (page == nullptr || m_pageCount >= kMaxPages) return false;
    if (isRegistered(page)) return true;
    m_pages[m_pageCount++] = page;
    return true;
}

bool PageRenderer::isRegistered(const Page *page) const
{
    for (size_t i = 0U; i < m_pageCount; ++i) {
        if (m_pages[i] == page) return true;
    }
    return false;
}

void PageRenderer::setActivePage(Page *page)
{
    if (page != nullptr && !isRegistered(page)) return;
    if (page == m_activePage) return;
    m_activePage = page;
    m_windowStart = 0U;
    m_pointerRow = kNoPointer;
    m_dirty = true;
}

void PageRenderer::setWindowStart(uint8_t row)
{
    if (m_activePage != nullptr) {
        const uint8_t rowCount = m_activePage->rowCount();
        const uint8_t maxStart = rowCount > kVisibleRows
            ? static_cast<uint8_t>(rowCount - kVisibleRows)
            : 0U;
        if (row > maxStart) row = maxStart;
    }
    if (row == m_windowStart) return;
    m_windowStart = row;
    m_dirty = true;
}

void PageRenderer::setPointerRow(uint8_t row)
{
    if (row == m_pointerRow) return;
    m_pointerRow = row;
    m_dirty = true;
}

void PageRenderer::requestRedraw()
{
    m_dirty = true;
}

void PageRenderer::execute()
{
    if (!m_running) return;
    if (m_activePage != nullptr && m_activePage->isDirty()) {
        m_dirty = true;
    }
    if (m_dirty && m_lcdController != nullptr && m_lcdController->isIdle()) {
        renderFrame();
        m_dirty = false;
    }
}

void PageRenderer::renderFrame()
{
    char line[Page::kColumns + 1U];
    for (uint8_t row = 0U; row < kVisibleRows; ++row) {
        const uint8_t pageRow = static_cast<uint8_t>(m_windowStart + row);
        if (m_activePage != nullptr && pageRow < m_activePage->rowCount()) {
            m_activePage->renderRow(pageRow, line);
        } else {
            std::memset(line, ' ', Page::kColumns);
            line[Page::kColumns] = '\0';
        }
        if (pageRow == m_pointerRow) {
            line[0] = '*';
        }
        m_lcdController->setCursor(0U, row);
        m_lcdController->printString(line);
    }
    if (m_activePage != nullptr) {
        m_activePage->clearDirty();
    }
}
