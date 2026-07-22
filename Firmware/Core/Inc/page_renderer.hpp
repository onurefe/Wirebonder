#pragma once

#include "lcd_controller_module.hpp"
#include "widgets.hpp"
#include <cstddef>
#include <cstdint>

// Renders the active Page onto the 20x4 LCD and refreshes it whenever the
// page reports dirty widgets. Policy lives above: higher-level code decides
// which registered page is active, where the visible window starts, and where
// the pointer sits — this class only draws.
class PageRenderer {
public:
    static constexpr uint8_t kVisibleRows = 4U;
    static constexpr size_t kMaxPages = 12U;
    // Pointer row value meaning "no pointer drawn".
    static constexpr uint8_t kNoPointer = 0xFFU;

    explicit PageRenderer(LcdControllerModule *lcdController);

    void start();
    void stop();
    void execute();

    // Registers a page; the renderer does not take ownership. Returns false
    // when the page table is full.
    bool registerPage(Page *page);

    // Selects which registered page is drawn; nullptr blanks the screen.
    // Unregistered pages are ignored.
    void setActivePage(Page *page);
    Page *activePage() const { return m_activePage; }

    // First page row shown on the top LCD line. Clamped so the window stays
    // inside the page.
    void setWindowStart(uint8_t row);
    uint8_t windowStart() const { return m_windowStart; }

    // Row (in page coordinates) marked with '*' in column 0, or kNoPointer.
    void setPointerRow(uint8_t row);
    uint8_t pointerRow() const { return m_pointerRow; }

    // Forces a full redraw on the next execute().
    void requestRedraw();

private:
    bool isRegistered(const Page *page) const;
    void renderFrame();

    LcdControllerModule *m_lcdController;
    Page *m_pages[kMaxPages];
    size_t m_pageCount;
    Page *m_activePage;
    uint8_t m_windowStart;
    uint8_t m_pointerRow;
    bool m_dirty;
    bool m_running;
};
