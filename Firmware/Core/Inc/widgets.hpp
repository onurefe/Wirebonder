#pragma once

#include <cstddef>
#include <cstdint>

class Page;

// Rectangular widget placed on a Page. Position and width live in the
// widget itself; the page composes widgets row by row into 20-column lines.
// An widget with a click callback is triggered when the pointer sits on its
// row and enter is pressed.
class Widget {
public:
    using ClickCallback = void (*)(void *ctx);

    Widget(uint8_t row, uint8_t column, uint8_t width);

    uint8_t row() const { return m_row; }
    uint8_t column() const { return m_column; }
    uint8_t width() const { return m_width; }

    void setClickCallback(void *ctx, ClickCallback callback);
    bool isClickable() const { return m_clickCallback != nullptr; }
    void click();

    // Hidden widgets are skipped when the page composes its rows.
    void setVisible(bool visible);
    bool isVisible() const { return m_visible; }

    // Writes the widget content as a NUL-terminated string of at most
    // width() characters into text.
    virtual void render(char *text, size_t textSize) const = 0;

protected:
    ~Widget() = default;

    // Propagates a content change to the owning page so the renderer knows
    // the frame is stale.
    void markDirty();

private:
    friend class Page;

    uint8_t m_row;
    uint8_t m_column;
    uint8_t m_width;
    bool m_visible;
    Page *m_page;
    ClickCallback m_clickCallback;
    void *m_clickCallbackCtx;
};

class TextWidget : public Widget {
public:
    static constexpr size_t kMaxTextLength = 20U;

    TextWidget(uint8_t row,
                  uint8_t column,
                  uint8_t width,
                  const char *text = "");

    void setText(const char *text);
    const char *text() const { return m_text; }

    void render(char *text, size_t textSize) const override;

private:
    char m_text[kMaxTextLength + 1U];
};

// Renders a fixed-point decimal right-aligned in its field. Avoids %f so it
// works with the newlib-nano printf used by the firmware.
class FloatWidget : public Widget {
public:
    FloatWidget(uint8_t row,
                   uint8_t column,
                   uint8_t width,
                   uint8_t decimals = 2U,
                   float value = 0.0f);

    void setValue(float value);
    float value() const { return m_value; }

    void render(char *text, size_t textSize) const override;

private:
    float m_value;
    uint8_t m_decimals;
};

class IntegerWidget : public Widget {
public:
    IntegerWidget(uint8_t row,
                     uint8_t column,
                     uint8_t width,
                     int32_t value = 0);

    void setValue(int32_t value);
    int32_t value() const { return m_value; }

    void render(char *text, size_t textSize) const override;

private:
    int32_t m_value;
};

// A logical screen of widgets. Always 20 columns wide, but may have more
// rows than the 4 the LCD shows; the DisplayRenderer scrolls a 4-row window
// over it following the pointer.
class Page {
public:
    static constexpr uint8_t kColumns = 20U;
    static constexpr size_t kMaxWidgets = 24U;

    explicit Page(uint8_t rowCount);

    uint8_t rowCount() const { return m_rowCount; }

    // Registers an widget; the page does not take ownership. Returns false
    // when the widget table is full.
    bool addWidget(Widget *widget);

    // Composes every widget on the given row into a 20-character,
    // NUL-terminated line (line must hold kColumns + 1 bytes).
    void renderRow(uint8_t row, char *line) const;

    // First clickable widget on the given row, or nullptr.
    Widget *clickableWidgetAt(uint8_t row) const;
    bool hasClickableWidgets() const;

    bool isDirty() const { return m_dirty; }
    void clearDirty() { m_dirty = false; }
    void markDirty() { m_dirty = true; }

private:
    Widget *m_widgets[kMaxWidgets];
    size_t m_widgetCount;
    uint8_t m_rowCount;
    bool m_dirty;
};
