#include "widgets.hpp"
#include <cstdio>
#include <cstring>

namespace {

// Right-aligns source into a field of fieldWidth characters. Values wider
// than the field are marked instead of silently truncated to wrong digits.
void rightAlign(char *text, size_t textSize, size_t fieldWidth,
                const char *source)
{
    if (fieldWidth >= textSize) fieldWidth = textSize - 1U;
    const size_t sourceLength = strlen(source);
    if (sourceLength > fieldWidth) {
        memset(text, '#', fieldWidth);
        text[fieldWidth] = '\0';
        return;
    }
    const size_t padding = fieldWidth - sourceLength;
    memset(text, ' ', padding);
    memcpy(text + padding, source, sourceLength + 1U);
}

} // namespace

Widget::Widget(uint8_t row, uint8_t column, uint8_t width)
    : m_row(row)
    , m_column(column)
    , m_width(width)
    , m_visible(true)
    , m_page(nullptr)
    , m_clickCallback(nullptr)
    , m_clickCallbackCtx(nullptr)
{
}

void Widget::setClickCallback(void *ctx, ClickCallback callback)
{
    m_clickCallbackCtx = ctx;
    m_clickCallback = callback;
}

void Widget::click()
{
    if (m_clickCallback != nullptr) {
        m_clickCallback(m_clickCallbackCtx);
    }
}

void Widget::setVisible(bool visible)
{
    if (m_visible == visible) return;
    m_visible = visible;
    markDirty();
}

void Widget::markDirty()
{
    if (m_page != nullptr) {
        m_page->markDirty();
    }
}

TextWidget::TextWidget(uint8_t row,
                             uint8_t column,
                             uint8_t width,
                             const char *text)
    : Widget(row, column, width)
    , m_text{}
{
    setText(text);
}

void TextWidget::setText(const char *text)
{
    if (text == nullptr) text = "";
    if (strncmp(m_text, text, kMaxTextLength) == 0) return;
    strncpy(m_text, text, kMaxTextLength);
    m_text[kMaxTextLength] = '\0';
    markDirty();
}

void TextWidget::render(char *text, size_t textSize) const
{
    snprintf(text, textSize, "%-*.*s",
             static_cast<int>(width()), static_cast<int>(width()), m_text);
}

FloatWidget::FloatWidget(uint8_t row,
                               uint8_t column,
                               uint8_t width,
                               uint8_t decimals,
                               float value)
    : Widget(row, column, width)
    , m_value(value)
    , m_decimals(decimals > 3U ? 3U : decimals)
{
}

void FloatWidget::setValue(float value)
{
    if (m_value == value) return;
    m_value = value;
    markDirty();
}

void FloatWidget::render(char *text, size_t textSize) const
{
    static const uint32_t kPow10[4] = {1U, 10U, 100U, 1000U};
    const int decimals = m_decimals & 0x3;
    const uint32_t scale = kPow10[decimals];

    float value = m_value;
    const bool negative = value < 0.0f;
    if (negative) value = -value;
    const uint32_t scaled =
        static_cast<uint32_t>(value * static_cast<float>(scale) + 0.5f);

    char number[16];
    if (decimals == 0) {
        snprintf(number, sizeof(number), "%s%lu",
                 negative ? "-" : "",
                 static_cast<unsigned long>(scaled));
    } else {
        snprintf(number, sizeof(number), "%s%lu.%0*lu",
                 negative ? "-" : "",
                 static_cast<unsigned long>(scaled / scale),
                 decimals,
                 static_cast<unsigned long>(scaled % scale));
    }
    rightAlign(text, textSize, width(), number);
}

IntegerWidget::IntegerWidget(uint8_t row,
                                   uint8_t column,
                                   uint8_t width,
                                   int32_t value)
    : Widget(row, column, width)
    , m_value(value)
{
}

void IntegerWidget::setValue(int32_t value)
{
    if (m_value == value) return;
    m_value = value;
    markDirty();
}

void IntegerWidget::render(char *text, size_t textSize) const
{
    char number[13];
    snprintf(number, sizeof(number), "%ld", static_cast<long>(m_value));
    rightAlign(text, textSize, width(), number);
}

Page::Page(uint8_t rowCount)
    : m_widgets{}
    , m_widgetCount(0U)
    , m_rowCount(rowCount)
    , m_dirty(true)
{
}

bool Page::addWidget(Widget *widget)
{
    if (widget == nullptr || m_widgetCount >= kMaxWidgets) return false;
    widget->m_page = this;
    m_widgets[m_widgetCount++] = widget;
    m_dirty = true;
    return true;
}

void Page::renderRow(uint8_t row, char *line) const
{
    memset(line, ' ', kColumns);
    line[kColumns] = '\0';

    for (size_t i = 0U; i < m_widgetCount; ++i) {
        const Widget *widget = m_widgets[i];
        if (!widget->isVisible() || widget->row() != row ||
            widget->column() >= kColumns) continue;

        char text[kColumns + 1U];
        widget->render(text, sizeof(text));

        size_t span = widget->width();
        if (widget->column() + span > kColumns) {
            span = kColumns - widget->column();
        }
        const size_t textLength = strlen(text);
        if (span > textLength) span = textLength;
        memcpy(line + widget->column(), text, span);
    }
}

Widget *Page::clickableWidgetAt(uint8_t row) const
{
    for (size_t i = 0U; i < m_widgetCount; ++i) {
        if (m_widgets[i]->isVisible() &&
            m_widgets[i]->row() == row && m_widgets[i]->isClickable()) {
            return m_widgets[i];
        }
    }
    return nullptr;
}

bool Page::hasClickableWidgets() const
{
    for (size_t i = 0U; i < m_widgetCount; ++i) {
        if (m_widgets[i]->isVisible() && m_widgets[i]->isClickable()) {
            return true;
        }
    }
    return false;
}
