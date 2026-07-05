#pragma once

#include "bonder_config.hpp"
#include "lcd_module.hpp"
#include "control_panel_service.hpp"
#include "generic.h"
#include <cstdint>

// ---------------------------------------------------------------------------
// UiModule
//
// Non-blocking parameter editor for BonderConfig displayed on a 20×4 LCD.
//
// Dependencies: LcdModule, ButtonChannels from the left panel.
// No dependency on BonderModule — config changes are delivered through a
// registered callback so the application decides how to apply them.
//
// LCD layout (20 columns × 4 rows):
//
//   Heights              2/6   ← group name + index
//   >Search Height             ← cursor + parameter name
//     1.500 mm                 ← current value + unit
//   step:0.100                 ← step size / timed status
//
// Navigation:
//   left/right   switch parameter group
//   up/down      scroll parameter within group
//   plus/minus   adjust selected parameter ± step
//   step_±       cycle step size  0.01→0.05→0.1→0.5→1.0
//   tail_±       directly nudge tailDisplacement (+ jump cursor)
//   loop_±       directly nudge loopHeight       (+ jump cursor)
//   search_±     directly nudge searchHeight     (+ jump cursor)
//   save/load    fire registered persistence callbacks
// ---------------------------------------------------------------------------
class UiModule {
public:
    using ConfigChangedCallback = void (*)(void *ctx, const BonderConfig&);
    using SaveCallback          = void (*)(void *ctx, const BonderConfig&);
    using LoadCallback          = void (*)(void *ctx, BonderConfig&);

    struct Buttons {
        ButtonChannel *up;
        ButtonChannel *down;
        ButtonChannel *left;
        ButtonChannel *right;
        ButtonChannel *plus;
        ButtonChannel *minus;
        ButtonChannel *save;
        ButtonChannel *load;
        ButtonChannel *tailPlus;
        ButtonChannel *tailMinus;
        ButtonChannel *loopPlus;
        ButtonChannel *loopMinus;
        ButtonChannel *searchPlus;
        ButtonChannel *searchMinus;
        ButtonChannel *stepPlus;
        ButtonChannel *stepMinus;
        ButtonChannel *factoryReset;
        ButtonChannel *enter;
    };

    UiModule(LcdModule *lcd, const Buttons& btns);

    // Fired on every parameter value change.
    void setConfigChangedListenerCallback(void *ctx, ConfigChangedCallback cb);

    // Persistence hooks — flash read/write is the application's responsibility.
    void setPersistenceControllerCallbacks(void *ctx, SaveCallback save, LoadCallback load);

    // Register all button callbacks. Call before entering the main loop.
    void startService();
    void stopService();

    // Must be called every main-loop iteration.
    void execute();
    bool isOperating() const { return m_state == ServiceState::OPERATING; }

private:
    // ---- Parameter descriptor table ----
    struct ParamDescriptor {
        const char *name;       // ≤14 chars — shown on row 1 after ">"
        const char *unit;       // display unit: "mm", "A", "s", "kHz", ""
        uint16_t    offset;     // offsetof(BonderConfig, field)
        float       scale;      // stored → display  (1.0 or 0.001 for Hz→kHz)
        float       minDisplay;
        float       maxDisplay;
        uint8_t     groupIndex;
        bool        isInteger;  // field is uint16_t, not float
    };

    static constexpr uint8_t kNumGroups  = 6U;
    static constexpr uint8_t kParamCount = 23U;
    static const ParamDescriptor kParams[kParamCount];
    static const char *kGroupNames[kNumGroups];

    // ---- Step size cycling ----
    static constexpr float   kStepSizes[]  = { 0.01f, 0.05f, 0.1f, 0.5f, 1.0f };
    static constexpr uint8_t kNumStepSizes = 5U;

    // ---- Internal helpers ----
    uint8_t groupParamCount(uint8_t group) const;
    const ParamDescriptor *currentDescriptor() const;
    const ParamDescriptor *descriptorByOffset(uint16_t offset) const;
    void jumpToParam(uint16_t offset);

    void navigate(int paramDelta, int groupDelta);
    void adjustCurrent(float sign);
    void adjustHotkey(uint16_t offset, float sign);
    void adjustParam       (uint16_t offset, float delta);
    void adjustIntegerParam(const ParamDescriptor *d, float delta);
    void adjustFloatParam  (const ParamDescriptor *d, float delta);

    void doSave();
    void doLoad();
    void setStatus(const char *msg);

    void refreshDisplay();
    void buildRow(char *line, uint8_t row) const;
    void buildGroupRow    (char *line) const;
    void buildParamNameRow(char *line) const;
    void buildValueRow    (char *line) const;
    void buildStatusRow   (char *line) const;
    void buildConfirmRow  (char *line, uint8_t row) const;

    void doFactoryReset();

    // ---- Static button callbacks ----
    static void onUp_(void *ctx);
    static void onDown_(void *ctx);
    static void onLeft_(void *ctx);
    static void onRight_(void *ctx);
    static void onPlus_(void *ctx);
    static void onMinus_(void *ctx);
    static void onSave_(void *ctx);
    static void onLoad_(void *ctx);
    static void onTailPlus_(void *ctx);
    static void onTailMinus_(void *ctx);
    static void onLoopPlus_(void *ctx);
    static void onLoopMinus_(void *ctx);
    static void onSearchPlus_(void *ctx);
    static void onSearchMinus_(void *ctx);
    static void onStepPlus_     (void *ctx);
    static void onStepMinus_    (void *ctx);
    static void onFactoryReset_ (void *ctx);
    static void onEnter_        (void *ctx);

    // ---- Mode ----
    enum class Mode { NORMAL, CONFIRMING_FACTORY_RESET };

    // ---- State ----
    LcdModule *m_lcd;
    Buttons    m_btns;
    BonderConfig m_config;

    Mode     m_mode;
    uint8_t  m_groupIndex;
    uint8_t  m_paramIndex;
    uint8_t  m_stepIndex;

    bool     m_displayOutOfDate;
    uint32_t m_statusClearTick;
    char     m_statusMsg[21];

    ConfigChangedCallback m_configChangedCallback;
    void                 *m_configChangedCtx;
    SaveCallback          m_saveCallback;
    LoadCallback          m_loadCallback;
    void                 *m_persistenceCtx;
    ServiceState          m_state;
};
