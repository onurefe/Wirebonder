#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "generic.h"
#include <cmath>

class PidController {
public:
    struct Config {
        float gain;            // Kp
        float integralTc;      // Ti
        float derivativeTc;    // Td
        float dt;              // Sampling time
        float filterTc;        // Filter time constant for Error
        float outputMin;
        float outputMax;
    };

    struct State {
        float measured;
        float errorRaw;
        float errorFiltered;
        float integral;
        float derivative;
        float output;
    };

    // -------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------
    PidController(const Config& config);

    void start(void);
    float execute(float setpoint, float measuredValue);
    void stop();
    
    // Live tuning helper
    void updateConfig(const Config& newConfig);
    void enableBypass();
    void disableBypass();
    bool isBypassEnabled() const;

private:
    void calculateError(float setpoint, float measured);
    void applyFilter();
    void calculateDerivative();
    void updateIntegral();
    float computePidOutput();
    void saturateOutput(float rawOutput);
    void updateHistory();

    enum ControllerState {
        STATE_READY,
        STATE_OPERATING
    };

    Config m_config;
    State m_state;
    ControllerState m_internalState;
    bool m_bypassEnabled;
    float m_filterAlpha;
    float m_prevFilteredError; // Explicit history tracking
};

#endif // PID_CONTROLLER_H
