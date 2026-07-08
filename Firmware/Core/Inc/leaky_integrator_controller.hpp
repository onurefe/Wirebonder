#ifndef LEAKY_INTEGRATOR_CONTROLLER_HPP
#define LEAKY_INTEGRATOR_CONTROLLER_HPP

#include "generic.h"

class LeakyIntegratorController {
public:
    struct Config {
        float Ri;
        float Rf;
        float Cf;
        float outputMin;
        float outputMax;
        float dt;
        float preamplifierGain;
    };

    LeakyIntegratorController(const Config& config);

    void start();
    float execute(float setpoint, float measuredValue);
    void stop();

    void updateConfig(const Config& newConfig);
    void enableBypass();
    void disableBypass();
    bool isBypassEnabled() const;

private:
    enum ControllerState {
        STATE_READY,
        STATE_OPERATING
    };

    float clampOutput(float rawOutput) const;

    Config m_config;
    float m_capacitorVoltage;
    ControllerState m_internalState;
    bool m_bypassEnabled;
};

#endif
