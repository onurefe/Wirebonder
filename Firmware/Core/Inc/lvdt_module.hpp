#ifndef LVDT_MODULE_HPP
#define LVDT_MODULE_HPP

#include "adc_service.hpp"
#include "dac_service.hpp"
#include "generic.h"

class LvdtSensorModule {
public:
    using MeasurementCallback = void (*)(void* context, float positionMm);

    LvdtSensorModule(SineGeneratorChannel* excitation, 
        IQDemodulatorChannel* secondaryA, 
        IQDemodulatorChannel* secondaryB,
        float strokeMm);

    void start();
    void addMeasurementListenerCallback(void* callbackContext, MeasurementCallback callback);
    void stop();
    bool isOperating() const { return m_state == ServiceState::OPERATING; }

private:
    enum class Secondary {
        A,
        B
    };

    static void onMeasurementA(void* context, float re, float im);
    static void onMeasurementB(void* context, float re, float im);

    void handleMeasurement(Secondary secondary, float re, float im);

    bool readFreshPairIfAvailable(float& magA, float& magB);
    bool tryComputePositionMm(float magA, float magB, float& positionMm) const;
    void fireCallback(float positionMm) const;

    SineGeneratorChannel* m_excitation;
    IQDemodulatorChannel* m_secondaryA;
    IQDemodulatorChannel* m_secondaryB;

    MeasurementCallback m_callback;
    void* m_callbackContext;

    float m_strokeMm;

    volatile float m_magA;
    volatile float m_magB;
    volatile bool m_updatedA;
    volatile bool m_updatedB;
    ServiceState m_state;
};

#endif // LVDT_MODULE_HPP
