#ifndef LVDT_MODULE_HPP
#define LVDT_MODULE_HPP

#include "callback_list.hpp"
#include "adc_service.hpp"
#include "process.hpp"
#include "dac_service.hpp"
#include "generic.h"

class LvdtSensorModule : public Process {
public:
    using MeasurementCallback = void (*)(void* context, float positionMm, float magA, float magB);

    LvdtSensorModule(SineGeneratorChannel* excitation, 
        IQDemodulatorChannel* secondaryA, 
        IQDemodulatorChannel* secondaryB,
        float strokeMm);

    // Where the sensor's electrical centre sits above the Z origin. Starts at
    // the configured estimate and is replaced by the measured value once the
    // axis has been calibrated against its top stop.
    void setPositionOffsetMm(float offsetMm);
    float getPositionOffsetMm() const;


    bool addMeasurementListenerCallback(void* callbackContext, MeasurementCallback callback);
    bool removeMeasurementListenerCallback(void* callbackContext, MeasurementCallback callback);
    bool startMeasurement();
    void stopMeasurement();
    bool isMeasuring() const;

private:
    enum class MeasurementState : uint8_t { Idle, Measuring };

    void onStart() override;
    void onStop() override;

    enum class Secondary {
        A,
        B
    };

    static void onMeasurementA(void* context, float re, float im);
    static void onMeasurementB(void* context, float re, float im);

    void handleMeasurement(Secondary secondary, float re, float im);

    bool readFreshPairIfAvailable(float& magA, float& magB);
    bool tryComputePositionMm(float magA, float magB, float& positionMm) const;
    void fireCallback(float positionMm, float magA, float magB) const;

    SineGeneratorChannel* m_excitation;
    IQDemodulatorChannel* m_secondaryA;
    IQDemodulatorChannel* m_secondaryB;

    ListenerList<float, float, float> m_callbacks;

    float m_strokeMm;
    float m_positionOffsetMm;

    volatile float m_magA;
    volatile float m_magB;
    volatile bool m_updatedA;
    volatile bool m_updatedB;
    MeasurementState m_measurementState;
};

#endif // LVDT_MODULE_HPP
