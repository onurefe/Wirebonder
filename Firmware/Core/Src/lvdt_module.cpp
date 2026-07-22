#include "lvdt_module.hpp"
#include <cmath>

#include "generic.h"

LvdtSensorModule::LvdtSensorModule(SineGeneratorChannel* excitation, IQDemodulatorChannel* secondaryA, IQDemodulatorChannel* secondaryB, float strokeMm)
    : m_excitation(excitation)
    , m_secondaryA(secondaryA)
    , m_secondaryB(secondaryB)
    , m_callback(nullptr)
    , m_callbackContext(nullptr)
    , m_strokeMm(strokeMm)
    , m_magA(0.0f)
    , m_magB(0.0f)
    , m_updatedA(false)
    , m_updatedB(false)
    , m_measurementState(MeasurementState::Idle)
{
}

void LvdtSensorModule::addMeasurementListenerCallback(void* callbackContext, MeasurementCallback callback) 
{   
    m_callback = callback;
    m_callbackContext = callbackContext;   
}

void LvdtSensorModule::onStart()
{
    if (m_excitation == nullptr || m_secondaryA == nullptr ||
        m_secondaryB == nullptr) {
        setProcessError();
        return;
    }

    if (!m_secondaryA->addMeasurementListenerCallback(
            this, &LvdtSensorModule::onMeasurementA) ||
        !m_secondaryB->addMeasurementListenerCallback(
            this, &LvdtSensorModule::onMeasurementB)) {
        setProcessError();
    }
}

void LvdtSensorModule::onStop()
{
    stopMeasurement();
}

bool LvdtSensorModule::startMeasurement()
{
    if (!isOperating() || m_measurementState != MeasurementState::Idle) {
        return false;
    }

    m_updatedA = false;
    m_updatedB = false;
    m_measurementState = MeasurementState::Measuring;
    m_excitation->start(
        LVDT_MODULE_EXCITATION_AMPLITUDE,
        LVDT_MODULE_EXCITATION_AVERAGE,
        static_cast<float>(LVDT_MODULE_DRIVING_FREQUENCY) /
            static_cast<float>(DAC2_SAMPLING_FREQ));
    return true;
}

void LvdtSensorModule::stopMeasurement()
{
    if (m_measurementState == MeasurementState::Idle) return;
    m_excitation->stop();
    m_updatedA = false;
    m_updatedB = false;
    m_measurementState = MeasurementState::Idle;
}

bool LvdtSensorModule::isMeasuring() const
{
    return m_measurementState == MeasurementState::Measuring;
}

void LvdtSensorModule::onMeasurementA(void* context, float re, float im) {
    auto* sensor = static_cast<LvdtSensorModule*>(context);

    if (sensor != nullptr) {
        sensor->handleMeasurement(Secondary::A, re, im);
    }
}

void LvdtSensorModule::onMeasurementB(void* context, float re, float im) {
    auto* sensor = static_cast<LvdtSensorModule*>(context);

    if (sensor != nullptr) {
        sensor->handleMeasurement(Secondary::B, re, im);
    }
}

void LvdtSensorModule::handleMeasurement(Secondary secondary, float re, float im) {
    if (!isOperating() || !isMeasuring()) {
        return;
    }

    float magA = 0.0f;
    float magB = 0.0f;
    float magnitude = sqrtf(re * re + im * im); 

    bool pairReady = false;

    {
        InterruptLock lock;

        if (secondary == Secondary::A) {
            m_magA = magnitude;
            m_updatedA = true;
        } else {
            m_magB = magnitude;
            m_updatedB = true;
        }

        pairReady = readFreshPairIfAvailable(magA, magB);
    }

    if (!pairReady) {
        return;
    }

    float positionMm = 0.0f;

    if (tryComputePositionMm(magA, magB, positionMm)) {
        fireCallback(positionMm, magA, magB);
    }
}

bool LvdtSensorModule::readFreshPairIfAvailable(float& magA, float& magB) {
    if (!m_updatedA || !m_updatedB) {
        return false;
    }

    magA = m_magA;
    magB = m_magB;

    m_updatedA = false;
    m_updatedB = false;

    return true;
}

bool LvdtSensorModule::tryComputePositionMm(float magA, float magB, float& positionMm) const {
    const float sum = magA + magB;

    if (sum < LVDT_MODULE_MIN_TOTAL_MAGNITUDE) {
        return false;
    }

    const float normalizedPosition = (magA - magB) / sum;

    positionMm = normalizedPosition * (m_strokeMm * 0.5f);
    return true;
}

void LvdtSensorModule::fireCallback(float positionMm, float magA, float magB) const {
    if (m_callback != nullptr) {
        m_callback(m_callbackContext, positionMm, magA, magB);
    }
}
