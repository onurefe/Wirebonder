#ifndef ZAXIS_KALMAN_FILTER_HPP
#define ZAXIS_KALMAN_FILTER_HPP

#include <cstdint>

// Estimates the Z carriage's position, velocity and an input-referred
// disturbance from the LVDT alone, using a first-order DC motor model driven
// by the voltage actually applied:
//
//   velocity'    = (motorGain * (voltage + disturbance) - velocity) / motorTimeConstant
//   position'    = velocity
//   disturbance' = white noise
//
// Units follow the Z convention: mm, mm/s and volts, up positive. The
// disturbance is whatever extra voltage would explain the motion the model
// did not predict -- friction, the head's weight, model error -- so it reads
// in volts and can be cancelled by feeding it back with the opposite sign.
//
// The gain is the steady-state Kalman gain, solved once by initialize()
// (Riccati recursion in double precision, so call it after the clocks are
// up rather than from a static constructor). Each tick is then a fixed
// handful of float multiply-adds.
class ZAxisKalmanFilter {
public:
    static constexpr uint8_t kStateCount = 3U;

    enum State : uint8_t {
        POSITION = 0,
        VELOCITY = 1,
        DISTURBANCE = 2
    };

    struct Config {
        float motorGain;         // steady-state velocity per volt (mm/s/V)
        float motorTimeConstant; // mechanical time constant (s)
        float dt;                // sample period (s)
        float positionNoise;     // LVDT noise, 1 sigma (mm)
        float accelerationNoise; // unmodelled acceleration, 1 sigma (mm/s^2)
        float disturbanceDrift;  // disturbance random walk (V/sqrt(s))
    };

    explicit ZAxisKalmanFilter(const Config &config);

    // Discretises the model and solves for the steady-state gain. False if
    // the configuration is not physical or the recursion does not converge,
    // in which case the filter must not be used.
    bool initialize();
    bool isInitialized() const;

    // Starts from a known position at rest with no disturbance.
    void reset(float position);
    // Measurement update with this tick's LVDT position.
    void correct(float measuredPosition);
    // Time update with the voltage applied until the next measurement.
    void predict(float appliedVoltage);

    float getPosition() const;
    float getVelocity() const;
    float getDisturbance() const;
    float getGain(State state) const;

private:
    Config m_config;
    bool m_initialized;

    float m_transition[kStateCount][kStateCount];
    float m_input[kStateCount];
    float m_gain[kStateCount];
    float m_state[kStateCount];
};

#endif // ZAXIS_KALMAN_FILTER_HPP
