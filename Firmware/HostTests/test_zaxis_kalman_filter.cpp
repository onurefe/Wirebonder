#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "zaxis_kalman_filter.hpp"

// Same values as the configuration.h placeholders, spelled out so the tests do
// not move when those are tuned on the machine.
static ZAxisKalmanFilter::Config cfg(float positionNoise = 0.002f) {
    ZAxisKalmanFilter::Config config{};
    config.motorGain         = 1.0f;
    config.motorTimeConstant = 0.020f;
    config.dt                = 0.001f;
    config.positionNoise     = positionNoise;
    config.accelerationNoise = 200.0f;
    config.disturbanceDrift  = 2.0f;
    return config;
}

// The plant the filter assumes, discretised the same way, so a noise-free run
// isolates the estimator from model mismatch.
struct Plant {
    float km;
    float tau;
    float dt;
    float position = 0.0f;
    float velocity = 0.0f;

    void step(float voltage) {
        const float a = expf(-dt / tau);
        const float g = tau * (1.0f - a);
        position += g * velocity + km * (dt - g) * voltage;
        velocity = a * velocity + km * (1.0f - a) * voltage;
    }
};

TEST(ZAxisKalmanFilter, RejectsNonPhysicalConfig) {
    ZAxisKalmanFilter::Config config = cfg();
    config.motorTimeConstant = 0.0f;
    ZAxisKalmanFilter noTimeConstant(config);
    EXPECT_FALSE(noTimeConstant.initialize());
    EXPECT_FALSE(noTimeConstant.isInitialized());

    ZAxisKalmanFilter noMeasurementNoise(cfg(0.0f));
    EXPECT_FALSE(noMeasurementNoise.initialize());

    config = cfg();
    config.motorGain = -1.0f;
    ZAxisKalmanFilter negativeGain(config);
    EXPECT_FALSE(negativeGain.initialize());
}

TEST(ZAxisKalmanFilter, SteadyStateGainMatchesReferenceSolution) {
    // Reference from an independent numpy iteration of the same Riccati
    // recursion (model and noise as in cfg()).
    ZAxisKalmanFilter filter(cfg());
    ASSERT_TRUE(filter.initialize());
    EXPECT_NEAR(filter.getGain(ZAxisKalmanFilter::POSITION), 0.341213f, 1e-4f);
    EXPECT_NEAR(filter.getGain(ZAxisKalmanFilter::VELOCITY), 69.9414f, 1e-2f);
    EXPECT_NEAR(filter.getGain(ZAxisKalmanFilter::DISTURBANCE), 25.6668f, 1e-2f);
}

TEST(ZAxisKalmanFilter, ResetStartsAtRestWithoutDisturbance) {
    ZAxisKalmanFilter filter(cfg());
    ASSERT_TRUE(filter.initialize());
    filter.reset(4.5f);
    EXPECT_FLOAT_EQ(filter.getPosition(), 4.5f);
    EXPECT_FLOAT_EQ(filter.getVelocity(), 0.0f);
    EXPECT_FLOAT_EQ(filter.getDisturbance(), 0.0f);
}

TEST(ZAxisKalmanFilter, TracksVelocityOfModelledPlant) {
    ZAxisKalmanFilter filter(cfg());
    ASSERT_TRUE(filter.initialize());
    Plant plant{1.0f, 0.020f, 0.001f};
    filter.reset(plant.position);

    // A 3 V step settles at 3 mm/s; give it ten time constants.
    for (int k = 0; k < 200; ++k) {
        filter.correct(plant.position);
        filter.predict(3.0f);
        plant.step(3.0f);
    }
    filter.correct(plant.position);

    EXPECT_NEAR(plant.velocity, 3.0f, 1e-3f);
    EXPECT_NEAR(filter.getVelocity(), plant.velocity, 0.01f);
    EXPECT_NEAR(filter.getPosition(), plant.position, 1e-4f);
    EXPECT_NEAR(filter.getDisturbance(), 0.0f, 0.01f);
}

TEST(ZAxisKalmanFilter, EstimatesUnmodelledLoadAsDisturbance) {
    // The plant sees 1 V less than the filter is told was applied -- a
    // friction or gravity load. The filter should attribute it to the
    // disturbance state and still get the velocity right.
    ZAxisKalmanFilter filter(cfg());
    ASSERT_TRUE(filter.initialize());
    Plant plant{1.0f, 0.020f, 0.001f};
    filter.reset(plant.position);

    for (int k = 0; k < 2000; ++k) {
        filter.correct(plant.position);
        filter.predict(4.0f);
        plant.step(4.0f - 1.0f);
    }
    filter.correct(plant.position);

    EXPECT_NEAR(plant.velocity, 3.0f, 1e-3f);
    EXPECT_NEAR(filter.getVelocity(), plant.velocity, 0.01f);
    EXPECT_NEAR(filter.getDisturbance(), -1.0f, 0.01f);
}

TEST(ZAxisKalmanFilter, VelocityNoiseIsFarBelowNaiveDifferentiation) {
    // 2 um LVDT noise. Differencing raw positions at 1 kHz would give
    // sqrt(2) * 2 um / 1 ms ~= 2.8 mm/s RMS of velocity noise.
    ZAxisKalmanFilter filter(cfg());
    ASSERT_TRUE(filter.initialize());
    Plant plant{1.0f, 0.020f, 0.001f};
    std::mt19937 rng(1234U);
    std::normal_distribution<float> noise(0.0f, 0.002f);
    filter.reset(plant.position);

    double squaredError = 0.0;
    int counted = 0;
    for (int k = 0; k < 4000; ++k) {
        const float voltage = (k % 1000 < 500) ? 3.0f : -3.0f;
        filter.correct(plant.position + noise(rng));
        if (k >= 100) {
            const double error = filter.getVelocity() - plant.velocity;
            squaredError += error * error;
            ++counted;
        }
        filter.predict(voltage);
        plant.step(voltage);
    }

    const double rms = sqrt(squaredError / counted);
    EXPECT_LT(rms, 0.4);
}
