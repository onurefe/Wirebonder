#include "zaxis_kalman_filter.hpp"

#include <cmath>

namespace {

constexpr uint8_t N = ZAxisKalmanFilter::kStateCount;

// The recursion settles in a few hundred steps for any sensible model; the
// cap only bounds a configuration that never will.
constexpr uint32_t kMaxRiccatiIterations = 20000U;
constexpr double kGainTolerance = 1e-9;

}

ZAxisKalmanFilter::ZAxisKalmanFilter(const Config &config)
    : m_config(config)
    , m_initialized(false)
    , m_transition{}
    , m_input{}
    , m_gain{}
    , m_state{}
{
}

bool ZAxisKalmanFilter::initialize()
{
    m_initialized = false;

    const double km = m_config.motorGain;
    const double tau = m_config.motorTimeConstant;
    const double dt = m_config.dt;
    const double r = static_cast<double>(m_config.positionNoise) *
                     static_cast<double>(m_config.positionNoise);

    if (!(km > 0.0) || !(tau > 0.0) || !(dt > 0.0) || !(r > 0.0) ||
        (m_config.accelerationNoise < 0.0f) ||
        (m_config.disturbanceDrift < 0.0f)) {
        return false;
    }

    // Exact zero-order-hold discretisation: the voltage is held for the whole
    // period, the velocity relaxes towards km * voltage with time constant
    // tau, and the position integrates it.
    const double a = exp(-dt / tau);
    const double g = tau * (1.0 - a);

    const double f[N][N] = {
        {1.0, g,   km * (dt - g)},
        {0.0, a,   km * (1.0 - a)},
        {0.0, 0.0, 1.0}
    };
    const double b[N] = {km * (dt - g), km * (1.0 - a), 0.0};

    // Process noise enters as an acceleration on the velocity and a random
    // walk on the disturbance; the position only moves through the velocity.
    const double qv = static_cast<double>(m_config.accelerationNoise) * dt;
    const double qd = static_cast<double>(m_config.disturbanceDrift);
    const double q[N] = {0.0, qv * qv, qd * qd * dt};

    // Prior covariance: the position is about as good as one LVDT reading,
    // the rest is unknown.
    double p[N][N] = {
        {r,   0.0, 0.0},
        {0.0, 1e2, 0.0},
        {0.0, 0.0, 1e2}
    };
    double k[N] = {0.0, 0.0, 0.0};
    bool converged = false;

    for (uint32_t iteration = 0U; iteration < kMaxRiccatiIterations; ++iteration) {
        // The LVDT observes the position alone, so the innovation covariance
        // is a scalar and the gain is a column of the prior.
        const double s = p[0][0] + r;
        double kNext[N];
        for (uint8_t i = 0U; i < N; ++i) {
            kNext[i] = p[i][0] / s;
        }

        double change = 0.0;
        for (uint8_t i = 0U; i < N; ++i) {
            const double relative = fabs(kNext[i] - k[i]) / (1.0 + fabs(kNext[i]));
            change = (relative > change) ? relative : change;
            k[i] = kNext[i];
        }

        // Posterior, then the next prior: F (P - K H P) F' + Q.
        double posterior[N][N];
        for (uint8_t i = 0U; i < N; ++i) {
            for (uint8_t j = 0U; j < N; ++j) {
                posterior[i][j] = p[i][j] - k[i] * p[0][j];
            }
        }

        double fp[N][N];
        for (uint8_t i = 0U; i < N; ++i) {
            for (uint8_t j = 0U; j < N; ++j) {
                double sum = 0.0;
                for (uint8_t m = 0U; m < N; ++m) {
                    sum += f[i][m] * posterior[m][j];
                }
                fp[i][j] = sum;
            }
        }

        for (uint8_t i = 0U; i < N; ++i) {
            for (uint8_t j = i; j < N; ++j) {
                double sum = (i == j) ? q[i] : 0.0;
                for (uint8_t m = 0U; m < N; ++m) {
                    sum += fp[i][m] * f[j][m];
                }
                // Written symmetrically so rounding cannot skew it.
                p[i][j] = sum;
                p[j][i] = sum;
            }
        }

        if ((iteration > 0U) && (change < kGainTolerance)) {
            converged = true;
            break;
        }
    }

    if (!converged) {
        return false;
    }

    for (uint8_t i = 0U; i < N; ++i) {
        for (uint8_t j = 0U; j < N; ++j) {
            m_transition[i][j] = static_cast<float>(f[i][j]);
        }
        m_input[i] = static_cast<float>(b[i]);
        m_gain[i] = static_cast<float>(k[i]);
    }

    m_initialized = true;
    return true;
}

bool ZAxisKalmanFilter::isInitialized() const
{
    return m_initialized;
}

void ZAxisKalmanFilter::reset(float position)
{
    m_state[POSITION] = position;
    m_state[VELOCITY] = 0.0f;
    m_state[DISTURBANCE] = 0.0f;
}

void ZAxisKalmanFilter::correct(float measuredPosition)
{
    const float innovation = measuredPosition - m_state[POSITION];

    for (uint8_t i = 0U; i < N; ++i) {
        m_state[i] += m_gain[i] * innovation;
    }
}

void ZAxisKalmanFilter::predict(float appliedVoltage)
{
    float next[N];

    for (uint8_t i = 0U; i < N; ++i) {
        float sum = m_input[i] * appliedVoltage;
        for (uint8_t j = 0U; j < N; ++j) {
            sum += m_transition[i][j] * m_state[j];
        }
        next[i] = sum;
    }

    for (uint8_t i = 0U; i < N; ++i) {
        m_state[i] = next[i];
    }
}

float ZAxisKalmanFilter::getPosition() const
{
    return m_state[POSITION];
}

float ZAxisKalmanFilter::getVelocity() const
{
    return m_state[VELOCITY];
}

float ZAxisKalmanFilter::getDisturbance() const
{
    return m_state[DISTURBANCE];
}

float ZAxisKalmanFilter::getGain(State state) const
{
    return m_gain[state];
}
