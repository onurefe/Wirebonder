#ifndef TRANSDUCER_ANALYZER_HPP
#define TRANSDUCER_ANALYZER_HPP

#include <cstdint>

#include "complex.h"

// Fits a Butterworth-Van Dyke equivalent circuit to a set of complex
// impedance samples taken on a uniform frequency grid:
//
//   Y(jw) = jwC0 + 1 / (R1 + j(wL1 - 1/(wC1)))
//
// The fit is fully closed-form. With C0 known, the motional impedance
// Zm = 1/(Y - jwC0) satisfies Re(Zm) = R1 and w*Im(Zm) = w^2*L1 - 1/C1,
// which is a straight line in w^2 — ordinary linear regression. C0 itself
// is refined by alternating least squares over a few iterations.
//
// The fitted model also determines the plant the PLL frequency controller
// sees, so its PID tuning can be derived here (see frequencyPidTuning).
class TransducerAnalyzer {
public:
    struct Parameters {
        float r1;                 // motional resistance (ohm)
        float l1;                 // motional inductance (H)
        float c1;                 // motional capacitance (F)
        float c0;                 // static capacitance (F)
        float seriesResonance;    // fs = 1/(2*pi*sqrt(L1*C1)) (Hz)
        float parallelResonance;  // fp = fs*sqrt(1 + C1/C0) (Hz)
        float qFactor;            // Q = ws*L1/R1
    };

    struct FrequencyPidTuning {
        float gain;          // Kp (Hz of correction per rad of phase error)
        float integralTc;    // Ti (s)
        float derivativeTc;  // always 0 — derivative action only amplifies
                             // demodulation noise on a dead-time plant
    };

    // Returns false when the data cannot support a physical fit
    // (too few points, degenerate regression, or non-positive parameters).
    static bool fit(const complexf *impedances,
                    uint8_t count,
                    float startFrequency,
                    float frequencyStep,
                    Parameters &out);

    static complexf admittance(const Parameters &params, float frequency);

    // PI tuning for the PLL frequency loop by loop shaping. The plant is a
    // static gain K = 2Q/fs (phase slope at series resonance, rad/Hz) in
    // series with a pure dead time (correction queue + demodulation window
    // group delay). Crossover is placed at wc = crossoverFraction/deadTime;
    // gain and integral time then follow in closed form from the
    // unit-magnitude and phase-margin conditions at wc. Returns false when
    // the requested margin is infeasible at that crossover.
    static bool frequencyPidTuning(const Parameters &params,
                                   float deadTime,
                                   float phaseMargin,
                                   float crossoverFraction,
                                   FrequencyPidTuning &out);
};

#endif /* TRANSDUCER_ANALYZER_HPP */
