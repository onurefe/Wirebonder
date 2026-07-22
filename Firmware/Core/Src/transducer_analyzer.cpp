#include "transducer_analyzer.hpp"

#include <cmath>

#define TRANSDUCER_ANALYZER_MIN_POINTS        4
#define TRANSDUCER_ANALYZER_C0_ITERATIONS     20

// A bonding transducer has Q in the hundreds; a "resonance" below this is
// the regression extrapolating noise on a flat load (e.g. a test resistor).
#define TRANSDUCER_ANALYZER_MIN_QFACTOR       5.0f

// Motional-branch regression uses only points whose motional admittance
// exceeds this fraction of the peak: far from series resonance |Ym| is the
// small difference of two nearly equal terms (Y and jwC0), so those points
// are noise-dominated — on a high-Q unloaded transducer they can even show
// negative Re(Z) — and poison the least squares.
#define TRANSDUCER_ANALYZER_MIN_REL_YM        0.05f

// ---------------------------------------------------------------------------
// Local-Helpers
// ---------------------------------------------------------------------------

namespace {

bool isPositiveFinite(float x)
{
    return std::isfinite(x) && (x > 0.0f);
}

double omegaAt(float startFrequency, float frequencyStep, uint8_t index)
{
    return 2.0 * M_PI * (startFrequency + index * frequencyStep);
}

// Converts one impedance sample to admittance. Returns false for a
// degenerate (zero) sample.
bool sampleAdmittance(const complexf &z, double &yRe, double &yIm)
{
    double z_abs2 = (double)z.re * z.re + (double)z.im * z.im;

    if (z_abs2 <= 0.0) {
        return false;
    }

    yRe = z.re / z_abs2;
    yIm = -z.im / z_abs2;
    return true;
}

// Initial C0 from a Kasa circle fit on the admittance locus. The motional
// branch traces a circle of center (1/(2R1), 0) and radius 1/(2R1); adding
// jwC0 shifts it up by ~w*C0 (nearly constant over a narrow scan), so the
// fitted center's imaginary part gives C0. Falls back to 0 when the data
// does not describe a circle; the downstream sanity checks reject the fit
// in that case.
double estimateStaticCapacitance(const complexf *impedances,
                                 uint8_t count,
                                 float startFrequency,
                                 float frequencyStep)
{
    double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
    double sxz = 0.0, syz = 0.0, sz = 0.0, sw = 0.0;
    int valid = 0;

    for (uint8_t i = 0; i < count; i++) {
        double x, y;
        if (!sampleAdmittance(impedances[i], x, y)) {
            continue;
        }

        double z2 = x * x + y * y;

        sx += x;
        sy += y;
        sxx += x * x;
        syy += y * y;
        sxy += x * y;
        sxz += x * z2;
        syz += y * z2;
        sz += z2;
        sw += omegaAt(startFrequency, frequencyStep, i);
        valid++;
    }

    if (valid < TRANSDUCER_ANALYZER_MIN_POINTS) {
        return 0.0;
    }

    // Normal equations for x^2 + y^2 = A*x + B*y + C (Cramer's rule).
    double n = (double)valid;
    double det = sxx * (syy * n - sy * sy) -
                 sxy * (sxy * n - sy * sx) +
                 sx * (sxy * sy - syy * sx);

    if (det == 0.0) {
        return 0.0;
    }

    double det_b = sxx * (syz * n - sy * sz) -
                   sxz * (sxy * n - sy * sx) +
                   sx * (sxy * sz - syz * sx);

    double cy = 0.5 * det_b / det;
    double c0 = cy / (sw / n);

    if (!std::isfinite(c0) || c0 < 0.0) {
        return 0.0;
    }

    return c0;
}

// With C0 fixed, the motional impedance Zm = 1/(Y - jwC0) satisfies
// Re(Zm) = R1 and w*Im(Zm) = w^2*L1 - 1/C1 — ordinary linear regression
// of y = w*Im(Zm) against x = w^2.
bool fitMotionalBranch(const complexf *impedances,
                       uint8_t count,
                       float startFrequency,
                       float frequencyStep,
                       double c0,
                       double &r1,
                       double &l1,
                       double &invC1)
{
    // First pass: peak motional admittance, to set the inclusion floor.
    double ym_abs2_max = 0.0;
    for (uint8_t i = 0; i < count; i++) {
        double y_re, y_im;
        if (!sampleAdmittance(impedances[i], y_re, y_im)) {
            continue;
        }

        double w = omegaAt(startFrequency, frequencyStep, i);
        y_im -= w * c0;

        double ym_abs2 = y_re * y_re + y_im * y_im;
        if (ym_abs2 > ym_abs2_max) {
            ym_abs2_max = ym_abs2;
        }
    }

    double ym_abs2_floor = ym_abs2_max *
        (double)(TRANSDUCER_ANALYZER_MIN_REL_YM * TRANSDUCER_ANALYZER_MIN_REL_YM);

    double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    double sum_r = 0.0, sum_weight = 0.0;
    int valid = 0;

    for (uint8_t i = 0; i < count; i++) {
        double y_re, y_im;
        if (!sampleAdmittance(impedances[i], y_re, y_im)) {
            continue;
        }

        double w = omegaAt(startFrequency, frequencyStep, i);
        y_im -= w * c0;

        double ym_abs2 = y_re * y_re + y_im * y_im;
        if (ym_abs2 <= 0.0 || ym_abs2 < ym_abs2_floor) {
            continue;
        }

        // Negative motional conductance is not passive — noise; skip.
        if (y_re <= 0.0) {
            continue;
        }

        double zm_re = y_re / ym_abs2;
        double zm_im = -y_im / ym_abs2;

        double x = w * w;
        double y = w * zm_im;

        // Weight by |Ym|^2: the I-sense SNR scales with the drawn current,
        // so points near series resonance carry the reliable phase and the
        // noise-dominated shoulders are attenuated instead of trusted.
        double weight = ym_abs2 / ym_abs2_max;

        sum_x += weight * x;
        sum_y += weight * y;
        sum_xx += weight * x * x;
        sum_xy += weight * x * y;
        sum_r += weight * zm_re;
        sum_weight += weight;
        valid++;
    }

    if (valid < TRANSDUCER_ANALYZER_MIN_POINTS || sum_weight <= 0.0) {
        return false;
    }

    double denom = sum_weight * sum_xx - sum_x * sum_x;
    if (denom <= 0.0) {
        return false;
    }

    double slope = (sum_weight * sum_xy - sum_x * sum_y) / denom;
    double intercept = (sum_y - slope * sum_x) / sum_weight;

    if (slope <= 0.0 || intercept >= 0.0) {
        return false;
    }

    r1 = sum_r / sum_weight;
    l1 = slope;
    invC1 = -intercept;
    return true;
}

// Refines C0 against the residual susceptance:
// Im(Y_i) - Im(Ym_model,i) = w_i*C0, least squares over all points.
bool refineStaticCapacitance(const complexf *impedances,
                             uint8_t count,
                             float startFrequency,
                             float frequencyStep,
                             double r1,
                             double l1,
                             double invC1,
                             double &c0)
{
    double sum_wb = 0.0, sum_ww = 0.0;

    for (uint8_t i = 0; i < count; i++) {
        double y_re, y_im;
        if (!sampleAdmittance(impedances[i], y_re, y_im)) {
            continue;
        }

        double w = omegaAt(startFrequency, frequencyStep, i);

        double xm = w * l1 - invC1 / w;
        double zm_abs2 = r1 * r1 + xm * xm;
        if (zm_abs2 <= 0.0) {
            continue;
        }

        double ym_model_im = -xm / zm_abs2;

        sum_wb += w * (y_im - ym_model_im);
        sum_ww += w * w;
    }

    if (sum_ww <= 0.0) {
        return false;
    }

    c0 = sum_wb / sum_ww;
    return true;
}

} // namespace

// ---------------------------------------------------------------------------
// Public-Interface
// ---------------------------------------------------------------------------

bool TransducerAnalyzer::fit(const complexf *impedances,
                       uint8_t count,
                       float startFrequency,
                       float frequencyStep,
                       Parameters &out)
{
    if (impedances == nullptr || count < TRANSDUCER_ANALYZER_MIN_POINTS ||
        startFrequency <= 0.0f || frequencyStep <= 0.0f) {
        return false;
    }

    // Double accumulators are affordable (the fit runs once per bond) and
    // avoid cancellation in the w^2 terms (~1e11).
    double c0 = estimateStaticCapacitance(impedances, count,
                                          startFrequency, frequencyStep);
    double r1 = 0.0;
    double l1 = 0.0;
    double inv_c1 = 0.0;

    // Alternate between the motional-branch regression and the C0
    // least-squares update; the pair converges to the joint optimum.
    for (int iteration = 0; iteration < TRANSDUCER_ANALYZER_C0_ITERATIONS; iteration++) {
        if (!fitMotionalBranch(impedances, count, startFrequency, frequencyStep,
                               c0, r1, l1, inv_c1)) {
            return false;
        }

        if (!refineStaticCapacitance(impedances, count, startFrequency, frequencyStep,
                                     r1, l1, inv_c1, c0)) {
            return false;
        }
    }

    if (r1 <= 0.0 || l1 <= 0.0 || inv_c1 <= 0.0 || c0 <= 0.0) {
        return false;
    }

    double c1 = 1.0 / inv_c1;
    double ws = std::sqrt(inv_c1 / l1);

    out.r1 = (float)r1;
    out.l1 = (float)l1;
    out.c1 = (float)c1;
    out.c0 = (float)c0;
    out.seriesResonance = (float)(ws / (2.0 * M_PI));
    out.parallelResonance = (float)((ws / (2.0 * M_PI)) * std::sqrt(1.0 + c1 / c0));
    out.qFactor = (float)(ws * l1 / r1);

    if (!isPositiveFinite(out.seriesResonance) ||
        !isPositiveFinite(out.parallelResonance) ||
        !isPositiveFinite(out.qFactor)) {
        return false;
    }

    if (out.qFactor < TRANSDUCER_ANALYZER_MIN_QFACTOR) {
        return false;
    }

    return true;
}

complexf TransducerAnalyzer::admittance(const Parameters &params, float frequency)
{
    float w = 2.0f * (float)M_PI * frequency;

    complexf zm = complexf_create(params.r1, w * params.l1 - 1.0f / (w * params.c1));
    complexf ym = complexf_div(complexf_create(1.0f, 0.0f), zm);

    return complexf_add(ym, complexf_create(0.0f, w * params.c0));
}

bool TransducerAnalyzer::frequencyPidTuning(const Parameters &params,
                                            float deadTime,
                                            float phaseMargin,
                                            float crossoverFraction,
                                            FrequencyPidTuning &out)
{
    if (!isPositiveFinite(deadTime) ||
        !isPositiveFinite(phaseMargin) ||
        !isPositiveFinite(crossoverFraction) ||
        !isPositiveFinite(params.qFactor) ||
        !isPositiveFinite(params.seriesResonance)) {
        return false;
    }

    // Plant gain: phase slope of the motional branch at series resonance.
    float plant_gain = 2.0f * params.qFactor / params.seriesResonance;

    // Open loop at crossover wc = c/deadTime:
    //   phase = -c - atan(1/(wc*Ti))          (dead time + PI lag)
    //   |L|   = Kp*K*sqrt(1 + 1/(wc*Ti)^2)
    // Setting phase = -pi + phaseMargin fixes the PI lag phi; the
    // unit-magnitude condition then gives Kp = cos(phi)/K.
    float wc = crossoverFraction / deadTime;
    float pi_phase = (float)M_PI - phaseMargin - crossoverFraction;

    // The PI lag must lie strictly inside (0, pi/2): below 0 the margin is
    // unreachable even without integral action; pi/2 needs infinite lag.
    if (pi_phase <= 0.0f || pi_phase >= 0.5f * (float)M_PI) {
        return false;
    }

    out.gain = cosf(pi_phase) / plant_gain;
    out.integralTc = 1.0f / (wc * tanf(pi_phase));
    out.derivativeTc = 0.0f;

    return isPositiveFinite(out.gain) && isPositiveFinite(out.integralTc);
}
