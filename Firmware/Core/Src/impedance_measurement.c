#include "impedance_measurement.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ----------------- Definitions ----------------- */
#define NUM_SAMPLES_PER_SET 4       // Each set has 4 pairs (i.e. 4 current and 4 voltage samples)
                                   // (The four-point phase algorithm operates on 4 samples.)
// The DMA buffer is sized 320 (i.e. 4 × 40 × 2). In each DMA callback (half or full),
// we will receive 160 values which corresponds to 160/2 = 80 pairs. Since each set has 4 pairs,
// the number of complete sets per DMA callback = 80/4 = 20.

/* ----------------- State Definitions ----------------- */
typedef enum {
    IMPEDANCE_STATE_UNINIT = 0,
    IMPEDANCE_STATE_READY,
    IMPEDANCE_STATE_OPERATING
} ImpedanceState_t;

static ImpedanceState_t sImpedanceState = IMPEDANCE_STATE_UNINIT;

/* ----------------- Private Variables ----------------- */
// We use a local processing function to extract sets from the DMA buffer.
// No global accumulators or counters are needed since the DMA buffer length is fixed.
 
// Registered callback functions.
#define MAX_IMPEDANCE_CALLBACKS 8
static ImpedanceMeasurementCallback sImpedanceCallbacks[MAX_IMPEDANCE_CALLBACKS];
static uint8_t sNumImpedanceCallbacks = 0;

/* ----------------- Private Function Prototypes ----------------- */
static void compute4PointPhase(const uint16_t samples[NUM_SAMPLES_PER_SET],
                               float *amplitude,
                               float *offset,
                               float *phaseRad);
                               
static void processDMAData(uint16_t *dmaBuffer, uint16_t numElements);

/* ----------------- Helper Functions ----------------- */
/**
 * @brief Fast approximate atan2 function.
 *
 * This function approximates atan2(y, x) with moderate accuracy.
 * The result is in radians in the range [-π, π].
 *
 * @param y The numerator.
 * @param x The denominator.
 * @return Approximation of atan2(y, x).
 */
static inline float fast_atan2f(float y, float x)
{
    /* Handle the degenerate case (both x and y are zero) */
    if (x == 0.0f && y == 0.0f)
        return 0.0f;

    float abs_y = fabsf(y);
    float angle;

    if (x >= 0.0f)
    {
        /* Compute a ratio between (x - |y|) and (x + |y|) */
        float r = (x - abs_y) / (x + abs_y);
        /* Map r linearly into an angle between 0 and π/4 */
        angle = (float)(M_PI / 4.0f) - ((float)M_PI / 4.0f) * r;
    }
    else
    {
        /* When x is negative, adjust the ratio accordingly */
        float r = (x + abs_y) / (abs_y - x);
        angle = (float)(3.0f * M_PI / 4.0f) - ((float)M_PI / 4.0f) * r;
    }
    /* Adjust the sign based on y */
    return (y < 0.0f) ? -angle : angle;
}

/**
 * @brief Compute a four-point phase measurement from ADC samples.
 *
 * The algorithm computes:
 *   1. The DC offset as the mean of the 4 samples.
 *   2. The AC components by subtracting the offset.
 *   3. I = S1 - S3 and Q = S2 - S4.
 *   4. Amplitude as sqrt(I^2 + Q^2)/2.
 *   5. Phase using atan2f(I, Q).
 *
 * @param samples Array of 4 ADC samples.
 * @param amplitude (out) Computed amplitude.
 * @param offset (out) Computed DC offset.
 * @param phaseRad (out) Computed phase in radians.
 */
static void compute4PointPhase(const uint16_t samples[NUM_SAMPLES_PER_SET],
                               float *amplitude,
                               float *offset,
                               float *phaseRad)
{
    float sum = (float)samples[0] + samples[1] + samples[2] + samples[3];
    *offset = sum / 4.0f;
    
    float S1 = samples[0] - *offset;
    float S2 = samples[1] - *offset;
    float S3 = samples[2] - *offset;
    float S4 = samples[3] - *offset;
    
    float I = S1 - S3;
    float Q = S2 - S4;
    
    float mag = sqrtf(I * I + Q * Q);
    *amplitude = mag / 2.0f;
    *phaseRad = fast_atan2f(I, Q);
}

/**
 * @brief Process the DMA buffer data.
 *
 * The DMA buffer contains interleaved measurements:
 *   even indices: current measurements,
 *   odd indices: voltage measurements.
 *
 * The buffer length (numElements) is assumed to be half of the full DMA buffer
 * (for a half-transfer or full-transfer callback). For example, if the full DMA buffer
 * is 320 elements, each callback will supply 160 elements.
 *
 * From the DMA buffer, we compute the number of pairs as numPairs = numElements/2.
 * Then we group these pairs into sets of 4 pairs each:
 *   numSets = (numElements/2) / 4 = numElements/8.
 *
 * For each sample position (0..3) within a set, we average the corresponding
 * current and voltage values across all sets. Then the averaged 4 samples (per channel)
 * are used to compute amplitude and phase via the four-point phase algorithm.
 *
 * Finally, the complex impedance is computed as:
 *   |Z| = V_ampl / I_ampl,  phase_Z = V_phase - I_phase (wrapped into [-π, π]),
 * and all registered callbacks are invoked.
 *
 * @param dmaBuffer Pointer to the DMA buffer (half or full).
 * @param numElements Number of uint16_t elements in the buffer.
 */
static void processDMAData(uint16_t *dmaBuffer, uint16_t numElements)
{
    // Number of pairs in the buffer.
    uint16_t numPairs = numElements / 2;
    // Number of complete sets (each set is 4 pairs).
    uint16_t numSets = numPairs / NUM_SAMPLES_PER_SET;
    
    if (numSets == 0)
        return;  // Not enough data.
    
    // Accumulators for each sample position (0, 1, 2, 3) for both channels.
    uint32_t sumCurrent[NUM_SAMPLES_PER_SET] = {0};
    uint32_t sumVoltage[NUM_SAMPLES_PER_SET] = {0};
    
    // Process each set.
    // For each set k (0 to numSets-1), the set consists of 4 consecutive pairs.
    // For set k and sample position j (0..3):
    //   pair index = 4*k + j,
    //   current measurement = dmaBuffer[2*(4*k + j)],
    //   voltage measurement = dmaBuffer[2*(4*k + j) + 1].
    for (uint16_t k = 0; k < numSets; k++) {
        for (uint8_t j = 0; j < NUM_SAMPLES_PER_SET; j++) {
            uint16_t pairIndex = 4 * k + j;
            uint16_t currentVal = dmaBuffer[2 * pairIndex];
            uint16_t voltageVal = dmaBuffer[2 * pairIndex + 1];
            sumCurrent[j] += currentVal;
            sumVoltage[j] += voltageVal;
        }
    }
    
    // Compute average for each sample position.
    uint16_t avgCurrent[NUM_SAMPLES_PER_SET];
    uint16_t avgVoltage[NUM_SAMPLES_PER_SET];
    for (uint8_t j = 0; j < NUM_SAMPLES_PER_SET; j++) {
        avgCurrent[j] = (uint16_t)(sumCurrent[j] / numSets);
        avgVoltage[j] = (uint16_t)(sumVoltage[j] / numSets);
    }
    
    // Compute four-point phase for voltage and current.
    float V_ampl, V_offset, V_phase;
    float I_ampl, I_offset, I_phase;
    compute4PointPhase(avgVoltage, &V_ampl, &V_offset, &V_phase);
    compute4PointPhase(avgCurrent, &I_ampl, &I_offset, &I_phase);
    
    ComplexImpedance impedance;
    if (I_ampl == 0.0f) {
        impedance.magnitude = 0.0f;
        impedance.phase = 0.0f;
    } else {
        impedance.magnitude = V_ampl / I_ampl;
        impedance.phase = V_phase - I_phase;
        while (impedance.phase > M_PI)
            impedance.phase -= 2.0f * M_PI;
        while (impedance.phase < -M_PI)
            impedance.phase += 2.0f * M_PI;
    }
    
    // Notify all registered callbacks.
    for (uint8_t i = 0; i < sNumImpedanceCallbacks; i++) {
        if (sImpedanceCallbacks[i])
            sImpedanceCallbacks[i](impedance);
    }
}

/* ----------------- Public Functions ----------------- */

void ImpedanceMeasurement_Init(void)
{
    if (sImpedanceState != IMPEDANCE_STATE_UNINIT)
        return;
    
    sImpedanceState = IMPEDANCE_STATE_READY;
    sNumImpedanceCallbacks = 0;
}

void ImpedanceMeasurement_Start(void)
{
    if (sImpedanceState != IMPEDANCE_STATE_READY)
        return;
    
    sImpedanceState = IMPEDANCE_STATE_OPERATING;
}

void ImpedanceMeasurement_Stop(void)
{
    if (sImpedanceState != IMPEDANCE_STATE_OPERATING)
        return;
    
    sImpedanceState = IMPEDANCE_STATE_READY;
}

void ImpedanceMeasurement_RegisterCallback(ImpedanceMeasurementCallback callback)
{
    if (callback && sNumImpedanceCallbacks < MAX_IMPEDANCE_CALLBACKS)
        sImpedanceCallbacks[sNumImpedanceCallbacks++] = callback;
}

/**
 * @brief DMA half-transfer callback.
 *
 * This function should be called by the DMA half-transfer interrupt.
 * @param buffer Pointer to the first half of the DMA buffer.
 * @param length Number of uint16_t elements in this half-buffer.
 */
void ImpedanceMeasurement_DMAHalfCallback(uint16_t *buffer, uint16_t length)
{
    if (sImpedanceState != IMPEDANCE_STATE_OPERATING)
        return;
    
    processDMAData(buffer, length);
}

/**
 * @brief DMA transfer complete callback.
 *
 * This function should be called by the DMA transfer complete interrupt.
 * @param buffer Pointer to the second half (or full DMA buffer as configured) of the DMA buffer.
 * @param length Number of uint16_t elements in this buffer.
 */
void ImpedanceMeasurement_DMAFullCallback(uint16_t *buffer, uint16_t length)
{
    if (sImpedanceState != IMPEDANCE_STATE_OPERATING)
        return;
    
    processDMAData(buffer, length);
}
