#include "lvdt_phase_detection.h"
#include "adc_dispatcher.h"      // Our modular ADC dispatcher
#include <math.h>                // for sqrtf, fabsf, etc.
#include <stdint.h>
#include <stdbool.h>

/* ---------------- Definitions & Variables -------------- */
#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif

// Sampling rate is 4 kHz.
#define LVDT_SAMPLE_RATE_HZ   (4000U)
// Use 4 successive samples per channel to compute phase.
#define NUM_SAMPLES 4

// Define the module states.
typedef enum {
    LVDT_STATE_UNINIT = 0,
    LVDT_STATE_READY,
    LVDT_STATE_OPERATING
} LVDT_State_t;

// Internal state variable.
static LVDT_State_t sLVDTState = LVDT_STATE_UNINIT;

// ADC channel indices (assigned by the dispatcher)
static int8_t sChannelIndexA = -1; // e.g. ADC_CHANNEL_6 (A)
static int8_t sChannelIndexB = -1; // e.g. ADC_CHANNEL_7 (B)

// Buffers to hold NUM_SAMPLES for each channel.
static uint16_t sSamplesA[NUM_SAMPLES];
static uint16_t sSamplesB[NUM_SAMPLES];

// Buffer indices.
static uint8_t sSampleIndexA = 0;
static uint8_t sSampleIndexB = 0;

// Latest computed phase for each channel (in radians).
static float sPhaseA = 0.0f;
static float sPhaseB = 0.0f;

// Flags indicating that a new phase value is available for each channel.
static bool sNewPhaseAReady = false;
static bool sNewPhaseBReady = false;

/* Maximum number of phase callback functions that can be registered. */
#define MAX_PHASE_CALLBACKS 8
static LVDT_PhaseCallback sPhaseCallbacks[MAX_PHASE_CALLBACKS];
static uint8_t sNumPhaseCallbacks = 0;

/* Forward declarations for the ADC callback functions. */
static void adcCallbackChannelA(uint8_t channelIndex, uint16_t value);
static void adcCallbackChannelB(uint8_t channelIndex, uint16_t value);

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

/*-----------------------------------------------------------
  compute4PointPhase

  Given 4 samples, this function computes:
    1. The DC offset (mean) of the samples.
    2. The AC components (samples minus offset).
    3. I = S1 – S3 (proportional to sine of the phase).
    4. Q = S2 – S4 (proportional to cosine of the phase).
    5. Amplitude and phase (using fast_atan2f) of the waveform.
  
  @param samples   Array of 4 ADC samples.
  @param amplitude Pointer to store the computed amplitude.
  @param offset    Pointer to store the computed offset.
  @param phaseRad  Pointer to store the computed phase (in radians, –π to +π).
-----------------------------------------------------------*/
void compute4PointPhase(const uint16_t samples[NUM_SAMPLES], 
                        float *amplitude, 
                        float *offset, 
                        float *phaseRad)
{
    /* 1. Compute DC offset */
    float sum = (float)samples[0] + (float)samples[1] +
                (float)samples[2] + (float)samples[3];
    *offset = sum / 4.0f;

    /* 2. Subtract offset to get AC values */
    float S1 = (float)samples[0] - (*offset);
    float S2 = (float)samples[1] - (*offset);
    float S3 = (float)samples[2] - (*offset);
    float S4 = (float)samples[3] - (*offset);

    /* 3. Compute I and Q */
    float I = S1 - S3;  // ~ 2A * sin(phase)
    float Q = S2 - S4;  // ~ 2A * cos(phase)

    /* 4. Compute amplitude */
    float magnitude = sqrtf(I * I + Q * Q);
    *amplitude = magnitude / 2.0f;

    /* 5. Compute phase in radians using fast_atan2f */
    *phaseRad = fast_atan2f(I, Q);
}

/*-----------------------------------------------------------
  Helper: notifyPhaseDifference

  Computes the phase difference (channel A minus channel B), wraps it to 
  the range –π to +π, and calls every registered phase callback.
-----------------------------------------------------------*/
static void notifyPhaseDifference(void)
{
    float phaseDiff = LVDT_GetPhaseDifference();
    for (uint8_t i = 0; i < sNumPhaseCallbacks; i++)
    {
        if (sPhaseCallbacks[i] != NULL)
        {
            sPhaseCallbacks[i](phaseDiff);
        }
    }
}

/*-----------------------------------------------------------
  Public Functions
-----------------------------------------------------------*/

/**
 * @brief Initializes the LVDT phase detection module.
 *
 * This function registers the ADC channels and sets the module state to READY.
 * It is allowed only when the module is in UNINIT state.
 */
void LVDT_PhaseDetection_Init(void)
{
    if (sLVDTState != LVDT_STATE_UNINIT)
    {
        // Already initialized – ignore repeated calls.
        return;
    }

    /* Register ADC channels with their respective callbacks */
    sChannelIndexA = ADC_Dispatcher_RegisterChannel(ADC_CHANNEL_6, adcCallbackChannelA);
    sChannelIndexB = ADC_Dispatcher_RegisterChannel(ADC_CHANNEL_7, adcCallbackChannelB);

    sLVDTState = LVDT_STATE_READY;
}

/**
 * @brief Starts the LVDT phase detection.
 *
 * This function starts the ADC dispatcher and sets the module state to OPERATING.
 * It is allowed only when the module state is READY.
 */
void LVDT_PhaseDetection_Start(void)
{
    if (sLVDTState != LVDT_STATE_READY)
    {
        // Not in the proper state to start.
        return;
    }

    ADC_Dispatcher_Start();
    sLVDTState = LVDT_STATE_OPERATING;
}

/**
 * @brief Stops the LVDT phase detection.
 *
 * This function stops the ADC dispatcher and reverts the module state to READY.
 * It is allowed only when the module state is OPERATING.
 */
void LVDT_PhaseDetection_Stop(void)
{
    if (sLVDTState != LVDT_STATE_OPERATING)
    {
        // Not operating; nothing to stop.
        return;
    }

    ADC_Dispatcher_Stop();
    sLVDTState = LVDT_STATE_READY;
}

/**
 * Registers a phase callback.
 */
void LVDT_RegisterPhaseCallback(LVDT_PhaseCallback callback)
{
    if (callback != NULL && sNumPhaseCallbacks < MAX_PHASE_CALLBACKS)
    {
        sPhaseCallbacks[sNumPhaseCallbacks++] = callback;
    }
}

/*-----------------------------------------------------------
  ADC Callback Implementations
-----------------------------------------------------------*/

/* Channel A callback */
static void adcCallbackChannelA(uint8_t channelIndex, uint16_t value)
{
    /* Process samples only when operating */
    if (sLVDTState != LVDT_STATE_OPERATING)
    {
        return;
    }

    (void)channelIndex; // Unused parameter
    sSamplesA[sSampleIndexA++] = value;

    if (sSampleIndexA >= NUM_SAMPLES)
    {
        float amplitude, offset, phase;
        compute4PointPhase(sSamplesA, &amplitude, &offset, &phase);
        sPhaseA = phase;          // Save the computed phase for channel A.
        sNewPhaseAReady = true;   // Mark that channel A has a new phase value.
        sSampleIndexA = 0;        // Reset sample index.

        /* If channel B has also provided a new phase, notify callbacks */
        if (sNewPhaseBReady)
        {
            notifyPhaseDifference();
            sNewPhaseAReady = false;
            sNewPhaseBReady = false;
        }
    }
}

/* Channel B callback */
static void adcCallbackChannelB(uint8_t channelIndex, uint16_t value)
{
    /* Process samples only when operating */
    if (sLVDTState != LVDT_STATE_OPERATING)
    {
        return;
    }

    (void)channelIndex; // Unused parameter
    sSamplesB[sSampleIndexB++] = value;

    if (sSampleIndexB >= NUM_SAMPLES)
    {
        float amplitude, offset, phase;
        compute4PointPhase(sSamplesB, &amplitude, &offset, &phase);
        sPhaseB = phase;          // Save the computed phase for channel B.
        sNewPhaseBReady = true;   // Mark that channel B has a new phase value.
        sSampleIndexB = 0;        // Reset sample index.

        /* If channel A has also provided a new phase, notify callbacks */
        if (sNewPhaseAReady)
        {
            notifyPhaseDifference();
            sNewPhaseAReady = false;
            sNewPhaseBReady = false;
        }
    }
}

/*-----------------------------------------------------------
  Compute and return phase difference

  Computes (sPhaseA – sPhaseB) and wraps it into the range –π .. +π.
-----------------------------------------------------------*/
float LVDT_GetPhaseDifference(void)
{
    float phaseDiff = sPhaseA - sPhaseB;

    /* Wrap the phase difference to the range [-π, +π] */
    while (phaseDiff > M_PI)
        phaseDiff -= 2.0f * M_PI;
    while (phaseDiff < -M_PI)
        phaseDiff += 2.0f * M_PI;

    return phaseDiff;
}
