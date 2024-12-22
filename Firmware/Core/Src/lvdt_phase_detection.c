#include "lvdt_phase_detection.h"
#include "adc_dispatcher.h"      // Our modular ADC dispatcher
#include <math.h>                // for sinf, atan2f, etc. if needed

/* ---------------- Definitions & Variables -------------- */

#define LVDT_SAMPLE_RATE_HZ   (4000U)  // 4 kHz sampling rate

// Track the indices returned by the ADC dispatcher for each channel
static int8_t sChannelIndexA = -1; // for ADC_CHANNEL_6 (A)
static int8_t sChannelIndexB = -1; // for ADC_CHANNEL_7 (B)

static void adcCallbackChannelA(uint8_t channelIndex, uint16_t value);
static void adcCallbackChannelB(uint8_t channelIndex, uint16_t value);

/* ------------------------------------------------------- */
/*                 Public Functions                        */
/* ------------------------------------------------------- */

void LVDT_PhaseDetection_Init(void)
{
    // 1) Register the two channels we need:
    //    - Channel A: ADC_CHANNEL_6
    //    - Channel B: ADC_CHANNEL_7
    //    We pass NULL for the callback if we just want to poll.
    sChannelIndexA = ADC_Dispatcher_RegisterChannel(ADC_CHANNEL_6, NULL);
    sChannelIndexB = ADC_Dispatcher_RegisterChannel(ADC_CHANNEL_7, NULL);
}

void LVDT_PhaseDetection_Start(void)
{
    // Simply start the dispatcher
    ADC_Dispatcher_Start();
}

void LVDT_PhaseDetection_Stop(void)
{
    // Stop the dispatcher (stops timer + ADC + DMA)
    ADC_Dispatcher_Stop();
}

static void adcCallbackChannelA(uint8_t channelIndex, uint16_t value)
{
}

static void adcCallbackChannelB(uint8_t channelIndex, uint16_t value)
{
}

void compute4PointPhase(const uint16_t samples[4], 
                        float *amplitude, 
                        float *offset, 
                        float *phaseRad)
{
    // 1. Compute offset
    float sum = (float)samples[0] + (float)samples[1] + 
                (float)samples[2] + (float)samples[3];
    *offset = sum / 4.0f;

    // 2. Subtract offset to get the AC values
    float S1 = (float)samples[0] - (*offset);
    float S2 = (float)samples[1] - (*offset);
    float S3 = (float)samples[2] - (*offset);
    float S4 = (float)samples[3] - (*offset);

    // 3. Compute I, Q
    float I = S1 - S3;  // ~ 2A sin(phi)
    float Q = S2 - S4;  // ~ 2A cos(phi)

    // 4. Amplitude
    float magnitude = sqrtf(I * I + Q * Q);
    *amplitude = magnitude / 2.0f;

    // 5. Phase in radians (range: -pi..+pi)
    *phaseRad = atan2f(I, Q);
}
