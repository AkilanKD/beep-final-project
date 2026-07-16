#ifndef HELPERS_H
#define HELPERS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "driver/gpio.h"

// ---------------------------
// Pin Mapping
// ---------------------------
// GPIO pin used by each piano key button.
#define C_PIN 15
#define C_SHARP_PIN 13
#define D_PIN 14
#define D_SHARP_PIN 4
#define E_PIN 16
#define F_PIN 17
#define F_SHARP_PIN 5
#define G_PIN 18
#define G_SHARP_PIN 19
#define A_PIN 21
#define A_SHARP_PIN 22
#define B_PIN 23

// GPIO/channel for analog volume control and DAC audio output.
#define VOLUME_CHANNEL ADC_CHANNEL_7 // ADC1_CH7
#define OUTPUT_PIN 25 // DAC_CH1

// GPIO for octave shifts.
#define OCTAVE_UP_PIN 33 // ADC1_CH5
#define OCTAVE_DOWN_PIN 32 // ADC1_CH4

// ---------------------------
// Note Frequencies (Hz)
// ---------------------------
// One-octave equal-tempered scale reference frequencies (using octave 4).

#define C_FREQ 261.63
#define C_SHARP_FREQ 277.18
#define D_FREQ 293.66
#define D_SHARP_FREQ 311.13
#define E_FREQ 329.63
#define F_FREQ 349.23
#define F_SHARP_FREQ 369.99
#define G_FREQ 392
#define G_SHARP_FREQ 415.30
#define A_FREQ 440
#define A_SHARP_FREQ 466.16
#define B_FREQ 493.88

// Legacy debounce constant kept for compatibility with older code paths.
#define DEBOUNCE_TIME 100000

// Minimum and maximum octave shifts from octave 4.
#define MIN_OCTAVE_SHIFT -2 // Minimum octave shift (equal to octave 2)
#define MAX_OCTAVE_SHIFT 2 // Maximum octave shift (equal to octave 6)
// Number of octaves
#define OCTAVE_COUNT (MAX_OCTAVE_SHIFT - MIN_OCTAVE_SHIFT + 1)

// Number of note keys in one octave.
#define OCTAVE_NOTE_COUNT 12
// Number of total note keys
#define NOTE_COUNT (OCTAVE_COUNT * OCTAVE_NOTE_COUNT)

#define MIN_VOLUME 0
#define MAX_VOLUME 8
#define LOUDNESS_COUNT (128 + 1)

// Constants defined by ISO 226:2023
#define P0_PA_2 4e-10 // Value of (P_0 / P_a)^2
#define A_R 0.3 // Exponent for loudness perception at 1000 Hz
#define T_R 2.4 // Threshold of hearing at 1000 Hz (dB)

#define IMPEDENCE 8 // Impedence of speaker (Ω)
#define SENSITIVITIY 82


// Represents a experimental data point used by ISO 226:2023 to develop equal-loudness contours
typedef struct {
    int f;     // Frequency (Hz)
    float a_f; // Exponent of loudness perception
    float L_U; // Magnitude of linear transformation function (dB)
    float T_f; // Threshold of hearing (dB)
} data_point;

const gpio_num_t *helpers_get_note_pins(void);
void helpers_init_note_freqs(float s_note_freqs[NOTE_COUNT]);
int helpers_note_index_from_pin(int pin);
void helpers_init_sine_lut(float *lut, size_t lut_size);
uint8_t helpers_mix_to_dac_u8(float sample);
void helpers_init_loudnesses(double s_loudnesses[LOUDNESS_COUNT][NOTE_COUNT]);

#endif // HELPERS_H