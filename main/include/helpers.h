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

// GPIO for analog volume control and DAC audio output.
#define VOLUME_PIN 36 // ADC1_CH0
#define OUTPUT_PIN 25 // DAC_CH1
#define OCTAVE_UP_PIN 33
#define OCTAVE_DOWN_PIN 32

// ---------------------------
// Note Frequencies (Hz)
// ---------------------------
// One-octave equal-tempered scale reference frequencies.

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

// Number of note keys in one octave.
#define NOTE_COUNT 12

// Shared note metadata used by ISR and synthesis code.
const gpio_num_t *helpers_get_note_pins(void);
const float *helpers_get_note_freqs(void);

// Returns [0, NOTE_COUNT) for a valid note pin, or -1 when unknown.
int helpers_note_index_from_pin(int pin);

// Fills a sine LUT in the range [-1.0, 1.0].
void helpers_init_sine_lut(float *lut, size_t lut_size);

// Converts normalized audio sample [-1.0, 1.0] to unsigned DAC byte [0, 255].
uint8_t helpers_mix_to_dac_u8(float sample);

#endif // HELPERS_H