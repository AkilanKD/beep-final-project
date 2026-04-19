#include <math.h>
#include "helpers.h"

static const gpio_num_t s_note_pins[NOTE_COUNT] = {
	C_PIN, C_SHARP_PIN, D_PIN, D_SHARP_PIN, E_PIN, F_PIN,
	F_SHARP_PIN, G_PIN, G_SHARP_PIN, A_PIN, A_SHARP_PIN, B_PIN
};

static const float s_note_freqs[NOTE_COUNT] = {
	C_FREQ, C_SHARP_FREQ, D_FREQ, D_SHARP_FREQ, E_FREQ, F_FREQ,
	F_SHARP_FREQ, G_FREQ, G_SHARP_FREQ, A_FREQ, A_SHARP_FREQ, B_FREQ
};

/*
 * Returns the array of note pins
 */
const gpio_num_t *helpers_get_note_pins(void)
{
	return s_note_pins;
}

/*
 * Returns the array of note frequencies
 */
const float *helpers_get_note_freqs(void)
{
	return s_note_freqs;
}

/*
 * Returns the index of a note, given its pin
 */
int helpers_note_index_from_pin(int pin)
{
	for (int i = 0; i < NOTE_COUNT; ++i) {
		if ((int)s_note_pins[i] == pin) {
			return i;
		}
	}
	return -1;
}

/*
 * Precalculates the sine phases for all the frequencies in the lookup table
 */
void helpers_init_sine_lut(float *lut, size_t lut_size)
{
	if (lut == NULL || lut_size == 0) {
		return;
	}

	for (size_t i = 0; i < lut_size; ++i) {
		const float phase = 2.0f * (float)M_PI * ((float)i / (float)lut_size);
		lut[i] = sinf(phase);
	}
}

uint8_t helpers_mix_to_dac_u8(float sample)
{
	int sample_u8 = (int)(sample * 127.0f) + 128;
	if (sample_u8 < 0) {
		sample_u8 = 0;
	} else if (sample_u8 > 255) {
		sample_u8 = 255;
	}
	return (uint8_t)sample_u8;
}
