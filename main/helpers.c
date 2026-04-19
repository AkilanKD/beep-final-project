#include <math.h>
#include "helpers.h"

// GPIO order matches frequency order so indices can be shared everywhere.
static const gpio_num_t s_note_pins[NOTE_COUNT] = {
	C_PIN, C_SHARP_PIN, D_PIN, D_SHARP_PIN, E_PIN, F_PIN,
	F_SHARP_PIN, G_PIN, G_SHARP_PIN, A_PIN, A_SHARP_PIN, B_PIN
};

// Frequencies for one octave, aligned by index with s_note_pins.
static const float s_note_freqs[NOTE_COUNT] = {
	C_FREQ, C_SHARP_FREQ, D_FREQ, D_SHARP_FREQ, E_FREQ, F_FREQ,
	F_SHARP_FREQ, G_FREQ, G_SHARP_FREQ, A_FREQ, A_SHARP_FREQ, B_FREQ
};

/*
 * Returns a pointer to the static note-pin table.
 */
const gpio_num_t *helpers_get_note_pins(void)
{
	return s_note_pins;
}

/*
 * Returns a pointer to the static note-frequency table.
 */
const float *helpers_get_note_freqs(void)
{
	return s_note_freqs;
}

/*
 * Finds the note index for a GPIO pin.
 * Returns -1 when pin does not correspond to any known note key.
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
 * Fills lut with one full sine cycle in the range [-1.0, 1.0].
 */
void helpers_init_sine_lut(float *lut, size_t lut_size)
{
	// Guard invalid caller input.
	if (lut == NULL || lut_size == 0) {
		return;
	}

	for (size_t i = 0; i < lut_size; ++i) {
		// phase goes from 0..2pi across the table.
		const float phase = 2.0f * (float)M_PI * ((float)i / (float)lut_size);
		lut[i] = sinf(phase);
	}
}

/*
 * Converts normalized sample [-1.0, 1.0] to DAC byte [0, 255].
 */
uint8_t helpers_mix_to_dac_u8(float sample)
{
	// Scale to signed 8-bit range then shift into unsigned domain.
	int sample_u8 = (int)(sample * 127.0f) + 128;
	// Clamp for safety if caller sends values outside [-1, 1].
	if (sample_u8 < 0) {
		sample_u8 = 0;
	} else if (sample_u8 > 255) {
		sample_u8 = 255;
	}
	return (uint8_t)sample_u8;
}
