#include <math.h>
#include "helpers.h"

// GPIO order matches frequency order so indices can be shared everywhere.
static const gpio_num_t s_note_pins[OCTAVE_NOTE_COUNT] = {
	C_PIN, C_SHARP_PIN, D_PIN, D_SHARP_PIN, E_PIN, F_PIN,
	F_SHARP_PIN, G_PIN, G_SHARP_PIN, A_PIN, A_SHARP_PIN, B_PIN
};

// Frequencies for one octave, aligned by index with s_note_pins.
static const float s_base_note_freqs[OCTAVE_NOTE_COUNT] = {
	C_FREQ, C_SHARP_FREQ, D_FREQ, D_SHARP_FREQ, E_FREQ, F_FREQ,
	F_SHARP_FREQ, G_FREQ, G_SHARP_FREQ, A_FREQ, A_SHARP_FREQ, B_FREQ
};

static const data_point s_iso_data_points[29] = {
	{20, 0.635f, -31.5f, 78.1f},
	{25, 0.602f, -27.2f, 68.7f},
	{31, 0.569f, -23.1f, 59.5f},
	{40, 0.537f, -19.3f, 51.1f},
	{50, 0.509f, -16.1f, 44.0f},
	{63, 0.482f, -13.1f, 37.5f},
	{80, 0.456f, -10.4f, 31.5f},
	{100, 0.433f, -8.2f, 26.5f},
	{125, 0.412f, -6.3f, 22.1f},
	{160, 0.391f, -4.6f, 17.9f},
	{200, 0.373f, -3.2f, 14.4f},
	{250, 0.357f, -2.1f, 11.4f},
	{315, 0.343f, -1.2f, 8.6f},
	{400, 0.33f, -0.5f, 6.2f},
	{500, 0.32f, 0.0f, 4.4f},
	{630, 0.311f, 0.4f, 3.0f},
	{800, 0.303f, 0.5f, 2.2f},
	{1000, 0.3f, 0.0f, 2.4f},
	{1250, 0.295f, -2.7f, 3.5f},
	{1600, 0.292f, -4.2f, 1.7f},
	{2000, 0.29f, -1.2f, -1.3f},
	{2500, 0.29f, 1.4f, -4.2f},
	{3150, 0.289f, 2.3f, -6.0f},
	{4000, 0.289f, 1.0f, -5.4f},
	{5000, 0.289f, -2.3f, -1.5f},
	{6300, 0.293f, -7.2f, 6.0f},
	{8000, 0.303f, -11.2f, 12.6f},
	{10000, 0.323f, -10.9f, 13.9f},
	{12500, 0.354f, -3.5f, 12.3f},
};

/*
 * Calculates the frequency of a note, given its number.
 */
static inline float helpers_calculate_note_freq(int note_num) {
	return s_base_note_freqs[note_num % OCTAVE_NOTE_COUNT] * pow(2, (note_num / OCTAVE_NOTE_COUNT) + MIN_OCTAVE_SHIFT);
}

/*
 * Calculates the loudness pressure of a frequency (data point) and loudness (L_N) using ISO 226:2023
 */
static inline double helpers_calculate_loudness_pressure(const data_point* data_point, double L_N) {
	return ((10.0 / data_point->a_f) *
	       log10(
               (powf(P0_PA_2, A_R - data_point->a_f) * powf(10, A_R * L_N / 10) - powf(10, A_R * T_R / 10)) +
			   powf(10, data_point->a_f * (data_point->T_f + data_point->L_U) / 10)
		   )
		   ) - data_point->L_U;
}

/*
 * Converts from sones (linear) to phons (logarithmic)
 */
static inline double helpers_convert_sones_to_phons(float sones) {
	if (sones < 1) {
		return 40 * pow(sones + 0.005f, 0.350f);
	}
	return 40 + 10 * log2(sones);
}

/*
 * Calculates the voltage to the speaker to achieve the specified loudness
 */
static inline double helpers_convert_phons_to_volts(double loudness) {
	return pow(10, (loudness - SENSITIVITIY) / 20) * sqrt(IMPEDENCE);
}


/*
 * Returns a pointer to the static note-pin table.
 */
const gpio_num_t *helpers_get_note_pins(void)
{
	return s_note_pins;
}

/*
 * Calculates the note frequencies of every note, then fill out a lookup dable
 */
void helpers_init_note_freqs(float s_note_freqs[NOTE_COUNT]) {
	// Guard invalid caller input
	if (s_note_freqs == NULL) {
		return;
	}

	for (int i = 0; i < NOTE_COUNT; i++) {
		s_note_freqs[i] = helpers_calculate_note_freq(i);
	}
}

/*
 * Finds the note index for a GPIO pin, based on the core pins.
 * Returns -1 when pin does not correspond to any known note key.
 */
int helpers_note_index_from_pin(int pin)
{
	for (int i = 0; i < OCTAVE_NOTE_COUNT; ++i) {
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
	// Guard invalid caller input
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

/*
 * Initializes a 2D array of the loudnesses of every note and for every volume level.
 */
void helpers_init_loudnesses(double s_loudnesses[LOUDNESS_COUNT][NOTE_COUNT]) {
	// Guard invalid caller input
	if (s_loudnesses == NULL) {
		return;
	}

	// Maps every frequency to two reference ISO data points (which are constant)
	// The first point has a frequency directly before the given frequency
	// The second point has a frequency directly after the gtiven frequency
	const data_point *data_point_notes[NOTE_COUNT][2] = { 0 };

	int data_index = 0;
	for (int note = 0; note < NOTE_COUNT; note++) {
		// Finds the frequency of the note
		float note_freq = helpers_calculate_note_freq(note);
		// Moves the re-used index of the data to the lower bound of the frequency
		for (data_index = data_index; (data_index < 28) && (s_iso_data_points[data_index].f < note_freq); data_index++);
		data_index -= 1;

		data_point_notes[note][0] = s_iso_data_points + data_index;
		data_point_notes[note][1] = s_iso_data_points + data_index + 1;
	}

	for (int level = 0; level < LOUDNESS_COUNT; level++) {
		// Finds the phon for the given loudness level
		int L_N = helpers_convert_sones_to_phons((level * (float) (MAX_VOLUME - MIN_VOLUME) / (LOUDNESS_COUNT - 1)) + MIN_VOLUME);
		for (int note = 0; note < NOTE_COUNT; note++) {
			// Frequency of note (log)
			double log_note_freq = log10(helpers_calculate_note_freq(note));
			// Frequencies of ISO reference points (log)
			double log_before = log10(data_point_notes[note][0]->f);
			double log_after = log10(data_point_notes[note][1]->f);
			// Loudness of ISO reference points
            double L_f_before;
			double L_f_after;
			if (L_N < 10) {
				// Uses the equal-loudness contour at 10 phons, then scales down linearly to the correct loudness level
				L_f_before = (L_N / 10.0) * helpers_calculate_loudness_pressure(data_point_notes[note][0], 10);
				L_f_after = (L_N / 10.0) * helpers_calculate_loudness_pressure(data_point_notes[note][1], 10);
			}
			else {
				// Uses the equal-loudness contour at the given loudness level
				L_f_before = helpers_calculate_loudness_pressure(data_point_notes[note][0], L_N);
				L_f_after = helpers_calculate_loudness_pressure(data_point_notes[note][1], L_N);
			}

			// Uses a log-linear interpolation with the data points to approximate the loudness of the frequency
			double loudness = ((L_f_after - L_f_before) / (log_after - log_before)) * (log_note_freq - log_before) + L_f_before;
			// Converts the loudness to volts
			s_loudnesses[level][note] = helpers_convert_phons_to_volts(loudness);
		}
	}
}