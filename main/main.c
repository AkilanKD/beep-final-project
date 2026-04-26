#include <stdint.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/dac_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "helpers.h"

// Audio output sample rate (samples generated per second).
#define SAMPLE_RATE_HZ      22050
// Number of samples rendered per loop before sending to DAC DMA.
#define AUDIO_BUFFER_SAMPLES 256
// Size of the sine wave lookup table (power-of-two for fast wrapping).
#define LUT_SIZE            256
// Fixed divisor used to normalize the mixed output for up to 4 notes.
#define MAX_MIX_POLYPHONY   4.0f
// Debounce window for button interrupts, in microseconds.
#define DEBOUNCE_US         20000

// DAC continuous mode DMA descriptor count and DMA buffer size.
#define DAC_DMA_DESC_NUM    6
#define DAC_DMA_BUF_SIZE    512

// ---------------------------
// Synth Runtime State
// ---------------------------
// `TAG` labels log output so messages are easy to filter in ESP-IDF monitor.
static const char *TAG = "poly_synth";

// Handle for DAC continuous output driver.
static dac_continuous_handle_t dac_handle;
// Handle for ADC one-shot reads (volume control).
static adc_oneshot_unit_handle_t adc_handle;

// Pointers to immutable board note mappings from helpers.c.
static const gpio_num_t *note_pins;
static float note_freqs[NOTE_COUNT];

// Per-note mutable state used by ISR and audio render loop.
static volatile bool note_active[NOTE_COUNT];
static volatile int64_t last_isr_time_us[NOTE_COUNT];
// Stores unpressed level sampled at boot (supports active-low/high wiring).
static uint8_t note_idle_level[NOTE_COUNT];
// Shared waveform table and per-note DDS phase state.
static float sine_lut[LUT_SIZE];
// phase_acc stores "where we are" in each note's sine cycle.
static float phase_acc[NOTE_COUNT];
// phase_step stores "how much to move" in the cycle per audio sample.
static float phase_step[NOTE_COUNT];
// Octave shift done on the piano
static volatile int octave_shift;
static volatile int64_t octave_up_last_isr_time_us;
static volatile int64_t octave_down_last_isr_time_us;

/*
 * Returns true when the note input level differs from its startup idle level.
 */
static inline bool is_note_pressed_level(int idx, int level)
{
    // A note is considered pressed when the pin differs from its boot-time idle level.
    return level != note_idle_level[idx];
}

// ---------------------------
// Input ISR
// ---------------------------
/*
 * GPIO ISR for note buttons.
 *
 * Steps:
 * 1) Decode which note pin fired.
 * 2) Ignore bounce edges inside DEBOUNCE_US.
 * 3) Update note_active from the latest pin level.
 */
static void IRAM_ATTR note_gpio_isr(void *arg)
{
    // Timestamp for debounce checks.
    const int64_t now_us = esp_timer_get_time();
    // ISR arg is the GPIO pin number cast through void*.
    const int pin = (int)(intptr_t)arg;
    // Convert pin number to note index in runtime arrays.
    const int idx = helpers_note_index_from_pin(pin);
    if (idx < 0) {
        return;
    }

    // Ignore rapid toggles from mechanical switch bounce.
    if ((now_us - last_isr_time_us[idx]) < DEBOUNCE_US) {
        return;
    }
    last_isr_time_us[idx] = now_us;

    // Update key state immediately so audio task sees changes with low latency.
    note_active[idx + OCTAVE_NOTE_COUNT * (octave_shift - MIN_OCTAVE_SHIFT)] = is_note_pressed_level(idx, gpio_get_level(pin));
}

/*
 * GPIO ISR for for octave up button
 */
static void octave_up_isr(void *arg) {
    const int64_t now_us = esp_timer_get_time();
    if ((now_us - octave_up_last_isr_time_us) < DEBOUNCE_US) {
        return;
    }
    octave_up_last_isr_time_us = now_us;
    
    if (octave_shift < MAX_OCTAVE_SHIFT) {
        octave_shift++;
    }
    else {
        octave_shift = MAX_OCTAVE_SHIFT;
    }
}

/*
 * GPIO ISR for for octave down button
 */
static void octave_down_isr(void *arg) {
    const int64_t now_us = esp_timer_get_time();
    if ((now_us - octave_down_last_isr_time_us) < DEBOUNCE_US) {
        return;
    }
    octave_down_last_isr_time_us = now_us;

    if (octave_shift > MIN_OCTAVE_SHIFT) {
        octave_shift--;
    }
    else {
        octave_shift = MIN_OCTAVE_SHIFT;
    }
}

// ---------------------------
// Initialization
// ---------------------------

/*
 * Initializes per-note DDS phase accumulators and phase increments.
 * phase_step = freq * LUT_SIZE / sample_rate
 */
static void init_phase_steps(void)
{
    // Initialize every note oscillator to a known state before playback starts.
    for (int i = 0; i < NOTE_COUNT; ++i) {
        // Start at beginning of waveform.
        phase_acc[i] = 0.0f;
        // Precompute increment once to avoid repeated math in audio loop.
        phase_step[i] = (note_freqs[i] * (float)LUT_SIZE) / (float)SAMPLE_RATE_HZ;
        note_active[i] = false;
        last_isr_time_us[i] = 0;
        note_idle_level[i] = 1;
    }
}

/*
 * Configures note GPIO inputs and installs note edge interrupts.
 */
static void init_note_gpio(void)
{
    // Every note pin is configured as active-low button input with edge interrupts.
    const gpio_config_t input_pin_cfg = {
        .pin_bit_mask = (1ULL << C_PIN) |
                        (1ULL << C_SHARP_PIN) |
                        (1ULL << D_PIN) |
                        (1ULL << D_SHARP_PIN) |
                        (1ULL << E_PIN) |
                        (1ULL << F_PIN) |
                        (1ULL << F_SHARP_PIN) |
                        (1ULL << G_PIN) |
                        (1ULL << G_SHARP_PIN) |
                        (1ULL << A_PIN) |
                        (1ULL << A_SHARP_PIN) |
                        (1ULL << B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    // Apply GPIO config to all note pins in one call.
    ESP_ERROR_CHECK(gpio_config(&input_pin_cfg));
    // Enable global GPIO ISR service before registering per-pin handlers.
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    for (int i = 0; i < OCTAVE_NOTE_COUNT; ++i) {
        // Register one ISR callback per note pin.
        ESP_ERROR_CHECK(gpio_isr_handler_add(note_pins[i], note_gpio_isr, (void *)(intptr_t)note_pins[i]));
        // Snapshot each key's idle level at startup.
        note_idle_level[i] = (uint8_t) gpio_get_level(note_pins[i]);
        // Start with all notes released and debounce timer cleared.
        note_active[i] = false;
        last_isr_time_us[i] = 0;
    }
}

/*
 *
 */
static void init_octave_gpio(void)
{
    const gpio_config_t octave_pin_cfg = {
        .pin_bit_mask = (1ULL << OCTAVE_UP_PIN) | 
                        (1ULL << OCTAVE_DOWN_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&octave_pin_cfg));
//    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_EDGE));
//    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(OCTAVE_UP_PIN, octave_up_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(OCTAVE_DOWN_PIN, octave_down_isr, NULL));

    octave_up_last_isr_time_us = 0;
    octave_down_last_isr_time_us = 0;

    // Initalizes the octave shift to be zero
    octave_shift = (MIN_OCTAVE_SHIFT + MAX_OCTAVE_SHIFT) / 2;
}

/*
 * Initializes ADC channel used as volume knob input.
 */
static void init_volume_adc(void)
{
    // ADC1 one-shot mode: explicit read each time in audio task.
    adc_oneshot_unit_init_cfg_t adc_init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_cfg, &adc_handle));

    // 12-bit conversion and 12 dB attenuation for broader voltage range.
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg));
}

/*
 * Initializes DAC in continuous DMA mode.
 * Why this matters: DMA keeps audio output smooth without CPU writing each sample directly.
 */
static void init_dac_tx(void)
{
    // ESP-IDF v5+ internal DAC path: continuous DMA output on DAC channel 0 (GPIO25).
    dac_continuous_config_t dac_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,
        .desc_num = DAC_DMA_DESC_NUM,
        .buf_size = DAC_DMA_BUF_SIZE,
        .freq_hz = SAMPLE_RATE_HZ,
        .offset = 0,
        .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode = DAC_CHANNEL_MODE_ALTER,
    };

    ESP_ERROR_CHECK(dac_continuous_new_channels(&dac_cfg, &dac_handle));
    // Start DAC DMA stream.
    ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
}

// ---------------------------
// Audio Engine
// ---------------------------
static void audio_task(void *arg)
{
    (void)arg;

    // Byte-oriented output buffer expected by internal DAC driver.
    uint8_t audio_buf[AUDIO_BUFFER_SAMPLES];
    // Tracks last log print time to limit UART spam.
    int64_t last_debug_log_us = 0;

    while (1) {
        // Keep note state synced even if an interrupt edge was missed.
        // Fallback state refresh in case interrupts are missed/noisy on a particular board.
        for (int i = 0; i < OCTAVE_NOTE_COUNT; ++i) {
            note_active[i] = is_note_pressed_level(i, gpio_get_level(note_pins[i]));
        }

        // Poll volume once per audio buffer to keep control responsive with low overhead.
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &raw));
        // Convert raw 12-bit value [0..4095] to gain [0.0..1.0].
        const float volume = (float)raw / 4095.0f;
        // Captured for debug printing only.
        int active_notes_snapshot = 0;

        for (int n = 0; n < AUDIO_BUFFER_SAMPLES; ++n) {
            // Mixed signal for one output sample.
            float mix = 0.0f;
            // Number of notes contributing to this sample.
            int active_count = 0;

            for (int i = 0; i < OCTAVE_NOTE_COUNT; ++i) {
                if (!note_active[i]) {
                    continue;
                }

                // Finds note index, adjusted for octave
                int idx = i + OCTAVE_NOTE_COUNT * (octave_shift - MIN_OCTAVE_SHIFT);

                // Read current waveform sample from LUT.
                int lut_idx = (int)phase_acc[idx] & (LUT_SIZE - 1);
                mix += sine_lut[lut_idx];
                active_count++;

                // Advance oscillator phase for next sample.
                phase_acc[idx] += phase_step[idx];
                if (phase_acc[idx] >= (float)LUT_SIZE) {
                    // Wrap phase when it crosses one full cycle.
                    phase_acc[idx] -= (float)LUT_SIZE;
                }
            }

            if (active_count > 0) {
                // Fixed polyphony normalization avoids audible volume pumping.
                // Beginner note: this keeps loudness more consistent as more keys are held.
                mix = (mix / MAX_MIX_POLYPHONY) * volume;
            } else {
                // Output silence if no notes are active.
                mix = 0.0f;
            }

            if (n == 0) {
                // Store note count from first sample as a cheap per-buffer snapshot.
                active_notes_snapshot = active_count;
            }

            // Constrain normalized sample to safe range before DAC conversion.
            if (mix > 1.0f) {
                mix = 1.0f;
            } else if (mix < -1.0f) {
                mix = -1.0f;
            }

            // Map normalized float [-1,1] to DAC byte [0,255].
            audio_buf[n] = helpers_mix_to_dac_u8(mix);
        }

        // Push generated buffer to DAC DMA; block until accepted.
        size_t bytes_loaded = 0;
        ESP_ERROR_CHECK(dac_continuous_write(dac_handle,
                                             audio_buf,
                                             sizeof(audio_buf),
                                             &bytes_loaded,
                                             -1));

        // Optional sanity check left implicit: bytes_loaded should match buffer size.

        // Print lightweight status at 5 Hz.
        const int64_t now_us = esp_timer_get_time();
        if ((now_us - last_debug_log_us) >= 200000) {
            ESP_LOGI(TAG,
                     "active_notes=%d volume=%.2f raw=%d, octave = %d",
                     active_notes_snapshot,
                     (double)volume,
                     raw,
                     octave_shift
                    );
            last_debug_log_us = now_us;
        }
    }
}

// ---------------------------
// Entry Point
// ---------------------------
void app_main(void)
{
    // Acquire static note metadata used by setup and audio rendering.
    note_pins = helpers_get_note_pins();
    helpers_populate_note_freqs(note_freqs);

    // Build waveform table once at startup; runtime only does table lookup.
    helpers_init_sine_lut(sine_lut, LUT_SIZE);
    init_phase_steps();
    init_note_gpio();
    init_octave_gpio();
    init_volume_adc();
    init_dac_tx();

    // One startup log line to confirm runtime configuration.
    ESP_LOGI(TAG, "Poly synth started at %d Hz (DAC DMA, debounce=%d us), octave = %d", SAMPLE_RATE_HZ, DEBOUNCE_US, octave_shift);

    for (int i = 0; i < NOTE_COUNT; i++) {
        ESP_LOGI(TAG, "Note %d freq is %f", i, note_freqs[i]);
    }

    // Run render task on core 1 near top priority for stable audio timing.
    xTaskCreatePinnedToCore(audio_task,
                            "audio_task",
                            4096,
                            NULL,
                            configMAX_PRIORITIES - 2,
                            NULL,
                            1);
}