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

#define SAMPLE_RATE_HZ      22050
#define AUDIO_BUFFER_SAMPLES 256
#define LUT_SIZE            256
#define MAX_MIX_POLYPHONY   4.0f
#define DEBOUNCE_US         20000

#define DAC_DMA_DESC_NUM    6
#define DAC_DMA_BUF_SIZE    512

// ---------------------------
// Synth Runtime State
// ---------------------------
static const char *TAG = "poly_synth";

static dac_continuous_handle_t dac_handle;
static adc_oneshot_unit_handle_t adc_handle;

static const gpio_num_t *note_pins;
static const float *note_freqs;

static volatile bool note_active[NOTE_COUNT];
static volatile int64_t last_isr_time_us[NOTE_COUNT];
static uint8_t note_idle_level[NOTE_COUNT];
static float sine_lut[LUT_SIZE];
static float phase_acc[NOTE_COUNT];
static float phase_step[NOTE_COUNT];

/*
 * Returns if a button note of the given index (idx) is pressed at certain level
 */
static inline bool is_note_pressed_level(int idx, int level)
{
    // A note is considered pressed when the pin differs from its boot-time idle level.
    return level != note_idle_level[idx];
}

// ---------------------------
// Input ISR
// ---------------------------
static void IRAM_ATTR note_gpio_isr(void *arg)
{
    const int64_t now_us = esp_timer_get_time();
    const int pin = (int)(intptr_t)arg;
    const int idx = helpers_note_index_from_pin(pin);
    if (idx < 0) {
        return;
    }

    // Ignore rapid toggles from mechanical switch bounce.
    if ((now_us - last_isr_time_us[idx]) < DEBOUNCE_US) {
        return;
    }
    last_isr_time_us[idx] = now_us;

    note_active[idx] = is_note_pressed_level(idx, gpio_get_level(pin));
}

// ---------------------------
// Initialization
// ---------------------------

/*
 * Initalizes the phase steps
 */
static void init_phase_steps(void)
{
    for (int i = 0; i < NOTE_COUNT; ++i) {
        phase_acc[i] = 0.0f;
        phase_step[i] = (note_freqs[i] * (float)LUT_SIZE) / (float)SAMPLE_RATE_HZ;
        note_active[i] = false;
        last_isr_time_us[i] = 0;
        note_idle_level[i] = 1;
    }
}

/*
 * Initializes the nodes for the 
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

    // Configures the keyboard button pins
    ESP_ERROR_CHECK(gpio_config(&input_pin_cfg));
    // Higher level processor enabler
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    for (int i = 0; i < NOTE_COUNT; ++i) {
        // 
        ESP_ERROR_CHECK(gpio_isr_handler_add(note_pins[i], note_gpio_isr, (void *)(intptr_t)note_pins[i]));
        note_idle_level[i] = (uint8_t) gpio_get_level(note_pins[i]);
        note_active[i] = false;
        last_isr_time_us[i] = 0;
    }
}

/*
 * 
 */
static void init_volume_adc(void)
{
    adc_oneshot_unit_init_cfg_t adc_init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg));
}

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
    ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
}

// ---------------------------
// Audio Engine
// ---------------------------
static void audio_task(void *arg)
{
    (void)arg;

    uint8_t audio_buf[AUDIO_BUFFER_SAMPLES];
    int64_t last_debug_log_us = 0;

    while (1) {
        // Fallback state refresh in case interrupts are missed/noisy on a particular board.
        for (int i = 0; i < NOTE_COUNT; ++i) {
            note_active[i] = is_note_pressed_level(i, gpio_get_level(note_pins[i]));
        }

        // Poll volume once per audio buffer to keep control responsive with low overhead.
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &raw));
        const float volume = (float)raw / 4095.0f;
        int active_notes_snapshot = 0;

        for (int n = 0; n < AUDIO_BUFFER_SAMPLES; ++n) {
            float mix = 0.0f;
            int active_count = 0;

            for (int i = 0; i < NOTE_COUNT; ++i) {
                if (!note_active[i]) {
                    continue;
                }

                int lut_idx = (int)phase_acc[i] & (LUT_SIZE - 1);
                mix += sine_lut[lut_idx];
                active_count++;

                phase_acc[i] += phase_step[i];
                if (phase_acc[i] >= (float)LUT_SIZE) {
                    phase_acc[i] -= (float)LUT_SIZE;
                }
            }

            if (active_count > 0) {
                // Fixed polyphony normalization avoids audible volume pumping.
                mix = (mix / MAX_MIX_POLYPHONY) * volume;
            } else {
                mix = 0.0f;
            }

            if (n == 0) {
                active_notes_snapshot = active_count;
            }

            if (mix > 1.0f) {
                mix = 1.0f;
            } else if (mix < -1.0f) {
                mix = -1.0f;
            }

            audio_buf[n] = helpers_mix_to_dac_u8(mix);
        }

        size_t bytes_loaded = 0;
        ESP_ERROR_CHECK(dac_continuous_write(dac_handle,
                                             audio_buf,
                                             sizeof(audio_buf),
                                             &bytes_loaded,
                                             -1));

        const int64_t now_us = esp_timer_get_time();
        if ((now_us - last_debug_log_us) >= 200000) {
            ESP_LOGI(TAG,
                     "active_notes=%d volume=%.2f raw=%d",
                     active_notes_snapshot,
                     (double)volume,
                     raw);
            last_debug_log_us = now_us;
        }
    }
}

// ---------------------------
// Entry Point
// ---------------------------
void app_main(void)
{
    note_pins = helpers_get_note_pins();
    note_freqs = helpers_get_note_freqs();

    // Build waveform table once at startup; runtime only does table lookup.
    helpers_init_sine_lut(sine_lut, LUT_SIZE);
    init_phase_steps();
    init_note_gpio();
    init_volume_adc();
    init_dac_tx();

    ESP_LOGI(TAG, "Poly synth started at %d Hz (DAC DMA, debounce=%d us)", SAMPLE_RATE_HZ, DEBOUNCE_US);

    xTaskCreatePinnedToCore(audio_task,
                            "audio_task",
                            4096,
                            NULL,
                            configMAX_PRIORITIES - 2,
                            NULL,
                            1);
}