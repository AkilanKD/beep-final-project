#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/rmt_tx.h"  
#include "driver/gpio.h"
#include "helpers.h"

// Input pin config
gpio_config_t input_pin = {
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
    .intr_type = GPIO_INTR_NEGEDGE
};

void app_main(void)
{

}