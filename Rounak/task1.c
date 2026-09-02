#include <stdio.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// GPIO34 is ADC1 Channel 6
#define ADC_CHANNEL ADC_CHANNEL_6 
#define SAMPLE_COUNT 64         // change to experiment with averaging
#define SAMPLE_INTERVAL_MS 10   // 10 ms -> ~100 Hz sampling

void app_main(void)
{
    // 1. Initialize the ADC Unit
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    // 2. Configure the Channel (12-bit width, 12dB attenuation for ~0-3.3V range)
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12, // use DB_12 instead of deprecated DB_11
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &chan_cfg));

    // 3. Setup Calibration (Line Fitting Scheme for ESP32)
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
        // default_vref is optional; set if you want to override the default reference (mV)
        .default_vref = 1100, // typical Vref in mV (driver may use EFUSE if available)
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle);
    if (ret == ESP_OK) {
        printf("ADC calibration (line fitting) enabled.\n");
    } else {
        cali_handle = NULL;
        printf("Warning: ADC calibration (line fitting) not available (err=%d). Falling back to approximate conversion.\n", ret);
    }

    while (1) {
        int raw_val = 0;
        int64_t sum_raw = 0;
        int min_raw = INT_MAX, max_raw = INT_MIN;
        int voltage_mv = 0;

        // Take SAMPLE_COUNT samples spaced at ~100 Hz
        for (int i = 0; i < SAMPLE_COUNT; ++i) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &raw_val));
            sum_raw += raw_val;
            if (raw_val < min_raw) min_raw = raw_val;
            if (raw_val > max_raw) max_raw = raw_val;

            vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
        }

        int avg_raw = (int)(sum_raw / SAMPLE_COUNT);

        if (cali_handle) {
            // Convert raw average to calibrated voltage (mV)
            esp_err_t r = adc_cali_raw_to_voltage(cali_handle, avg_raw, &voltage_mv);
            if (r != ESP_OK) {
                printf("adc_cali_raw_to_voltage failed: %d\n", r);
                voltage_mv = (int)((3300LL * avg_raw) / 4095); // fallback approx
            }
        } else {
            // Approximate conversion (not calibrated)
            voltage_mv = (int)((3300LL * avg_raw) / 4095);
        }

        printf("Avg raw: %4d | Min raw: %4d | Max raw: %4d | Voltage: %d mV\n",
               avg_raw, min_raw, max_raw, voltage_mv);
    }
}