#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stdint.h> // Added for int64_t

typedef struct {
    int64_t timestamp_ms; // <-- NEW: Stores epoch time in milliseconds
    float voltage_v;
    float current_a;
    float power_w;
} sensor_data_t;

esp_err_t adc_reader_init(void);
void adc_reader_start(QueueHandle_t queue);