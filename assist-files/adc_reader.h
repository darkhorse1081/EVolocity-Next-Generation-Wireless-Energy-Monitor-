#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

// The payload sent from the ADC task to the HTTP task
typedef struct {
    float voltage_v;
    float current_a;
    float power_w;
} sensor_data_t;

esp_err_t adc_reader_init(void);
void adc_reader_start(QueueHandle_t queue);