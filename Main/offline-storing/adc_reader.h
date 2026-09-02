#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

typedef struct {
    float voltage_v;
    float current_a;
    float power_w;
} sensor_data_t;

esp_err_t adc_reader_init(void);
void adc_reader_start(QueueHandle_t queue);