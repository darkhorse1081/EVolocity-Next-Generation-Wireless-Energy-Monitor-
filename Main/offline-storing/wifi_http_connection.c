#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_sleep.h"

#include "adc_reader.h"

#define WIFI_SSID               "RajPhone"
#define WIFI_PASS               "Capstone5"
#define SERVER_URL              "http://172.20.10.2:3000/api/readings"
#define HTTP_SEND_INTERVAL_MS   100

// --- OFFLINE BUFFER CONFIGURATION ---
// 5 minutes at 10Hz = 3000 samples
#define OFFLINE_BUFFER_SIZE     3000

static const char *TAG = "WIFI_HTTP_APP";
static volatile bool wifi_is_connected = false;
static QueueHandle_t adc_queue = NULL;

// Circular buffer for offline storage
static sensor_data_t offline_buffer[OFFLINE_BUFFER_SIZE];
static int buffer_head = 0;   // where next sample gets written
static int buffer_tail = 0;   // where next sample gets read
static int buffer_count = 0;  // how many samples are stored

static void buffer_push(sensor_data_t *data)
{
    offline_buffer[buffer_head] = *data;
    buffer_head = (buffer_head + 1) % OFFLINE_BUFFER_SIZE;

    if (buffer_count < OFFLINE_BUFFER_SIZE) {
        buffer_count++;
    } else {
        // Buffer full — overwrite oldest sample by advancing tail
        buffer_tail = (buffer_tail + 1) % OFFLINE_BUFFER_SIZE;
    }
}

static bool buffer_pop(sensor_data_t *data)
{
    if (buffer_count == 0) return false;

    *data = offline_buffer[buffer_tail];
    buffer_tail = (buffer_tail + 1) % OFFLINE_BUFFER_SIZE;
    buffer_count--;
    return true;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_is_connected = false;
        ESP_LOGI(TAG, "Wi-Fi disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_is_connected = true;
    }
}

static void wifi_init_sta(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static bool send_reading(esp_http_client_handle_t client, sensor_data_t *data)
{
    char post_data[200];
    snprintf(post_data, sizeof(post_data),
             "{\"deviceId\":\"esp32-02\",\"voltage\":%.2f,\"current\":%.2f,\"power\":%.2f}",
             data->voltage_v, data->current_a, data->power_w);

    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sent: %s | Status: %d",
                 post_data, esp_http_client_get_status_code(client));
        return true;
    } else {
        ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
        return false;
    }
}

static void http_post_task(void *pvParameters)
{
    sensor_data_t latest = {0};
    TickType_t last_wake_time = xTaskGetTickCount();

    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
        .skip_cert_common_name_check = true,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        vTaskDelete(NULL);
        return;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");

    while (1) {
        if (xQueueReceive(adc_queue, &latest, 0) == pdTRUE) {
            if (wifi_is_connected) {
                // First flush any buffered offline data
                if (buffer_count > 0) {
                    ESP_LOGI(TAG, "Wi-Fi reconnected — flushing %d buffered samples...",
                             buffer_count);
                    sensor_data_t buffered;
                    while (buffer_pop(&buffered)) {
                        if (!send_reading(client, &buffered)) {
                            // Send failed mid-flush, push back and stop flushing
                            buffer_push(&buffered);
                            break;
                        }
                        // Small delay between buffered sends to avoid flooding server
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                }

                // Send the live reading
                send_reading(client, &latest);

            } else {
                // No Wi-Fi — store in offline buffer
                buffer_push(&latest);
                ESP_LOGW(TAG, "Wi-Fi offline — buffering sample (%d/%d stored)",
                         buffer_count, OFFLINE_BUFFER_SIZE);
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(HTTP_SEND_INTERVAL_MS));
    }
}

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        ESP_LOGI(TAG, "Woke up from ULP due to racing activity!");
    } else {
        ESP_LOGI(TAG, "Normal boot (cause: %d)", cause);
    }

    ESP_LOGI(TAG, "Starting app...");

    adc_queue = xQueueCreate(1, sizeof(sensor_data_t));
    if (!adc_queue) {
        ESP_LOGE(TAG, "Failed to create ADC queue");
        return;
    }

    wifi_init_sta();
    adc_reader_init();
    adc_reader_start(adc_queue);

    xTaskCreate(http_post_task, "http_post_task", 8192, NULL, 5, NULL);
}