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

#define WIFI_SSID               "MKXLII (iPhone 14)"
#define WIFI_PASS               "123456789"
#define SERVER_URL              "http://172.20.10.5:3000/api/readings"
#define HTTP_SEND_INTERVAL_MS   100 // 100ms = 10Hz required by the milestone

static const char *TAG = "WIFI_HTTP_APP";
static volatile bool wifi_is_connected = false;
static QueueHandle_t adc_queue = NULL;

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
        if (wifi_is_connected) {
            // Check if there is new data in the queue
            if (xQueueReceive(adc_queue, &latest, 0) == pdTRUE) {
                char post_data[200];

                // Construct JSON with Voltage, Current, and Power
                snprintf(post_data, sizeof(post_data),
                         "{\"deviceId\":\"esp32-02\",\"voltage\":%.2f,\"current\":%.2f,\"power\":%.2f}",
                         latest.voltage_v, latest.current_a, latest.power_w);

                esp_http_client_set_post_field(client, post_data, strlen(post_data));

                esp_err_t err = esp_http_client_perform(client);
                if (err == ESP_OK) {
                    // Comment this out later if you want a cleaner terminal
                    ESP_LOGI(TAG, "Sent: %s | Status: %d",
                             post_data, esp_http_client_get_status_code(client));
                } else {
                    ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
                }
            }
        }

        // Wait precisely 100ms (10Hz) before sending the next packet
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(HTTP_SEND_INTERVAL_MS));
    }
}

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        ESP_LOGI(TAG, "Woke up from ULP due to Racing Activity!");
    } else {
        ESP_LOGI(TAG, "Normal boot (cause: %d)", cause);
    }

    ESP_LOGI(TAG, "Starting app...");

    // Create the queue based on the new sensor_data_t struct
    adc_queue = xQueueCreate(1, sizeof(sensor_data_t));
    if (!adc_queue) {
        ESP_LOGE(TAG, "Failed to create ADC queue");
        return;
    }

    wifi_init_sta();
    adc_reader_init();
    adc_reader_start(adc_queue);

    // Start the HTTP Post task
    xTaskCreate(http_post_task, "http_post_task", 8192, NULL, 5, NULL);
}