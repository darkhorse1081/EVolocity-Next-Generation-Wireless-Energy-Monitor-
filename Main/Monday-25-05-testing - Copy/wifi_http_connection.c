#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
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
#include "esp_sntp.h"
#include "adc_reader.h"
#include "timestamp.h"

extern portMUX_TYPE queue_mux;

#define WIFI_SSID               "RajPhone"
#define WIFI_PASS               "Capstone5"
#define SERVER_URL              "http://172.20.10.3:3000/api/readings"
#define HTTP_SEND_INTERVAL_MS   100

static const char *TAG = "WIFI_HTTP_APP";
static volatile bool wifi_is_connected = false;
static volatile bool time_is_synced = false;
static QueueHandle_t adc_queue = NULL;

// --- DYNAMIC OFFLINE BUFFER ---
#define OFFLINE_BUFFER_SIZE     3000

typedef struct {
    sensor_data_t data;
    time_t ts_sec;
    uint16_t ts_ms;
} buffered_sample_t;

// Allocated dynamically on heap during app_main to prevent .dram0.bss static overflow
static buffered_sample_t *offline_buffer = NULL;
static int buffer_head  = 0;
static int buffer_tail  = 0;
static int buffer_count = 0;

// --- CIRCULAR BUFFER ---
static void buffer_push(sensor_data_t *data)
{
    if (offline_buffer == NULL) return;

    struct timeval tv;
    gettimeofday(&tv, NULL);

    offline_buffer[buffer_head].data = *data;
    offline_buffer[buffer_head].ts_sec = tv.tv_sec;
    offline_buffer[buffer_head].ts_ms = tv.tv_usec / 1000;

    buffer_head = (buffer_head + 1) % OFFLINE_BUFFER_SIZE;

    if (buffer_count < OFFLINE_BUFFER_SIZE) {
        buffer_count++;
    } else {
        buffer_tail = (buffer_tail + 1) % OFFLINE_BUFFER_SIZE;
    }
}

static bool buffer_pop(buffered_sample_t *sample)
{
    if (buffer_count == 0 || offline_buffer == NULL) return false;

    *sample = offline_buffer[buffer_tail];
    buffer_tail = (buffer_tail + 1) % OFFLINE_BUFFER_SIZE;
    buffer_count--;
    return true;
}

// --- SNTP ---
static void sntp_sync_callback(struct timeval *tv)
{
    time_is_synced = true;
    char buf[36];
    get_timestamp(buf, sizeof(buf));
    ESP_LOGI(TAG, "Time synced: %s", buf);
}

static void sntp_init_and_sync(void)
{
    if (esp_sntp_enabled()) return;

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(sntp_sync_callback);
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP started, waiting for time sync...");
}

// --- WIFI ---
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
        sntp_init_and_sync();
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
            .listen_interval = 3,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // --- Wi-Fi POWER OPTIMIZATIONS TO COOL BOARD AND REDUCE NOISE ---
    
    // 1. Set Wi-Fi TX Power limit to 8 dBm (Value is 32 * 0.25 dBm increments)
    // This stops massive 300mA current spikes from corrupting ADC readings
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(32));
    ESP_LOGI(TAG, "Wi-Fi TX Power restricted to 8 dBm to stabilize ADC");

    // 2. Enable Wi-Fi Modem Sleep 
    // Radio powers off automatically in milliseconds between packets
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_LOGI(TAG, "Wi-Fi Modem Sleep enabled to cool down board");
}

// --- HTTP SEND ---
// Includes 'is_buffered' tag to distinguish live vs offline data [5, 6]
static bool send_reading(esp_http_client_handle_t client,
                         sensor_data_t *data, const char *timestamp, bool is_buffered)
{
    char post_data[256];
    snprintf(post_data, sizeof(post_data),
             "{\"deviceId\":\"esp32-02\",\"voltage\":%.2f,\"current\":%.2f,"
             "\"power\":%.2f,\"timestamp\":\"%s\",\"buffered\":%s}",
             data->voltage_v, data->current_a, data->power_w, timestamp,
             is_buffered ? "true" : "false"); // The buffered flag for the software team! [5, 6]

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

// --- HTTP TASK ---
static void http_post_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
        .skip_cert_common_name_check = true,
        .keep_alive_enable = true,
        .timeout_ms = 2000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        vTaskDelete(NULL);
        return;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");

    bool was_disconnected = false;

    while (1) {
        // Atomic queue read
        sensor_data_t local_copy;
        portENTER_CRITICAL(&queue_mux);
        bool has_data = (xQueueReceive(adc_queue, &local_copy, 0) == pdTRUE);
        portEXIT_CRITICAL(&queue_mux);

        if (has_data) {
            if (wifi_is_connected) {
                // Only flush ONCE per reconnect — guard with edge detection
                if (was_disconnected && buffer_count > 0) {
                    int initial_count = buffer_count;
                    ESP_LOGI(TAG, "Wi-Fi reconnected — flushing %d buffered samples...",
                             initial_count);

                    esp_http_client_cleanup(client);
                    client = esp_http_client_init(&config);
                    esp_http_client_set_header(client, "Content-Type", "application/json");

                    buffered_sample_t buffered;
                    int flush_count = 0;
                    int fail_count = 0;
                    const int MAX_RETRIES = 3;

                    while (buffer_pop(&buffered)) {
                        // Reconstruct timestamp from epoch seconds
                        char ts_str[36];
                        struct tm timeinfo;
                        localtime_r(&buffered.ts_sec, &timeinfo);
                        char base[24];
                        strftime(base, sizeof(base), "%Y-%m-%dT%H:%M:%S", &timeinfo);
                        snprintf(ts_str, sizeof(ts_str), "%s.%03d", base, buffered.ts_ms);

                        // Flushed data — is_buffered is set to true [7]
                        if (send_reading(client, &buffered.data, ts_str, true)) {
                            flush_count++;
                            fail_count = 0;
                            if (flush_count % 10 == 0 || buffer_count == 0) {
                                ESP_LOGI(TAG, "  Flush progress: %d/%d sent, %d remaining",
                                         flush_count, initial_count, buffer_count);
                            }
                        } else {
                            fail_count++;
                            if (fail_count >= MAX_RETRIES) {
                                ESP_LOGW(TAG, "Flush giving up after %d failures, "
                                         "%d samples flushed, %d remaining (dropped)",
                                         fail_count, flush_count, buffer_count);
                                buffer_head = buffer_tail = buffer_count = 0;
                                break;
                            }
                            if (buffer_count < OFFLINE_BUFFER_SIZE) {
                                buffer_tail = (buffer_tail - 1 + OFFLINE_BUFFER_SIZE) % OFFLINE_BUFFER_SIZE;
                                offline_buffer[buffer_tail] = buffered;
                                buffer_count++;
                            }
                            vTaskDelay(pdMS_TO_TICKS(500));
                            continue;
                        }
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    ESP_LOGI(TAG, "Buffer flush complete — %d/%d samples sent successfully",
                             flush_count, initial_count);
                }
                was_disconnected = false;

                // Get timestamp and discard if not yet synced
                char ts[36];
                get_timestamp(ts, sizeof(ts));

                if (strcmp(ts, "unknown") != 0) {
                    // Live data — is_buffered is set to false [8]
                    send_reading(client, &local_copy, ts, false);
                } else {
                    ESP_LOGW(TAG, "Time not synced — discarding sample");
                }

            } else {
                was_disconnected = true;
                buffer_push(&local_copy);
                ESP_LOGW(TAG, "Wi-Fi offline — buffering sample (%d/%d stored)",
                         buffer_count, OFFLINE_BUFFER_SIZE);
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(HTTP_SEND_INTERVAL_MS));
    }
}

void app_main(void)
{
    // Set timezone to NZST/NZDT (UTC+12, UTC+13 in summer)
    setenv("TZ", "NZST-12NZDT,M9.5.0,M4.1.0/3", 1);
    tzset();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        ESP_LOGI(TAG, "Woke up from ULP due to racing activity!");
    } else {
        ESP_LOGI(TAG, "Normal boot (cause: %d)", cause);
    }

    // Allocate memory on the heap dynamically to prevent static DRAM linker overflow
    offline_buffer = (buffered_sample_t *)malloc(OFFLINE_BUFFER_SIZE * sizeof(buffered_sample_t));
    if (offline_buffer == NULL) {
        ESP_LOGE(TAG, "FAILED TO ALLOCATE OFFLINE BUFFER ON THE HEAP!");
        return;
    }

    ESP_LOGI(TAG, "Starting app...");

    adc_queue = xQueueCreate(1, sizeof(sensor_data_t));
    if (!adc_queue) {
        ESP_LOGE(TAG, "Failed to create ADC queue");
        free(offline_buffer);
        return;
    }

    wifi_init_sta();
    adc_reader_init();
    adc_reader_start(adc_queue);

    xTaskCreate(http_post_task, "http_post_task", 8192, NULL, 5, NULL);
}