#include <stdio.h>
#include <sys/time.h>
#include "adc_reader.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_sleep.h"
#include "ulp.h"
#include "ulp_fsm_common.h"
#include "ulp_main.h"
#include "soc/sens_reg.h"
#include "soc/sens_struct.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"

// --- HARDWARE CONFIGURATION ---
#define ADC_CHANNEL_V               ADC_CHANNEL_6   // GPIO34 - voltage
#define ADC_CHANNEL_I               ADC_CHANNEL_7   // GPIO35 - current

// --- TIMING CONFIGURATION ---
#define SAMPLE_INTERVAL_MS          10              // 10ms = 100Hz sampling
#define SAMPLES_TO_AVERAGE          10              // 10 samples = 10Hz output

// --- SLEEP CONFIGURATION ---
#define ACTIVITY_CURRENT_THRESHOLD  0.5f            // Amps — below this = no activity
#define INACTIVITY_DURATION_MS      30000           // 30 seconds of inactivity before sleep
#define ULP_WAKEUP_THRESHOLD_RAW    50           // Raw ADC value on GPIO35 to trigger wakeup

// --- CALIBRATION CONSTANTS ---
#define V_ZERO_MV       142.0f                  // ADC reading at 0V input
#define V_REF_MV        892.0f                  // ADC reading at known reference voltage
#define V_REF_REAL      18.2f                   // actual voltage at reference point
#define V_MULTIPLIER    (V_REF_REAL / (V_REF_MV - V_ZERO_MV))
#define V_OFFSET        (-V_ZERO_MV * V_MULTIPLIER)
#define I_MULTIPLIER    (1.0f / 376.0f)          // 376mV per amp measured from your sensor
#define I_OFFSET        (-142.0f / 376.0f)        // zero point at 142mV

static const char *TAG = "ADC_READER";

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;
static QueueHandle_t s_queue = NULL;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void prepare_adc1_for_ulp(void)
{
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG,
                      SENS_FORCE_XPD_SAR, SENS_FORCE_XPD_SAR_PU, SENS_FORCE_XPD_SAR_S);
    CLEAR_PERI_REG_MASK(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DIG_FORCE);
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_FORCE);
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD_FORCE);

    // Set attenuation for current channel (GPIO35) to DB_12
    SET_PERI_REG_BITS(SENS_SAR_ATTEN1_REG, 0x3, 3, ADC_CHANNEL_I * 2);

    SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG,
                      SENS_SAR1_BIT_WIDTH, 3, SENS_SAR1_BIT_WIDTH_S);
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG,
                      SENS_SAR1_SAMPLE_BIT, 3, SENS_SAR1_SAMPLE_BIT_S);
    CLEAR_PERI_REG_MASK(RTC_IO_HALL_SENS_REG, RTC_IO_XPD_HALL);

    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG,
                      SENS_FORCE_XPD_AMP, SENS_FORCE_XPD_AMP_PD, SENS_FORCE_XPD_AMP_S);
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_CTRL_REG, SENS_AMP_RST_FB_FSM);
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_CTRL_REG, SENS_AMP_SHORT_REF_FSM);
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_CTRL_REG, SENS_AMP_SHORT_REF_GND_FSM);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG,
                      SENS_SAR_AMP_WAIT1, 1, SENS_SAR_AMP_WAIT1_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG,
                      SENS_SAR_AMP_WAIT2, 1, SENS_SAR_AMP_WAIT2_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG,
                      SENS_SAR_AMP_WAIT3, 1, SENS_SAR_AMP_WAIT3_S);
}

static void configure_and_start_ulp(void)
{
    ESP_LOGW(TAG, "No racing activity detected for 30s. Entering ULP deep sleep...");

    if (adc1_handle != NULL) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
    prepare_adc1_for_ulp();

    size_t program_size = (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t);
    ESP_ERROR_CHECK(ulp_load_binary(0, ulp_main_bin_start, program_size));

    ulp_high_thr = ULP_WAKEUP_THRESHOLD_RAW;

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,   ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);

    ulp_set_wakeup_period(0, 100000);
    ESP_ERROR_CHECK(ulp_run((uint32_t)&ulp_entry - (uint32_t)RTC_SLOW_MEM));

    esp_sleep_enable_ulp_wakeup();
    SET_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
    esp_deep_sleep_start();
}

static void adc_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t inactive_timer_ms = 0;

    int sum_mv_v = 0, sum_mv_i = 0;
    int sample_count = 0;

    while (1) {
        int raw_v = 0, raw_i = 0;
        int mv_v = 0, mv_i = 0;

        adc_oneshot_read(adc1_handle, ADC_CHANNEL_V, &raw_v);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_I, &raw_i);

        if (cali_handle) {
            adc_cali_raw_to_voltage(cali_handle, raw_v, &mv_v);
            adc_cali_raw_to_voltage(cali_handle, raw_i, &mv_i);
        } else {
            mv_v = (int)((3300LL * raw_v) / 4095);
            mv_i = (int)((3300LL * raw_i) / 4095);
        }

        sum_mv_v += mv_v;
        sum_mv_i += mv_i;
        sample_count++;

        if (sample_count >= SAMPLES_TO_AVERAGE) {
            float avg_mv_v = (float)sum_mv_v / SAMPLES_TO_AVERAGE;
            float avg_mv_i = (float)sum_mv_i / SAMPLES_TO_AVERAGE;

            float final_voltage = (avg_mv_v - V_ZERO_MV) * V_MULTIPLIER;
            float final_current = (avg_mv_i * I_MULTIPLIER) + I_OFFSET;
            float final_power   = final_voltage * final_current;

            if (avg_mv_v > V_ZERO_MV + 50.0f) {
                final_voltage = (final_voltage * 0.8205f) + 3.19f;
            } else {
                final_voltage = 0.0f;
            }

            // Clamp small noise around zero
            if (final_current < 0.1f && final_current > -0.1f) final_current = 0.0f;
            if (final_voltage < 0.5f) final_voltage = 0.0f;
            if (final_power < 0.0f)   final_power   = 0.0f;

            sensor_data_t reading = {
            // --- NEW: Get precise time ---
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            // Convert seconds and microseconds to total milliseconds
            int64_t time_ms = (int64_t)tv_now.tv_sec * 1000LL + (int64_t)tv_now.tv_usec / 1000LL;

            sensor_data_t reading = {
                .timestamp_ms = time_ms, // <-- Add timestamp here
                .voltage_v = final_voltage,
                .current_a = final_current,
                .power_w   = final_power
            };
            xQueueOverwrite(s_queue, &reading);

            ESP_LOGI(TAG, "V: %.2f V | I: %.2f A | P: %.2f W",
                     final_voltage, final_current, final_power);
            ESP_LOGI(TAG, "raw_v=%d raw_i=%d mv_v=%d mv_i=%d", raw_v, raw_i, mv_v, mv_i);

            // Inactivity check based on current
            if (final_current < ACTIVITY_CURRENT_THRESHOLD) {
                inactive_timer_ms += (SAMPLE_INTERVAL_MS * SAMPLES_TO_AVERAGE);
                if (inactive_timer_ms >= INACTIVITY_DURATION_MS) {
                    configure_and_start_ulp();
                }
            } else {
                inactive_timer_ms = 0;
            }

            sum_mv_v = 0;
            sum_mv_i = 0;
            sample_count = 0;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

esp_err_t adc_reader_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_V, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_I, &chan_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
        .default_vref = 1100,
    };

    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        cali_handle = NULL;
        ESP_LOGW(TAG, "ADC calibration unavailable");
    }

    ESP_LOGI(TAG, "ADC reader initialized successfully");
    return ESP_OK;
}

void adc_reader_start(QueueHandle_t queue)
{
    s_queue = queue;
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);
}