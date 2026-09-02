#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include "adc_reader.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_sleep.h"
#include "driver/ledc.h" 
#include "ulp.h"
#include "ulp_fsm_common.h"
#include "ulp_main.h"
#include "soc/sens_reg.h"
#include "soc/sens_struct.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "timestamp.h"
#include "driver/gpio.h"

// --- HARDWARE CONFIGURATION ---
#define ADC_CHANNEL_V               ADC_CHANNEL_6 // GPIO34
#define ADC_CHANNEL_I_LOW           ADC_CHANNEL_4 // GPIO32 (Low Range)
#define ADC_CHANNEL_I_HIGH          ADC_CHANNEL_7 // GPIO35 (High Range)

// Passive Buzzer configuration on Pin 18
#define LEDC_TIMER                  LEDC_TIMER_0
#define LEDC_MODE                   LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL                LEDC_CHANNEL_0
#define LEDC_DUTY_RES               LEDC_TIMER_10_BIT 
#define LEDC_FREQUENCY              2400              

// --- TIMING CONFIGURATION ---
#define SAMPLE_INTERVAL_MS          10
#define SAMPLES_TO_AVERAGE          10

// --- SLEEP CONFIGURATION ---
#define ACTIVITY_CURRENT_THRESHOLD  0.5f
#define INACTIVITY_DURATION_MS      30000

// --- ULP SLEEP WAKEUP THRESHOLD ---
// At rest, your low-range sensor raw reading is exactly 577.
// We set the wakeup threshold to 600. Now, any small current driving the raw 
// reading above 600 will wake the board up instantly!
#define ULP_WAKEUP_THRESHOLD_RAW    635

#define BUZZER_CURRENT_THRESHOLD    33.0f

// --- CALIBRATION CONSTANTS (Teammate's math combined with your physical hardware!) ---

#define V_ZERO_MV           0.0f                // no offset on new PCB

#define I_LOW_ZERO_MV       615.0f   // measured from your logs at rest
#define I_HIGH_ZERO_MV      970.0f   // assuming same, verify from logs when connected

#define I_LOW_MULTIPLIER    0.0472f    // keep as teammate had it
#define I_HIGH_MULTIPLIER   0.01f  // keep as teammate had it
#define V_MULTIPLIER        0.0603f  // keep as teammate had it

#define I_LOW_RANGE_LIMIT   18.0f

static const char *TAG = "ADC_READER";

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;
static QueueHandle_t s_queue = NULL;
portMUX_TYPE queue_mux = portMUX_INITIALIZER_UNLOCKED;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

// --- PASSIVE BUZZER PWM METHODS ---
static void buzzer_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BUZZER_GPIO,      
        .duty           = 0, 
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

static void buzzer_on(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 512);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void buzzer_off(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void prepare_adc1_for_ulp(void)
{
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG,
                      SENS_FORCE_XPD_SAR, SENS_FORCE_XPD_SAR_PU, SENS_FORCE_XPD_SAR_S);
    CLEAR_PERI_REG_MASK(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DIG_FORCE);
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_FORCE);
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD_FORCE);

    // Watch the highly sensitive Low-Range channel (GPIO32 / Channel 4) for wakeup
    SET_PERI_REG_BITS(SENS_SAR_ATTEN1_REG, 0x3, 3, ADC_CHANNEL_I_LOW * 2);

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

    int sum_mv_v = 0, sum_mv_i_low = 0, sum_mv_i_high = 0;
    int sample_count = 0;

    int last_raw_v = 0, last_raw_i_low = 0, last_raw_i_high = 0;
    int last_mv_v = 0, last_mv_i_low = 0, last_mv_i_high = 0;

    while (1) {
        // Read Voltage, Low Range Current, and High Range Current
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_V, &last_raw_v);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_I_LOW, &last_raw_i_low);
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_I_HIGH, &last_raw_i_high);

        if (cali_handle) {
            adc_cali_raw_to_voltage(cali_handle, last_raw_v, &last_mv_v);
            adc_cali_raw_to_voltage(cali_handle, last_raw_i_low, &last_mv_i_low);
            adc_cali_raw_to_voltage(cali_handle, last_raw_i_high, &last_mv_i_high);
        } else {
            last_mv_v = (int)((3300LL * last_raw_v) / 4095);
            last_mv_i_low = (int)((3300LL * last_raw_i_low) / 4095);
            last_mv_i_high = (int)((3300LL * last_raw_i_high) / 4095);
        }

        sum_mv_v += last_mv_v;
        sum_mv_i_low += last_mv_i_low;
        sum_mv_i_high += last_mv_i_high;
        sample_count++;

        if (sample_count >= SAMPLES_TO_AVERAGE) {
            float avg_mv_v = (float)sum_mv_v / SAMPLES_TO_AVERAGE;
            float avg_mv_i_low = (float)sum_mv_i_low / SAMPLES_TO_AVERAGE;
            float avg_mv_i_high = (float)sum_mv_i_high / SAMPLES_TO_AVERAGE;

            // Voltage Calculation with deadband clamp to prevent ghost readings at 0V [11]
            float final_voltage = 0.0f;
            if (avg_mv_v > 150.0f) {
                final_voltage = avg_mv_v * V_MULTIPLIER;
            }

            // Current Calculations
            float final_current_low  = (avg_mv_i_low  - I_LOW_ZERO_MV) * I_LOW_MULTIPLIER;
            float final_current_high = (avg_mv_i_high - I_HIGH_ZERO_MV) * I_HIGH_MULTIPLIER;

            // Auto-Ranging Switching Logic [10, 11]
            float final_current = (final_current_low * 0.826f) + 0.067f;
            if (final_current > I_LOW_RANGE_LIMIT || final_current < -I_LOW_RANGE_LIMIT) {
                final_current = final_current_high;
            }

            // Noise Clamps
            if (final_current_low < 0.1f && final_current_low > -0.1f) {
                if (final_current == final_current_low) final_current = 0.0f;
            }
            if (final_voltage < 0.5f) final_voltage = 0.0f;
            
            float final_power = final_voltage * final_current;
            if (final_power < 0.0f) final_power = 0.0f;

            // Passive Buzzer control via LEDC hardware PWM
            if (final_current > BUZZER_CURRENT_THRESHOLD ||
                final_current < -BUZZER_CURRENT_THRESHOLD) {
                buzzer_on();
            } else {
                buzzer_off();
            }

            char ts[36];
            get_timestamp(ts, sizeof(ts));

            // Output the raw values for calibration steps!
            ESP_LOGI(TAG, "raw_v=%d raw_i_low=%d raw_i_high=%d mv_v=%d mv_i_low=%d mv_i_high=%d",
                     last_raw_v, last_raw_i_low, last_raw_i_high, last_mv_v, last_mv_i_low, last_mv_i_high);

            sensor_data_t reading = {
                .voltage_v = final_voltage,
                .current_a = final_current,
                .power_w   = final_power
            };
            
            portENTER_CRITICAL(&queue_mux);
            xQueueOverwrite(s_queue, &reading);
            portEXIT_CRITICAL(&queue_mux);

            ESP_LOGI(TAG, "[%s] V: %.2f V | I: %.2f A | P: %.2f W",
                     ts, final_voltage, final_current, final_power);

            // Added fabs() so bidirectional negative current still registers as activity! [13]
            if (fabs(final_current) < ACTIVITY_CURRENT_THRESHOLD) {
                inactive_timer_ms += (SAMPLE_INTERVAL_MS * SAMPLES_TO_AVERAGE);
                if (inactive_timer_ms >= INACTIVITY_DURATION_MS) {
                    configure_and_start_ulp();
                }
            } else {
                inactive_timer_ms = 0;
            }

            sum_mv_v = 0; sum_mv_i_low = 0; sum_mv_i_high = 0;
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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_I_LOW, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_I_HIGH, &chan_cfg));

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

    // Initialize Passive Buzzer on GPIO18 using hardware LEDC PWM
    buzzer_init();
    ESP_LOGI(TAG, "Buzzer initialized on GPIO%d via LEDC PWM", BUZZER_GPIO);

    ESP_LOGI(TAG, "ADC reader initialized successfully");
    return ESP_OK;
}

void adc_reader_start(QueueHandle_t queue)
{
    s_queue = queue;
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);
}