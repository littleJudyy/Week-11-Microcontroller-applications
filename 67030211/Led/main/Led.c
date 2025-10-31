#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

// ======================
// Configuration
// ======================
#define SENSOR_CHANNEL ADC1_CHANNEL_6  // GPIO34 (ADC1_CH6)
#define DEFAULT_VREF    1100           // mV
#define OVERSAMPLES     100
#define FILTER_SIZE     10

static const char *TAG = "ADC_ENHANCED";
static esp_adc_cal_characteristics_t *adc_chars;

// ======================
// Moving Average Filter
// ======================
static float filterBuffer[FILTER_SIZE];
static int filterIndex = 0;
static float filterSum = 0.0;
static bool filterInitialized = false;

static float movingAverageFilter(float newValue) {
    if (!filterInitialized) {
        // เริ่มต้น buffer
        for (int i = 0; i < FILTER_SIZE; i++) filterBuffer[i] = newValue;
        filterSum = newValue * FILTER_SIZE;
        filterInitialized = true;
        return newValue;
    }

    filterSum -= filterBuffer[filterIndex];   // ลบค่าที่เก่าที่สุด
    filterBuffer[filterIndex] = newValue;     // เพิ่มค่าใหม่
    filterSum += newValue;

    filterIndex = (filterIndex + 1) % FILTER_SIZE; // update index

    return filterSum / FILTER_SIZE; // คืนค่าเฉลี่ย
}

// ======================
// ADC Helper Functions
// ======================
static bool check_efuse(void) {
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
        ESP_LOGI(TAG, "eFuse Two Point: รองรับ");
    else
        ESP_LOGI(TAG, "eFuse Two Point: ไม่รองรับ");

    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
        ESP_LOGI(TAG, "eFuse Vref: รองรับ");
    else
        ESP_LOGI(TAG, "eFuse Vref: ไม่รองรับ");

    return true;
}

static void print_char_val_type(esp_adc_cal_value_t val_type) {
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
        ESP_LOGI(TAG, "ใช้การปรับเทียบแบบ Two Point Value");
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
        ESP_LOGI(TAG, "ใช้การปรับเทียบแบบ eFuse Vref");
    else
        ESP_LOGI(TAG, "ใช้การปรับเทียบแบบ Default Vref");
}

// อ่านค่า ADC แบบ Oversampling
static float readADCOversampling(adc1_channel_t channel, int samples) {
    uint64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += adc1_get_raw(channel);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return (float)sum / samples;
}

// แปลง ADC raw เป็น Volt
static float adcRawToVolt(uint32_t raw) {
    uint32_t mv = esp_adc_cal_raw_to_voltage(raw, adc_chars);
    return mv / 1000.0f;
}

// ======================
// Main Application
// ======================
void app_main(void) {
    // ตรวจสอบ eFuse
    check_efuse();

    // ADC Configuration
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_CHANNEL, ADC_ATTEN_DB_11);

    // ADC Calibration
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
        ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars
    );
    print_char_val_type(val_type);

    ESP_LOGI(TAG, "ทดสอบ ADC: Oversampling + Moving Average Filter");
    ESP_LOGI(TAG, "Pin: GPIO34, Oversamples: %d, Filter Size: %d", OVERSAMPLES, FILTER_SIZE);
    ESP_LOGI(TAG, "----------------------------------------");

    while (1) {
        // อ่าน ADC แบบต่างๆ
        uint32_t rawValue = adc1_get_raw(SENSOR_CHANNEL);
        float oversampledValue = readADCOversampling(SENSOR_CHANNEL, OVERSAMPLES);
        float filteredValue = movingAverageFilter(oversampledValue);

        // แปลงเป็น Volt
        float rawVolt = adcRawToVolt(rawValue);
        float oversampledVolt = adcRawToVolt((uint32_t)oversampledValue);
        float filteredVolt = adcRawToVolt((uint32_t)filteredValue);

        // แสดงผล
        ESP_LOGI(TAG, "=== ADC Comparison ===");
        ESP_LOGI(TAG, "Raw        : %d (%.3fV)", rawValue, rawVolt);
        ESP_LOGI(TAG, "Oversample : %.1f (%.3fV)", oversampledValue, oversampledVolt);
        ESP_LOGI(TAG, "Filtered   : %.1f (%.3fV)", filteredValue, filteredVolt);
        ESP_LOGI(TAG, "");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
