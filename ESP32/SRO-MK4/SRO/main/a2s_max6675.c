/**
 * A2S - MAX6675 Temperature Sensor
 * Copyright (c) 2025-2030 Intector
 *
 * MAX6675 temperature sensor implementation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "a2s_max6675.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "A2S_MAX6675";

#pragma region // ***** local variable definitions *****
static spi_device_handle_t spi_device = NULL;
static bool initialized               = false;
static uint32_t last_read_time        = 0;
static float last_temperature         = 0.0f;
static bool last_thermocouple_open    = false;
#pragma endregion

#pragma region // ***** static function prototypes *****
static esp_err_t configure_gpio(void);
static esp_err_t configure_spi(void);
static uint16_t read_raw_data(void);
static float convert_raw_to_celsius(uint16_t raw_data);
static bool is_thermocouple_disconnected(uint16_t raw_data);
#pragma endregion

#pragma region // ***** public functions *****

esp_err_t a2s_max6675_init(void)
{
    if (initialized) {
        ESP_LOGW(TAG, "MAX6675 already initialized");
        return ESP_OK;
    }

    esp_err_t ret;

    // Configure GPIO pins
    ret = configure_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SPI interface
    ret = configure_spi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SPI: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized = true;

    return ESP_OK;
}

esp_err_t a2s_max6675_deinit(void)
{
    if (!initialized) {
        return ESP_OK;
    }

    if (spi_device != NULL) {
        esp_err_t ret = spi_bus_remove_device(spi_device);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to remove SPI device: %s", esp_err_to_name(ret));
        }
        spi_device = NULL;
    }

    // Reset CS pin to high impedance
    gpio_reset_pin(A2S_MAX6675_CS_PIN);

    initialized = false;

    return ESP_OK;
}

/*
a2s_max6675_error_t a2s_max6675_read_temperature(float *temperature)
{
    if (!initialized) {
        ESP_LOGE(TAG, "MAX6675 not initialized");
        return A2S_MAX6675_ERROR_INIT;
    }

    if (temperature == NULL) {
        ESP_LOGE(TAG, "Temperature pointer is NULL");
        return A2S_MAX6675_ERROR_READ;
    }

    // MAX6675 requires 220ms between conversions
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - last_read_time < 220) {
        // Return last reading if too soon
        *temperature = last_temperature;
        return last_thermocouple_open ? A2S_MAX6675_ERROR_THERMOCOUPLE_OPEN : A2S_MAX6675_OK;
    }

    uint16_t raw_data = read_raw_data();

    // Check if device responded (bit 15 should always be 0)
    if (raw_data == 0xFFFF) {
        ESP_LOGE(TAG, "MAX6675 not responding (no power or SPI fault)");
        return A2S_MAX6675_ERROR_READ;
    }
    
    // Check for thermocouple open circuit
    if (is_thermocouple_disconnected(raw_data)) {
        ESP_LOGW(TAG, "Thermocouple disconnected (raw: 0x%04X)", raw_data);
        last_thermocouple_open = true;
        return A2S_MAX6675_ERROR_THERMOCOUPLE_OPEN;
    }

    float temp = convert_raw_to_celsius(raw_data);

    // Validate temperature range
    if (temp < A2S_MAX6675_TEMP_MIN || temp > A2S_MAX6675_TEMP_MAX) {
        ESP_LOGW(TAG, "Temperature out of range: %.2fÂ°C", temp);
        return A2S_MAX6675_ERROR_OUT_OF_RANGE;
    }

    *temperature           = temp;
    last_temperature       = temp;
    last_thermocouple_open = false;
    last_read_time         = current_time;

    ESP_LOGD(TAG, "Temperature: %.2fÂ°C (raw: 0x%04X)", temp, raw_data);

    return A2S_MAX6675_OK;
}
*/
a2s_max6675_error_t a2s_max6675_read_temperature(float *temperature)
{
    if (!initialized) {
        ESP_LOGE(TAG, "MAX6675 not initialized");
        return A2S_MAX6675_ERROR_INIT;
    }

    if (temperature == NULL) {
        ESP_LOGE(TAG, "Temperature pointer is NULL");
        return A2S_MAX6675_ERROR_READ;
    }

    // MAX6675 requires 220ms between conversions
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - last_read_time < 220) {
        // Return last reading if too soon
        *temperature = last_temperature;
        return last_thermocouple_open ? A2S_MAX6675_ERROR_THERMOCOUPLE_OPEN : A2S_MAX6675_OK;
    }

    uint16_t raw_data = read_raw_data();

    // *** CRITICAL FIX: Update timing IMMEDIATELY after read (even on error) ***
    last_read_time = current_time;

    // Check for thermocouple open circuit
    if (is_thermocouple_disconnected(raw_data)) {
        ESP_LOGW(TAG, "Thermocouple disconnected (raw: 0x%04X)", raw_data);
        last_thermocouple_open = true;
        return A2S_MAX6675_ERROR_THERMOCOUPLE_OPEN;
    }

    float temp = convert_raw_to_celsius(raw_data);

    // Validate temperature range
    if (temp < A2S_MAX6675_TEMP_MIN || temp > A2S_MAX6675_TEMP_MAX) {
        ESP_LOGW(TAG, "Temperature out of range: %.2fÂ°C", temp);
        return A2S_MAX6675_ERROR_OUT_OF_RANGE;
    }

    *temperature           = temp;
    last_temperature       = temp;
    last_thermocouple_open = false;
    // Note: last_read_time already updated above

    ESP_LOGD(TAG, "Temperature: %.2fÂ°C (raw: 0x%04X)", temp, raw_data);

    return A2S_MAX6675_OK;
}

a2s_max6675_error_t a2s_max6675_read_full(a2s_max6675_reading_t *reading)
{
    if (!initialized) {
        ESP_LOGE(TAG, "MAX6675 not initialized");
        return A2S_MAX6675_ERROR_INIT;
    }

    if (reading == NULL) {
        ESP_LOGE(TAG, "Reading pointer is NULL");
        return A2S_MAX6675_ERROR_READ;
    }

    a2s_max6675_error_t result = a2s_max6675_read_temperature(&reading->temperature);
    reading->thermocouple_open = (result == A2S_MAX6675_ERROR_THERMOCOUPLE_OPEN);
    reading->timestamp         = esp_timer_get_time() / 1000;

    return result;
}

bool a2s_max6675_is_thermocouple_open(void)
{
    if (!initialized) {
        return true; // Assume open if not initialized
    }

    return last_thermocouple_open;
}

esp_err_t a2s_max6675_self_test(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "MAX6675 not initialized");
        return ESP_FAIL;
    }

    // Test 1: Read multiple samples
    float temperatures[5];
    int valid_readings = 0;

    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(250)); // Wait for conversion

        a2s_max6675_error_t result = a2s_max6675_read_temperature(&temperatures[i]);
        if (result == A2S_MAX6675_OK) {
            valid_readings++;
        }
        else {
            ESP_LOGW(TAG, "Reading %d failed: %d", i + 1, result);
        }
    }

    // Test 2: Check reading consistency (should not vary more than 5Â°C between consecutive readings)
    if (valid_readings >= 2) {
        float max_variation = 0.0f;
        for (int i = 1; i < valid_readings; i++) {
            float variation = fabs(temperatures[i] - temperatures[i - 1]);
            if (variation > max_variation) {
                max_variation = variation;
            }
        }

        if (max_variation > 5.0f) {
            ESP_LOGW(TAG, "Temperature readings too unstable (max variation: %.2fÂ°C)", max_variation);
        }
        else {
        }
    }

    // Test 3: Check if readings are within reasonable range for room temperature operation
    if (valid_readings > 0) {
        float avg_temp = 0.0f;
        for (int i = 0; i < valid_readings; i++) {
            avg_temp += temperatures[i];
        }
        avg_temp /= valid_readings;

        if (avg_temp < 0.0f || avg_temp > 50.0f) {
            ESP_LOGW(TAG, "Temperature outside expected room range (0-50Â°C)");
        }
    }

    return (valid_readings >= 3) ? ESP_OK : ESP_FAIL;
}

#pragma endregion

#pragma region // ***** static functions *****

static esp_err_t configure_gpio(void)
{
    // Configure CS pin as output
    gpio_config_t cs_config = {
        .pin_bit_mask = (1ULL << A2S_MAX6675_CS_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE};

    esp_err_t ret = gpio_config(&cs_config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set CS high (inactive)
    gpio_set_level(A2S_MAX6675_CS_PIN, 1);

    return ESP_OK;
}

static esp_err_t configure_spi(void)
{
    // Initialize SPI2 bus
    spi_bus_config_t bus_config = {
        .miso_io_num     = SRO_GPIO_MAX6675_MISO,
        .mosi_io_num     = -1, // MAX6675 is read-only
        .sclk_io_num     = SRO_GPIO_MAX6675_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4092,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means already initialized - that's OK
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add MAX6675 device to the bus
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = A2S_MAX6675_SPI_FREQ,
        .mode           = 0,
        .spics_io_num   = A2S_MAX6675_CS_PIN,
        .queue_size     = 1,
        .flags          = 0,
        .pre_cb         = NULL,
        .post_cb        = NULL};

    ret = spi_bus_add_device(SPI2_HOST, &dev_config, &spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/*
static uint16_t read_raw_data(void)
{
    if (spi_device == NULL) {
        ESP_LOGE(TAG, "SPI device not configured");
        return 0xFFFF;
    }

    uint8_t rx_data[2]      = {0};
    uint8_t rd_counter      = 0;
    uint16_t raw_data       = 0x0000;

    spi_transaction_t trans = {
        .length    = 16, // 16 bits
        .rx_buffer = rx_data,
        .tx_buffer = NULL,
        .flags     = 0 // Remove any flags that might conflict
    };

    do {

        esp_err_t ret = spi_device_transmit(spi_device, &trans);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
            return 0xFFFF;
        }

        // Combine bytes (MSB first)
        raw_data = (rx_data[0] << 8) | rx_data[1];
        if (raw_data == 0x0000) {
            raw_data = 0xFFFF;
        }

        rd_counter++;

  } while (raw_data == 0xFFFF | rd_counter < 5);

    return raw_data;
}
*/
/*
static uint16_t read_raw_data(void)
{
    if (spi_device == NULL) {
        ESP_LOGE(TAG, "SPI device not configured");
        return 0xFFFF;
    }

    uint8_t rx_data[2]      = {0};
    uint8_t rd_counter      = 0;
    uint16_t raw_data       = 0xFFFF;

    spi_transaction_t trans = {
        .length    = 16,
        .rx_buffer = rx_data,
        .tx_buffer = NULL,
        .flags     = 0};

    do {
        esp_err_t ret = spi_device_transmit(spi_device, &trans);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
            return 0xFFFF;
        }

        // Combine bytes (MSB first)
        raw_data = (rx_data[0] << 8) | rx_data[1];

        // Treat 0x0000 as no-response (sensor unpowered)
        if (raw_data == 0x0000) {
            raw_data = 0xFFFF;
        }

        rd_counter++;

        // Add small delay between retries if needed
        if (raw_data == 0xFFFF && rd_counter < 5) {
            vTaskDelay(pdMS_TO_TICKS(250)); // 250ms delay before retry
        }

    } while (raw_data == 0xFFFF && rd_counter < 5);
    //                           ↑↑ Logical AND - retry while error AND not max attempts

    if (raw_data == 0xFFFF) {
        ESP_LOGW(TAG, "MAX6675 not responding after %d attempts", rd_counter);
    }

    return raw_data;
}
*/
static uint16_t read_raw_data(void)
{
    if (spi_device == NULL) {
        ESP_LOGE(TAG, "SPI device not configured");
        return 0xFFFF;
    }

    uint8_t rx_data[2]      = {0};

    spi_transaction_t trans = {
        .length    = 16,
        .rx_buffer = rx_data,
        .tx_buffer = NULL,
        .flags     = 0};

    esp_err_t ret = spi_device_transmit(spi_device, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        return 0xFFFF;
    }

    uint16_t raw_data = (rx_data[0] << 8) | rx_data[1];

    return raw_data;
}

static float convert_raw_to_celsius(uint16_t raw_data)
{
    // MAX6675 data format:
    // Bit 15: Always 0
    // Bits 14-3: 12-bit temperature data (0.25Â°C resolution)
    // Bit 2: Thermocouple open (1 = open, 0 = connected)
    // Bit 1: Device ID (always 0)
    // Bit 0: Three-state (always 0)

    // Extract temperature data (bits 14-3)
    uint16_t temp_data = (raw_data >> 3) & 0x0FFF;

    // Convert to Celsius (0.25Â°C per LSB)
    float temperature = temp_data * A2S_MAX6675_RESOLUTION;

    return temperature;
}

static bool is_thermocouple_disconnected(uint16_t raw_data)
{
    // Check bit 2 for thermocouple open circuit
    return (raw_data & 0x0004) != 0;
}

#pragma endregion

// =====================================================================================
// end of a2s_max6675.c
// =====================================================================================
