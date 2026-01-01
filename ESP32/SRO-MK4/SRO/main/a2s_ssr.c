/**
 * A2S - SSR (Solid State Relay) Control
 * Copyright (c) 2025-2030 Intector
 *
 * SSR (Solid State Relay) control implementation
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

#include "a2s_ssr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "A2S_SSR";

#pragma region // ***** local variable definitions *****
static bool initialized = false;
static a2s_ssr_state_t heater_state = A2S_SSR_STATE_OFF;
static a2s_ssr_state_t fan_state = A2S_SSR_STATE_OFF;
static uint64_t heater_on_start_time = 0;
static uint64_t fan_on_start_time = 0;
static uint32_t heater_total_on_time_ms = 0;
static uint32_t fan_total_on_time_ms = 0;
static bool emergency_shutdown = false;
static bool in_self_test = false;
#pragma endregion

#pragma region // ***** static function prototypes *****
static esp_err_t configure_gpio_pins(void);
static void update_on_time_tracking(a2s_ssr_type_t ssr_type, a2s_ssr_state_t new_state);
static esp_err_t set_ssr_gpio_state(int gpio_pin, a2s_ssr_state_t state);
#pragma endregion

#pragma region // ***** public functions *****

esp_err_t a2s_ssr_init(void)
{
    if (initialized) {
        ESP_LOGW(TAG, "SSR control already initialized");
        return ESP_OK;
    }

    esp_err_t ret = configure_gpio_pins();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set initialized flag BEFORE calling any internal SSR functions
    initialized = true;

    // Ensure all SSRs are off at startup
    a2s_ssr_set_all_off();

    // Reset timing variables
    heater_total_on_time_ms = 0;
    fan_total_on_time_ms = 0;
    emergency_shutdown = false;


    return ESP_OK;
}

esp_err_t a2s_ssr_deinit(void)
{
    if (!initialized) {
        return ESP_OK;
    }

    // Turn off all SSRs before deinitializing
    a2s_ssr_set_all_off();

    // Reset GPIO pins
    gpio_reset_pin(A2S_SSR_HEATER_PIN);
    gpio_reset_pin(A2S_SSR_FAN_PIN);

    initialized = false;

    return ESP_OK;
}

esp_err_t a2s_ssr_set_heater(a2s_ssr_state_t state)
{
    if (!initialized) {
        ESP_LOGE(TAG, "SSR control not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (emergency_shutdown && state == A2S_SSR_STATE_ON) {
        ESP_LOGW(TAG, "Heater activation blocked due to emergency shutdown");
        return ESP_ERR_INVALID_STATE;
    }

    if (heater_state == state) {
        return ESP_OK; // No change needed
    }


    // Update timing before changing state
    update_on_time_tracking(A2S_SSR_HEATER, state);

    if(state == A2S_SSR_STATE_ON)
        a2s_apa102_set_effect(3, A2S_APA102_STATUS_SOLID, (a2s_apa102_color_t)A2S_APA102_COLOR_RED, 0.20);
    else
        a2s_apa102_set_effect(3, A2S_APA102_STATUS_SOLID, (a2s_apa102_color_t)A2S_APA102_COLOR_OFF, 0.20);

    esp_err_t ret = set_ssr_gpio_state(A2S_SSR_HEATER_PIN, state);
    if (ret == ESP_OK) {
        heater_state = state;
    }

    return ret;
}

esp_err_t a2s_ssr_set_fan(a2s_ssr_state_t state)
{
    if (!initialized) {
        ESP_LOGE(TAG, "SSR control not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (fan_state == state) {
        return ESP_OK; // No change needed
    }


    // Update timing before changing state
    update_on_time_tracking(A2S_SSR_FAN, state);

    esp_err_t ret = set_ssr_gpio_state(A2S_SSR_FAN_PIN, state);
    if (ret == ESP_OK) {
        fan_state = state;
    }

    return ret;
}

esp_err_t a2s_ssr_set_all_off(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "SSR control not initialized");
        return ESP_ERR_INVALID_STATE;
    }


    esp_err_t ret1 = a2s_ssr_set_heater(A2S_SSR_STATE_OFF);
    esp_err_t ret2 = a2s_ssr_set_fan(A2S_SSR_STATE_OFF);

    if (ret1 != ESP_OK || ret2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn off all SSRs");
        return ESP_FAIL;
    }

    return ESP_OK;
}

a2s_ssr_state_t a2s_ssr_get_heater_state(void)
{
    if (!initialized) {
        return A2S_SSR_STATE_OFF;
    }
    return heater_state;
}

a2s_ssr_state_t a2s_ssr_get_fan_state(void)
{
    if (!initialized) {
        return A2S_SSR_STATE_OFF;
    }
    return fan_state;
}

esp_err_t a2s_ssr_get_status(a2s_ssr_status_t* status)
{
    if (!initialized) {
        ESP_LOGE(TAG, "SSR control not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (status == NULL) {
        ESP_LOGE(TAG, "Status pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Update current on-time calculations
    uint64_t current_time = esp_timer_get_time();

    status->heater_state = heater_state;
    status->fan_state = fan_state;

    // Calculate total on time including current session
    status->heater_on_time_ms = heater_total_on_time_ms;
    status->fan_on_time_ms = fan_total_on_time_ms;

    if (heater_state == A2S_SSR_STATE_ON && heater_on_start_time > 0) {
        status->heater_on_time_ms += (current_time - heater_on_start_time) / 1000;
    }

    if (fan_state == A2S_SSR_STATE_ON && fan_on_start_time > 0) {
        status->fan_on_time_ms += (current_time - fan_on_start_time) / 1000;
    }

    return ESP_OK;
}

esp_err_t a2s_ssr_emergency_off(void)
{
    if (in_self_test) {
    } else {
        ESP_LOGE(TAG, "EMERGENCY SHUTDOWN ACTIVATED");
    }

    emergency_shutdown = true;

    // Immediately turn off all SSRs regardless of initialization state
    gpio_set_level(A2S_SSR_HEATER_PIN, !A2S_SSR_ACTIVE_LEVEL);
    gpio_set_level(A2S_SSR_FAN_PIN, !A2S_SSR_ACTIVE_LEVEL);

    // Update internal state
    if (initialized) {
        update_on_time_tracking(A2S_SSR_HEATER, A2S_SSR_STATE_OFF);
        update_on_time_tracking(A2S_SSR_FAN, A2S_SSR_STATE_OFF);
        heater_state = A2S_SSR_STATE_OFF;
        fan_state = A2S_SSR_STATE_OFF;
    }

    return ESP_OK;
}

esp_err_t a2s_ssr_self_test(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "SSR control not initialized");
        return ESP_FAIL;
    }


    // Save current states
    a2s_ssr_state_t orig_heater_state = heater_state;
    a2s_ssr_state_t orig_fan_state = fan_state;

    bool test_passed = true;
    
    // Set self-test mode
    in_self_test = true;

    // Test 1: Heater SSR GPIO Control
    if (a2s_ssr_set_heater(A2S_SSR_STATE_ON) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn on heater SSR");
        test_passed = false;
    } else {
        vTaskDelay(pdMS_TO_TICKS(100)); // Brief on period
        
        if (a2s_ssr_set_heater(A2S_SSR_STATE_OFF) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to turn off heater SSR");
            test_passed = false;
        }
    }

    // Test 2: Fan SSR GPIO Control  
    if (a2s_ssr_set_fan(A2S_SSR_STATE_ON) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn on fan SSR");
        test_passed = false;
    } else {
        vTaskDelay(pdMS_TO_TICKS(100)); // Brief on period
        
        if (a2s_ssr_set_fan(A2S_SSR_STATE_OFF) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to turn off fan SSR");
            test_passed = false;
        }
    }

    // Test 3: Emergency shutdown functionality
    a2s_ssr_set_heater(A2S_SSR_STATE_ON);
    a2s_ssr_set_fan(A2S_SSR_STATE_ON);
    
    a2s_ssr_emergency_off();
    
    // Verify emergency shutdown worked by checking internal state
    if (heater_state != A2S_SSR_STATE_OFF || fan_state != A2S_SSR_STATE_OFF) {
        ESP_LOGE(TAG, "Emergency shutdown failed to update internal state");
        test_passed = false;
    }

    // Clear self-test mode and reset emergency shutdown flag
    in_self_test = false;
    emergency_shutdown = false;

    // Restore original states
    a2s_ssr_set_heater(orig_heater_state);
    a2s_ssr_set_fan(orig_fan_state);


    return test_passed ? ESP_OK : ESP_FAIL;
}

esp_err_t a2s_ssr_clear_emergency(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "SSR control not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    emergency_shutdown = false;
    
    return ESP_OK;
}
#pragma endregion

#pragma region // ***** static functions *****

static esp_err_t configure_gpio_pins(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << A2S_SSR_HEATER_PIN) | (1ULL << A2S_SSR_FAN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set both pins to inactive state (SSRs off)
    gpio_set_level(A2S_SSR_HEATER_PIN, !A2S_SSR_ACTIVE_LEVEL);
    gpio_set_level(A2S_SSR_FAN_PIN, !A2S_SSR_ACTIVE_LEVEL);

    return ESP_OK;
}

static void update_on_time_tracking(a2s_ssr_type_t ssr_type, a2s_ssr_state_t new_state)
{
    uint64_t current_time = esp_timer_get_time();

    if (ssr_type == A2S_SSR_HEATER) {
        if (heater_state == A2S_SSR_STATE_ON && new_state == A2S_SSR_STATE_OFF) {
            // Turning off - add elapsed time
            if (heater_on_start_time > 0) {
                heater_total_on_time_ms += (current_time - heater_on_start_time) / 1000;
                heater_on_start_time = 0;
            }
        } else if (heater_state == A2S_SSR_STATE_OFF && new_state == A2S_SSR_STATE_ON) {
            // Turning on - record start time
            heater_on_start_time = current_time;
        }
    } else if (ssr_type == A2S_SSR_FAN) {
        if (fan_state == A2S_SSR_STATE_ON && new_state == A2S_SSR_STATE_OFF) {
            // Turning off - add elapsed time
            if (fan_on_start_time > 0) {
                fan_total_on_time_ms += (current_time - fan_on_start_time) / 1000;
                fan_on_start_time = 0;
            }
        } else if (fan_state == A2S_SSR_STATE_OFF && new_state == A2S_SSR_STATE_ON) {
            // Turning on - record start time
            fan_on_start_time = current_time;
        }
    }
}

static esp_err_t set_ssr_gpio_state(int gpio_pin, a2s_ssr_state_t state)
{
    int gpio_level = (state == A2S_SSR_STATE_ON) ? A2S_SSR_ACTIVE_LEVEL : !A2S_SSR_ACTIVE_LEVEL;
    
    esp_err_t ret = gpio_set_level(gpio_pin, gpio_level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO %d to level %d: %s", gpio_pin, gpio_level, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

#pragma endregion

// =====================================================================================
// end of a2s_ssr.c
// =====================================================================================
