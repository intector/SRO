/**
 * A2S - MAX6675 Driver Header File
 * Copyright (c) 2025-2030 Intector
 *
 * MAX6675 temperature sensor definitions
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

#pragma once

#ifndef __A2S_MAX6675_H__
#define __A2S_MAX6675_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// =====================================================================================
// constants and configuration
// =====================================================================================
#define A2S_MAX6675_MISO_PIN        11
#define A2S_MAX6675_CLK_PIN         12
#define A2S_MAX6675_CS_PIN          13
#define A2S_MAX6675_SPI_FREQ        1000000  // 1MHz
#define A2S_MAX6675_TEMP_MIN        0.0f
#define A2S_MAX6675_TEMP_MAX        1023.75f
#define A2S_MAX6675_RESOLUTION      0.25f    // 0.25°C resolution

// =====================================================================================
// type definitions
// =====================================================================================
typedef enum {
    A2S_MAX6675_OK = 0,
    A2S_MAX6675_ERROR_INIT,
    A2S_MAX6675_ERROR_READ,
    A2S_MAX6675_ERROR_THERMOCOUPLE_OPEN,
    A2S_MAX6675_ERROR_OUT_OF_RANGE
} a2s_max6675_error_t;

typedef struct {
    float temperature;
    bool thermocouple_open;
    uint32_t timestamp;
} a2s_max6675_reading_t;

// =====================================================================================
// function prototypes
// =====================================================================================
esp_err_t a2s_max6675_init(void);
esp_err_t a2s_max6675_deinit(void);
a2s_max6675_error_t a2s_max6675_read_temperature(float* temperature);
a2s_max6675_error_t a2s_max6675_read_full(a2s_max6675_reading_t* reading);
bool a2s_max6675_is_thermocouple_open(void);
esp_err_t a2s_max6675_self_test(void);

#ifdef __cplusplus
}
#endif

#endif // __A2S_MAX6675_H__