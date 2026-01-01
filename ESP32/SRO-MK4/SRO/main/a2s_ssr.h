/**
 * A2S - SSR Control Header File
 * Copyright (c) 2025-2030 Intector
 *
 * SSR (Solid State Relay) control definitions
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

#ifndef __A2S_SSR_H__
#define __A2S_SSR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// =====================================================================================
// constants and configuration
// =====================================================================================
#define A2S_SSR_HEATER_PIN          4
#define A2S_SSR_FAN_PIN             5
#define A2S_SSR_ACTIVE_LEVEL        1   // SSR is active high

// =====================================================================================
// type definitions
// =====================================================================================
typedef enum {
    A2S_SSR_HEATER = 0,
    A2S_SSR_FAN,
    A2S_SSR_COUNT
} a2s_ssr_type_t;

typedef enum {
    A2S_SSR_STATE_OFF = 0,
    A2S_SSR_STATE_ON = 1
} a2s_ssr_state_t;

typedef struct {
    a2s_ssr_state_t heater_state;
    a2s_ssr_state_t fan_state;
    uint32_t heater_on_time_ms;
    uint32_t fan_on_time_ms;
} a2s_ssr_status_t;

// =====================================================================================
// function prototypes
// =====================================================================================
esp_err_t a2s_ssr_init(void);
esp_err_t a2s_ssr_deinit(void);
esp_err_t a2s_ssr_set_heater(a2s_ssr_state_t state);
esp_err_t a2s_ssr_set_fan(a2s_ssr_state_t state);
esp_err_t a2s_ssr_set_all_off(void);
a2s_ssr_state_t a2s_ssr_get_heater_state(void);
a2s_ssr_state_t a2s_ssr_get_fan_state(void);
esp_err_t a2s_ssr_get_status(a2s_ssr_status_t* status);
esp_err_t a2s_ssr_emergency_off(void);
esp_err_t a2s_ssr_self_test(void);
esp_err_t a2s_ssr_clear_emergency(void);
;

#ifdef __cplusplus
}
#endif

#endif // __A2S_SSR_H__