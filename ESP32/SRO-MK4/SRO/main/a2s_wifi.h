/* Soldering Reflow Oven
 * Copyright (c) 2025-2030 Intector
 *
 * WiFi manager definitions
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

#ifndef __A2S_WIFI_H__
#define __A2S_WIFI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// =====================================================================================
// constants and configuration
// =====================================================================================
#define A2S_WIFI_SSID_MAX_LEN       32
#define A2S_WIFI_PASSWORD_MAX_LEN   64
#define A2S_WIFI_AP_SSID            "SRO-Setup"
#define A2S_WIFI_AP_PASSWORD        "sro123456"
#define A2S_WIFI_MAX_RETRY          5
#define A2S_WIFI_CONNECT_TIMEOUT_MS 10000

// =====================================================================================
// type definitions
// =====================================================================================
typedef enum {
    A2S_WIFI_STATE_DISCONNECTED,
    A2S_WIFI_STATE_CONNECTING,
    A2S_WIFI_STATE_CONNECTED,
    A2S_WIFI_STATE_AP_MODE,
    A2S_WIFI_STATE_ERROR
} a2s_wifi_state_t;

typedef struct {
    char ssid[A2S_WIFI_SSID_MAX_LEN];
    char password[A2S_WIFI_PASSWORD_MAX_LEN];
    bool auto_connect;
} a2s_wifi_config_t;

typedef void (*wifi_event_callback_t)(a2s_wifi_state_t state, void *user_data);

// =====================================================================================
// function prototypes
// =====================================================================================
esp_err_t a2s_wifi_init(void);
esp_err_t a2s_wifi_connect(const char* ssid, const char* password);
esp_err_t a2s_wifi_disconnect(void);
esp_err_t a2s_wifi_start_ap(void);
esp_err_t a2s_wifi_stop_ap(void);
a2s_wifi_state_t a2s_wifi_get_state(void);
esp_err_t a2s_wifi_save_config(const a2s_wifi_config_t* config);
esp_err_t a2s_wifi_load_config(a2s_wifi_config_t* config);
const char* a2s_wifi_get_ip_string(void);
int a2s_wifi_get_rssi(void);
esp_err_t a2s_wifi_connect_stored(void);

#ifdef __cplusplus
}
#endif

#endif // __A2S_WIFI_H__