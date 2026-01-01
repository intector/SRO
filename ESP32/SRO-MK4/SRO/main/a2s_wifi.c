/* Soldering Reflow Oven
 * Copyright (c) 2025-2030 Intector
 *
 * WiFi manager implementation with default credentials
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

#include "a2s_wifi.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include "wifi_credentials.h"

static const char *TAG = "A2S_WIFI";

#pragma region // ***** local variable definitions *****
static bool initialized = false;
static a2s_wifi_state_t current_state = A2S_WIFI_STATE_DISCONNECTED;
static esp_netif_t* netif_sta = NULL;
static esp_netif_t* netif_ap = NULL;
static EventGroupHandle_t wifi_event_group = NULL;
static nvs_handle_t wifi_nvs_handle = 0;
static int retry_count = 0;
static esp_ip4_addr_t current_ip = {0};
static int current_rssi = -100;
static char current_ssid[A2S_WIFI_SSID_MAX_LEN] = {0};
static bool tried_default_credentials = false;

// Event bits for WiFi event group
#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1
#define WIFI_AP_STARTED_BIT   BIT2

// NVS keys for configuration storage
#define NVS_NAMESPACE         "wifi_config"
#define NVS_KEY_SSID          "ssid"
#define NVS_KEY_PASSWORD      "password"
#define NVS_KEY_AUTO_CONNECT  "auto_connect"
#pragma endregion

#pragma region // ***** static function prototypes *****
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static esp_err_t start_station_mode(const char* ssid, const char* password);
static esp_err_t start_access_point_mode(void);
static esp_err_t open_nvs_handle(void);
static void close_nvs_handle(void);
static esp_err_t save_credentials_to_nvs(const char* ssid, const char* password, bool auto_connect);
static esp_err_t load_credentials_from_nvs(char* ssid, char* password, bool* auto_connect);
static void set_wifi_state(a2s_wifi_state_t new_state);
static esp_err_t set_hostname(void);
static esp_err_t try_default_credentials(void);
esp_err_t a2s_wifi_register_event_callback(wifi_event_callback_t callback, void *user_data);
#pragma endregion

#pragma region // ***** public functions *****

esp_err_t a2s_wifi_init(void)
{
    if (initialized) {
        ESP_LOGW(TAG, "WiFi already initialized");
        return ESP_OK;
    }

    esp_err_t ret;

    a2s_apa102_set_effect(1, A2S_APA102_STATUS_BLINK_FAST, (a2s_apa102_color_t)A2S_APA102_COLOR_YELLOW, 0.20);
    
    // Initialize TCP/IP stack
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create default event loop
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create network interfaces
    netif_sta = esp_netif_create_default_wifi_sta();
    netif_ap = esp_netif_create_default_wifi_ap();

    if (!netif_sta || !netif_ap) {
        ESP_LOGE(TAG, "Failed to create network interfaces");
        return ESP_FAIL;
    }

    // Set hostname for STA interface
    ret = set_hostname();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set hostname: %s", esp_err_to_name(ret));
    }

    // Initialize WiFi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create event group for WiFi events
    wifi_event_group = xEventGroupCreate();
    if (!wifi_event_group) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return ESP_FAIL;
    }

    // Register event handlers
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set WiFi mode to NULL initially
    ret = esp_wifi_set_mode(WIFI_MODE_NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    a2s_apa102_set_effect(1, A2S_APA102_STATUS_SOLID, (a2s_apa102_color_t)A2S_APA102_COLOR_YELLOW, 0.20);

    set_wifi_state(A2S_WIFI_STATE_DISCONNECTED);
    initialized = true;

    ESP_LOGI(TAG, "WiFi system initialized successfully");

    // Connection priority:
    // 1. Try saved credentials if available
    // 2. Try default credentials
    // 3. Start AP mode as fallback

    a2s_wifi_config_t config;
    if (a2s_wifi_load_config(&config) == ESP_OK && config.auto_connect) {
        ESP_LOGI(TAG, "Auto-connecting to saved network: %s", config.ssid);
        a2s_wifi_connect(config.ssid, config.password);
    } else {
        ESP_LOGI(TAG, "No saved credentials, trying default credentials");
        try_default_credentials();
    }

    return ESP_OK;
}

esp_err_t a2s_wifi_connect(const char* ssid, const char* password)
{
    if (!initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!ssid || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID");
        return ESP_ERR_INVALID_ARG;
    }

    if (!password) {
        password = ""; // Allow open networks
    }

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", ssid);

    // Stop AP mode if active
    if (current_state == A2S_WIFI_STATE_AP_MODE) {
        a2s_wifi_stop_ap();
    }

    set_wifi_state(A2S_WIFI_STATE_CONNECTING);
    retry_count = 0;

    esp_err_t ret = start_station_mode(ssid, password);
    if (ret != ESP_OK) {
        set_wifi_state(A2S_WIFI_STATE_ERROR);
        return ret;
    }

    return ESP_OK;
}

esp_err_t a2s_wifi_disconnect(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disconnecting from WiFi");

    esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to disconnect: %s", esp_err_to_name(ret));
    }

    set_wifi_state(A2S_WIFI_STATE_DISCONNECTED);
    return ESP_OK;
}

esp_err_t a2s_wifi_start_ap(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting Access Point mode");

    // Disconnect from station if connected
    if (current_state == A2S_WIFI_STATE_CONNECTED) {
        esp_wifi_disconnect();
    }

    esp_err_t ret = start_access_point_mode();
    if (ret != ESP_OK) {
        set_wifi_state(A2S_WIFI_STATE_ERROR);
        return ret;
    }

    return ESP_OK;
}

esp_err_t a2s_wifi_stop_ap(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (current_state != A2S_WIFI_STATE_AP_MODE) {
        return ESP_OK; // Already stopped
    }

    ESP_LOGI(TAG, "Stopping Access Point mode");

    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop AP: %s", esp_err_to_name(ret));
    }

    set_wifi_state(A2S_WIFI_STATE_DISCONNECTED);
    return ESP_OK;
}

a2s_wifi_state_t a2s_wifi_get_state(void)
{
    return current_state;
}

esp_err_t a2s_wifi_save_config(const a2s_wifi_config_t* config)
{
    if (!config) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(config->ssid) == 0) {
        ESP_LOGE(TAG, "SSID cannot be empty");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Saving WiFi configuration for SSID: %s", config->ssid);

    return save_credentials_to_nvs(config->ssid, config->password, config->auto_connect);
}

esp_err_t a2s_wifi_load_config(a2s_wifi_config_t* config)
{
    if (!config) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    return load_credentials_from_nvs(config->ssid, config->password, &config->auto_connect);
}

const char* a2s_wifi_get_ip_string(void)
{
    static char ip_str[16];

    if (current_state != A2S_WIFI_STATE_CONNECTED) {
        strcpy(ip_str, "0.0.0.0");
    } else {
        esp_ip4addr_ntoa(&current_ip, ip_str, sizeof(ip_str));
    }

    return ip_str;
}

int a2s_wifi_get_rssi(void)
{
    if (current_state != A2S_WIFI_STATE_CONNECTED) {
        return -100;
    }

    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_OK) {
        current_rssi = ap_info.rssi;
    }

    return current_rssi;
}

#pragma endregion

#pragma region // ***** static functions *****

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi station started");
                a2s_apa102_set_effect(1, A2S_APA102_STATUS_BREATHING, (a2s_apa102_color_t)A2S_APA102_COLOR_GREEN, 0.20);
                break;

            case WIFI_EVENT_STA_CONNECTED:
                {
                    wifi_event_sta_connected_t* event = (wifi_event_sta_connected_t*) event_data;
                    ESP_LOGI(TAG, "Connected to AP: %s", event->ssid);
                    strncpy(current_ssid, (char*)event->ssid, sizeof(current_ssid) - 1);
                    current_ssid[sizeof(current_ssid) - 1] = '\0';
                    a2s_apa102_set_effect(1, A2S_APA102_STATUS_SOLID, (a2s_apa102_color_t)A2S_APA102_COLOR_GREEN, 0.20);
                }
                break;
                    
            case WIFI_EVENT_STA_DISCONNECTED:
                {
                    wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
                    ESP_LOGW(TAG, "Disconnected from AP (reason: %d)", event->reason);

                    if (current_state == A2S_WIFI_STATE_CONNECTING) {
                        retry_count++;
                        if (retry_count < A2S_WIFI_MAX_RETRY) {
                            ESP_LOGI(TAG, "Retrying connection (%d/%d)", retry_count, A2S_WIFI_MAX_RETRY);
                            esp_wifi_connect();
                        } else {
                            ESP_LOGE(TAG, "Failed to connect after %d retries", A2S_WIFI_MAX_RETRY);
                            set_wifi_state(A2S_WIFI_STATE_ERROR);
                            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);

                            // Try default credentials if not already tried
                            if (!tried_default_credentials) {
                                ESP_LOGI(TAG, "Trying default credentials as fallback");
                                vTaskDelay(pdMS_TO_TICKS(1000)); // Brief delay
                                try_default_credentials();
                            } else {
                                // Start AP mode as final fallback
                                ESP_LOGI(TAG, "Starting AP mode as final fallback");
                                a2s_wifi_start_ap();
                            }
                        }
                    } else {
                        set_wifi_state(A2S_WIFI_STATE_DISCONNECTED);
                        a2s_apa102_set_effect(1, A2S_APA102_STATUS_SOLID, (a2s_apa102_color_t)A2S_APA102_COLOR_YELLOW, 0.20);
                    }

                    // Clear IP address
                    current_ip.addr = 0;
                    current_rssi = -100;
                    memset(current_ssid, 0, sizeof(current_ssid));
                }
                break;

            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "Access Point started");
                uint8_t mac[6];
                esp_wifi_get_mac(WIFI_IF_AP, mac);
                ESP_LOGI(TAG, "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                ESP_LOGI(TAG, "IP:  192.168.4.1");
                set_wifi_state(A2S_WIFI_STATE_AP_MODE);
                xEventGroupSetBits(wifi_event_group, WIFI_AP_STARTED_BIT);
                a2s_apa102_set_effect(1, A2S_APA102_STATUS_BREATHING, (a2s_apa102_color_t)A2S_APA102_COLOR_MAGENTA, 0.20);

                break;

            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "Access Point stopped");
                set_wifi_state(A2S_WIFI_STATE_DISCONNECTED);
                break;

            case WIFI_EVENT_AP_STACONNECTED:
                {
                    wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                    ESP_LOGI(TAG, "Station connected to AP: %02x:%02x:%02x:%02x:%02x:%02x", 
                             event->mac[0], event->mac[1], event->mac[2], 
                             event->mac[3], event->mac[4], event->mac[5]);
                    a2s_apa102_set_effect(1, A2S_APA102_STATUS_SOLID, (a2s_apa102_color_t)A2S_APA102_COLOR_MAGENTA, 0.20);
                }
                break;

            case WIFI_EVENT_AP_STADISCONNECTED:
                {
                    wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                    ESP_LOGI(TAG, "Station disconnected from AP: %02x:%02x:%02x:%02x:%02x:%02x", 
                             event->mac[0], event->mac[1], event->mac[2], 
                             event->mac[3], event->mac[4], event->mac[5]);
                    a2s_apa102_set_effect(1, A2S_APA102_STATUS_BREATHING, (a2s_apa102_color_t)A2S_APA102_COLOR_MAGENTA, 0.20);
            }
                break;

            default:
                ESP_LOGD(TAG, "Unhandled WiFi event: %ld", event_id);
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                {
                    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                    ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
                    current_ip = event->ip_info.ip;
                    set_wifi_state(A2S_WIFI_STATE_CONNECTED);
                    retry_count = 0;
                    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

                    ESP_LOGI(TAG, "=== IP Address Assigned ===");
                    uint8_t mac[6];
                    esp_wifi_get_mac(WIFI_IF_STA, mac);
                    ESP_LOGI(TAG, "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
                    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
                    ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));

                    // Check if we got the expected IP for default credentials
                    if (current_ip.addr == ESP_IP4TOADDR(192, 168, 1, 65)) {
                        ESP_LOGI(TAG, "Received expected DHCP reservation IP: 192.168.1.100");
                    }
                }
                break;

            default:
                ESP_LOGD(TAG, "Unhandled IP event: %ld", event_id);
                break;
        }
    }
}

static esp_err_t start_station_mode(const char* ssid, const char* password)
{
    esp_err_t ret;

    // Stop WiFi first if running
    esp_wifi_stop();

    // Set WiFi mode to station
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set station mode: %s", esp_err_to_name(ret));
        return ret;
    }

    uint8_t custom_mac[6] = {0xCA, 0xFE, 0xFE, 0xED, 0x17, 0x64};
    esp_err_t mac_ret     = esp_wifi_set_mac(WIFI_IF_STA, custom_mac);
    if (mac_ret == ESP_OK) {
        ESP_LOGI(TAG, "Custom MAC address set: CA:FE:FE:ED:17:64");
    }
    else {
        ESP_LOGE(TAG, "Failed to set custom MAC address: %s", esp_err_to_name(mac_ret));
    }

    // Configure WiFi
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    // Enable scan before connect for better reliability
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    // Clear event bits
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    // Start WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // connecting to WiFi network
    esp_wifi_connect();

    return ESP_OK;
}

static esp_err_t start_access_point_mode(void)
{
    esp_err_t ret;

    // Set WiFi mode to Access Point
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set AP mode: %s", esp_err_to_name(ret));
        return ret;
    }

            // ADD MAC ADDRESS SETTING HERE
    uint8_t custom_mac[6] = {0xCA, 0xFE, 0xFE, 0xED, 0x17, 0x65};
    esp_err_t mac_ret = esp_wifi_set_mac(WIFI_IF_STA, custom_mac);
    if (mac_ret == ESP_OK) {
        ESP_LOGI(TAG, "Custom MAC address set: CA:FE:FE:ED:17:65");
    } else {
        ESP_LOGE(TAG, "Failed to set custom MAC address: %s", esp_err_to_name(mac_ret));
    }


    // Configure Access Point
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = A2S_WIFI_AP_SSID,
            .ssid_len = strlen(A2S_WIFI_AP_SSID),
            .channel = 1,
            .password = A2S_WIFI_AP_PASSWORD,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    // Use open authentication if no password
    if (strlen(A2S_WIFI_AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set AP config: %s", esp_err_to_name(ret));
        return ret;
    }

    // Clear event bits
    xEventGroupClearBits(wifi_event_group, WIFI_AP_STARTED_BIT);

    ESP_LOGI(TAG, "AP configuration: SSID=%s, Password=%s", A2S_WIFI_AP_SSID, A2S_WIFI_AP_PASSWORD);

    return ESP_OK;
}

static esp_err_t set_hostname(void)
{
    esp_err_t ret;

    // Set hostname for mDNS and DHCP
    ret = esp_netif_set_hostname(netif_sta, A2S_WIFI_DEFAULT_HOSTNAME);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set hostname: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Set hostname to: %s", A2S_WIFI_DEFAULT_HOSTNAME);

    return ESP_OK;
}

static esp_err_t try_default_credentials(void)
{
    ESP_LOGI(TAG, "=== WiFi Connection Attempt Debug ===");
    ESP_LOGI(TAG, "Current WiFi state: %d", current_state);
    ESP_LOGI(TAG, "Retry count: %d", retry_count);
    ESP_LOGI(TAG, "Tried default credentials before: %s", tried_default_credentials ? "YES" : "NO");
    ESP_LOGI(TAG, "Network interface status:");
    
    // Check network interface status
    if (netif_sta) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif_sta, &ip_info) == ESP_OK) {
            ESP_LOGI(TAG, "STA IP: " IPSTR, IP2STR(&ip_info.ip));
        }
    }

    tried_default_credentials = true;
    ESP_LOGI(TAG, "Attempting connection with default credentials: %s", A2S_WIFI_DEFAULT_SSID);
    
    return a2s_wifi_connect(A2S_WIFI_DEFAULT_SSID, A2S_WIFI_DEFAULT_PASSWORD);
}

static esp_err_t open_nvs_handle(void)
{
    if (wifi_nvs_handle != 0) {
        return ESP_OK; // Already open
    }

    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &wifi_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(ret));
    }

    return ret;
}

static void close_nvs_handle(void)
{
    if (wifi_nvs_handle != 0) {
        nvs_close(wifi_nvs_handle);
        wifi_nvs_handle = 0;
    }
}

static esp_err_t save_credentials_to_nvs(const char* ssid, const char* password, bool auto_connect)
{
    esp_err_t ret = open_nvs_handle();
    if (ret != ESP_OK) {
        return ret;
    }

    // Save SSID
    ret = nvs_set_str(wifi_nvs_handle, NVS_KEY_SSID, ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save SSID: %s", esp_err_to_name(ret));
        close_nvs_handle();
        return ret;
    }

    // Save password
    ret = nvs_set_str(wifi_nvs_handle, NVS_KEY_PASSWORD, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save password: %s", esp_err_to_name(ret));
        close_nvs_handle();
        return ret;
    }

    // Save auto-connect flag
    ret = nvs_set_u8(wifi_nvs_handle, NVS_KEY_AUTO_CONNECT, auto_connect ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save auto-connect flag: %s", esp_err_to_name(ret));
        close_nvs_handle();
        return ret;
    }

    // Commit changes
    ret = nvs_commit(wifi_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "WiFi credentials saved successfully");
    }

    close_nvs_handle();
    return ret;
}

static esp_err_t load_credentials_from_nvs(char* ssid, char* password, bool* auto_connect)
{
    esp_err_t ret = open_nvs_handle();
    if (ret != ESP_OK) {
        return ret;
    }

    // Load SSID
    size_t ssid_len = A2S_WIFI_SSID_MAX_LEN;
    ret = nvs_get_str(wifi_nvs_handle, NVS_KEY_SSID, ssid, &ssid_len);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "No saved SSID found: %s", esp_err_to_name(ret));
        close_nvs_handle();
        return ret;
    }

    // Load password
    size_t password_len = A2S_WIFI_PASSWORD_MAX_LEN;
    ret = nvs_get_str(wifi_nvs_handle, NVS_KEY_PASSWORD, password, &password_len);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "No saved password found: %s", esp_err_to_name(ret));
        // Continue with empty password for open networks
        password[0] = '\0';
    }

    // Load auto-connect flag
    uint8_t auto_connect_val = 0;
    ret = nvs_get_u8(wifi_nvs_handle, NVS_KEY_AUTO_CONNECT, &auto_connect_val);
    if (ret == ESP_OK) {
        *auto_connect = (auto_connect_val != 0);
    } else {
        *auto_connect = false; // Default to false
    }

    close_nvs_handle();
    ESP_LOGI(TAG, "WiFi credentials loaded: SSID=%s, Auto-connect=%s", ssid, *auto_connect ? "true" : "false");

    return ESP_OK;
}

static void set_wifi_state(a2s_wifi_state_t new_state)
{
    if (current_state != new_state) {
        ESP_LOGI(TAG, "WiFi state changed: %d -> %d", current_state, new_state);
        current_state = new_state;
    }
}

#pragma endregion

esp_err_t a2s_wifi_connect_stored(void)
{
    a2s_wifi_config_t config;
    esp_err_t ret = a2s_wifi_load_config(&config);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "No stored WiFi config found: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (strlen(config.ssid) == 0) {
        ESP_LOGE(TAG, "No stored SSID found");
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Connecting to stored WiFi network: %s", config.ssid);
    return a2s_wifi_connect(config.ssid, config.password);
}

// =====================================================================================
// end of a2s_wifi.c
// =====================================================================================
