/**
 ******************************************************************************
 * @file           : SRO_WebSocketServer.c
 * @brief          : WebSocket Server implementation
 ******************************************************************************
 *
 * SRO - Soldering Reflow Oven
 * Copyright (c) 2024 Intector Inc.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
// Includes -------------------------------------------------------------------
#include "SRO_WebSocketServer.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "cJSON.h"
#include "a2s_max6675.h"
#include "a2s_ssr.h"
#include "a2s_wifi.h"
#include <string.h>
#include "esp_heap_caps.h"
#include <math.h>

static const char *TAG = "SRO_WEBSOCKET";

// ----------------------------------------------------------------------------
// static variable
// ----------------------------------------------------------------------------
static bool initialized = false;
static bool server_running = false;
static SRO_WebSocketCommandHandler_t command_handler = NULL;

// Connected clients tracking -------------------------------------------------
static struct {
    int fd;
    bool active;
    uint32_t last_ping;
} ws_clients[SRO_WS_MAX_CLIENTS];

static int active_client_count = 0;
static uint32_t last_ping_time = 0;
static httpd_handle_t g_httpd_handle = NULL;

// ----------------------------------------------------------------------------
// static function prototypes
// ----------------------------------------------------------------------------
static void handle_heater_command(const char *data);
static void handle_fan_command(const char *data);
static void handle_door_command(const char *data);
static esp_err_t ws_handler(httpd_req_t *req);
static void ws_async_send(void *arg);
static esp_err_t send_to_client(int fd, const char *message);
static esp_err_t broadcast_to_all_clients(const char *message);
static void add_client(int fd);
static void remove_client(int fd);
static void handle_client_message(int fd, const char *message);
static void send_ping_to_clients(void);
static void send_system_status(int fd);
static void send_wifi_status(int fd);
static esp_err_t SRO_Profile_LoadFromFile(uint8_t profile_id, sro_profile_t *profile, char *error_msg, size_t error_msg_len);
static esp_err_t parse_profile_fields(cJSON *json, sro_profile_t *profile, char *error_msg, size_t error_msg_len);
static void handle_start_profile(int client_fd, cJSON *request_data);
static void send_json_to_client(int client_fd, cJSON *json);

// ----------------------------------------------------------------------------
// public functions
// ----------------------------------------------------------------------------
esp_err_t SRO_WebSocketServer_Init(void)
{
    if (initialized) {
        ESP_LOGW(TAG, "WebSocket server already initialized");
        return ESP_OK;
    }

    // Initialize client tracking
    for (int i = 0; i < SRO_WS_MAX_CLIENTS; i++) {
        ws_clients[i].fd = -1;
        ws_clients[i].active = false;
        ws_clients[i].last_ping = 0;
    }

    active_client_count = 0;
    last_ping_time = 0;

    initialized = true;

    return ESP_OK;
}

esp_err_t SRO_WebSocketServer_Start(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "WebSocket server not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    a2s_apa102_set_effect(2, A2S_APA102_STATUS_BREATHING, (a2s_apa102_color_t)A2S_APA102_COLOR_BLUE, 0.20);

    if (server_running) {
        ESP_LOGW(TAG, "WebSocket server already running");
        return ESP_OK;
    }

    server_running = true;

    return ESP_OK;
}

esp_err_t SRO_WebSocketServer_Stop(void)
{
    if (!server_running) {
        ESP_LOGW(TAG, "WebSocket server not running");
        return ESP_OK;
    }

    // Disconnect all clients
    for (int i = 0; i < SRO_WS_MAX_CLIENTS; i++) {
        if (ws_clients[i].active) {
            remove_client(ws_clients[i].fd);
        }
    }

    server_running = false;

    return ESP_OK;
}

esp_err_t SRO_WebSocketServer_SendEvent(SRO_WebSocketEvent_t event, const char* json_data)
{
    if (!server_running || active_client_count == 0) {
        return ESP_OK; // No clients to send to
    }

    cJSON *json = cJSON_CreateObject();
    
    switch (event) {
        case SRO_WS_EVENT_TEMP_UPDATE:
            cJSON_AddStringToObject(json, "event", "tempUpdate");
            break;
        case SRO_WS_EVENT_PROFILE_STATUS:
            cJSON_AddStringToObject(json, "event", "profileStatus");
            break;
        case SRO_WS_EVENT_WIFI_STATUS:
            cJSON_AddStringToObject(json, "event", "wifiStatus");
            break;
        case SRO_WS_EVENT_SYSTEM_STATUS:
            cJSON_AddStringToObject(json, "event", "systemStatus");
            break;
        case SRO_WS_EVENT_ERROR:
            cJSON_AddStringToObject(json, "event", "error");
            break;
        case SRO_WS_EVENT_TUNING_PROGRESS:
            cJSON_AddStringToObject(json, "event", "tuningProgress");
            break;
        case SRO_WS_EVENT_TUNING_RESULTS:
            cJSON_AddStringToObject(json, "event", "tuningResults");
            break;
        case SRO_WS_EVENT_TUNING_DATA:
            cJSON_AddStringToObject(json, "event", "tuningData");
            break;
        default:
            cJSON_AddStringToObject(json, "event", "unknown");
            break;
    }

    cJSON_AddNumberToObject(json, "timestamp", esp_timer_get_time() / 1000);

    // Parse and merge the provided JSON data
    if (json_data) {
        cJSON *data = cJSON_Parse(json_data);
        if (data) {
            // Merge all items from data into the main json object
            cJSON *item = NULL;
            cJSON_ArrayForEach(item, data) {
                if (item->string) {
                    cJSON *copy = cJSON_Duplicate(item, 1);
                    if (copy) {
                        cJSON_AddItemToObject(json, item->string, copy);
                    }
                }
            }
            cJSON_Delete(data);
        }
    }

    char *json_string = cJSON_Print(json);
    if (json_string) {
        esp_err_t ret = broadcast_to_all_clients(json_string);
        free(json_string);
        cJSON_Delete(json);
        return ret;
    }

    cJSON_Delete(json);
    return ESP_FAIL;
}

esp_err_t SRO_WebSocketServer_SendToClient(int client_fd, const char* message)
{
    if (!server_running) {
        return ESP_ERR_INVALID_STATE;
    }

    return send_to_client(client_fd, message);
}

esp_err_t SRO_WebSocketServer_BroadcastMessage(const char* message)
{
    
    if (!server_running) {
        ESP_LOGE(TAG, "WebSocket server not running");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!message) {
        ESP_LOGE(TAG, "Message is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    return broadcast_to_all_clients(message);
}

esp_err_t SRO_WebSocketServer_SetCommandHandler(SRO_WebSocketCommandHandler_t handler)
{
    command_handler = handler;
    return ESP_OK;
}

int SRO_WebSocketServer_GetClientCount(void)
{
    return active_client_count;
}

esp_err_t SRO_WebSocketServer_DisconnectClient(int client_fd)
{
    remove_client(client_fd);
    return ESP_OK;
}

// Function to register WebSocket handler with HTTP server
esp_err_t SRO_WebSocketServer_RegisterHandler(httpd_handle_t server)
{
    if (!server) {
        return ESP_ERR_INVALID_ARG;
    }

    // Store server handle for sending data
    g_httpd_handle = server;

    httpd_uri_t ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = ws_handler,
        .user_ctx   = NULL,
        .is_websocket = true
    };
    
    esp_err_t ret = httpd_register_uri_handler(server, &ws);
    if (ret == ESP_OK) {
    } else {
        ESP_LOGE(TAG, "Failed to register WebSocket handler: %s", esp_err_to_name(ret));
    }
    
    return ret;

}

uint16_t SRO_WebSocketServer_GetActiveClientCount(void)
{
    // The active_client_count variable is already maintained internally
    // We can just return it, but let's verify by counting active clients
    uint16_t verified_count = 0;
    
    for (int i = 0; i < SRO_WS_MAX_CLIENTS; i++) {
        if (ws_clients[i].active && ws_clients[i].fd >= 0) {
            verified_count++;
        }
    }
    
    // If counts don't match, log a warning and correct the internal counter
    if (verified_count != active_client_count) {
        ESP_LOGW("SRO_WS", "Client count mismatch: internal=%d, actual=%d", 
                 active_client_count, verified_count);
        active_client_count = verified_count;  // Correct the internal counter
    }
    
    return verified_count;
}

esp_err_t SRO_WebSocketServer_DisconnectAllClients(void)
{
    ESP_LOGW("SRO_WS", "Emergency: Disconnecting all WebSocket clients");

    for (int i = 0; i < SRO_WS_MAX_CLIENTS; i++) {
        if (ws_clients[i].active) {
            ESP_LOGW("SRO_WS", "Force disconnecting client fd=%d", ws_clients[i].fd);
            remove_client(ws_clients[i].fd);
        }
    }

    // *** ADD THIS: Ensure broadcast is disabled after disconnecting all ***
    if (active_client_count == 0) {
        xEventGroupClearBits(sro_ctrl2_events, SRO_CE2_BROADCAST_ENABLE);
    }

    return ESP_OK;
}

void websocket_command_handler(int client_fd, SRO_WebSocketCommand_t cmd, const char *data)
{
    cJSON *response     = NULL;
    cJSON *request_data = NULL;

    // Parse request data if provided
    if (data && strlen(data) > 0) {
        request_data = cJSON_Parse(data);
    }

    switch (cmd) {
        case SRO_WS_CMD_START_PROFILE: {
            handle_start_profile(client_fd, request_data);
            break;
        }
        case SRO_WS_CMD_STOP_PROFILE: {
            SRO_ProfileManager_Stop();

            // Send response
            response = cJSON_CreateObject();
            cJSON_AddStringToObject(response, "event", "profileStopped");
            cJSON_AddBoolToObject(response, "success", true);
            cJSON_AddStringToObject(response, "message", "Profile stopped successfully");

            // Trigger broadcast
            xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_WEBSOCKET_BROADCAST);
            break;
        }

        case SRO_WS_CMD_DOOR_CONTROL:
            handle_door_command(data);
            break;
        case SRO_WS_CMD_HEATER_CONTROL:
            handle_heater_command(data);
            break;

        case SRO_WS_CMD_FAN_CONTROL:
            handle_fan_command(data);
            break;

        case SRO_WS_CMD_GET_PROFILES: {
            // This would need a more complex implementation
            // For now, just trigger a broadcast
            xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_WEBSOCKET_BROADCAST);
            break;
        }

        case SRO_WS_CMD_START_AUTO_TUNING: {
            response = cJSON_CreateObject();
            cJSON_AddStringToObject(response, "event", "autoTuneStatus");

            float target_temp  = 150.0f; // Default
            bool valid_request = false;

            if (request_data) {
                // New simplified format: {"targetTemp": 150}
                cJSON *target_item = cJSON_GetObjectItem(request_data, "targetTemp");
                if (target_item && cJSON_IsNumber(target_item)) {
                    target_temp   = (float)target_item->valuedouble;
                    valid_request = true;
                }

                // Legacy format support: {"method":"...", "parameters":{"target_temp":150}}
                if (!valid_request) {
                    cJSON *params = cJSON_GetObjectItem(request_data, "parameters");
                    if (params) {
                        cJSON *legacy_target = cJSON_GetObjectItem(params, "target_temp");
                        if (legacy_target && cJSON_IsNumber(legacy_target)) {
                            target_temp   = (float)legacy_target->valuedouble;
                            valid_request = true;
                        }
                    }
                }
            }

            if (valid_request) {
                esp_err_t err = SRO_TemperatureControl_StartTuning(A2S_PID_TUNE_STEP_RESPONSE, target_temp);

                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Auto-tune started: target=%.1f°C", target_temp);
                    cJSON_AddStringToObject(response, "phase", "heat");
                    cJSON_AddNumberToObject(response, "progress", 0);
                    cJSON_AddStringToObject(response, "message", "Heating to target temperature");
                    cJSON_AddNumberToObject(response, "targetTemp", target_temp);
                    cJSON_AddBoolToObject(response, "success", true);
                }
                else {
                    ESP_LOGE(TAG, "Auto-tune start failed: %s", esp_err_to_name(err));
                    cJSON_AddStringToObject(response, "phase", "idle");
                    cJSON_AddStringToObject(response, "message", "Failed to start auto-tune");
                    cJSON_AddBoolToObject(response, "success", false);
                }
            }
            else {
                ESP_LOGW(TAG, "Invalid auto-tune request - missing targetTemp");
                cJSON_AddStringToObject(response, "phase", "idle");
                cJSON_AddStringToObject(response, "message", "Missing targetTemp parameter");
                cJSON_AddBoolToObject(response, "success", false);
            }
            break;
        }
        case SRO_WS_CMD_STOP_TUNING: {
            SRO_TemperatureControl_StopTuning();

            response = cJSON_CreateObject();
            cJSON_AddStringToObject(response, "event", "autoTuneStatus");
            cJSON_AddStringToObject(response, "phase", "idle");
            cJSON_AddStringToObject(response, "message", "Auto-tune aborted");
            cJSON_AddBoolToObject(response, "success", true);
            break;
        }
        case SRO_WS_CMD_SAVE_TUNING_PARAMS: {
            response = cJSON_CreateObject();
            cJSON_AddStringToObject(response, "event", "tuningParamsSaved");

            if (request_data) {
                cJSON *kp_item = cJSON_GetObjectItem(request_data, "kp");
                cJSON *ki_item = cJSON_GetObjectItem(request_data, "ki");
                cJSON *kd_item = cJSON_GetObjectItem(request_data, "kd");

                if (cJSON_IsNumber(kp_item) && cJSON_IsNumber(ki_item) && cJSON_IsNumber(kd_item)) {
                    float kp      = (float)kp_item->valuedouble;
                    float ki      = (float)ki_item->valuedouble;
                    float kd      = (float)kd_item->valuedouble;

                    esp_err_t err = SRO_TemperatureControl_SetPIDParams(kp, ki, kd);

                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "PID params saved: Kp=%.4f, Ki=%.6f, Kd=%.2f", kp, ki, kd);
                        cJSON_AddBoolToObject(response, "success", true);
                    }
                    else {
                        ESP_LOGE(TAG, "Failed to save PID params: %s", esp_err_to_name(err));
                        cJSON_AddBoolToObject(response, "success", false);
                    }
                }
            }
            break;
        }
        default:
            break;
    }

    // Send response if one was created
    if (response) {
        char *json_string = cJSON_Print(response);
        if (json_string) {
            SRO_WebSocketServer_SendToClient(client_fd, json_string);
            free(json_string);
        }
        cJSON_Delete(response);
    }

    // Clean up request data
    if (request_data) {
        cJSON_Delete(request_data);
    }
}

// ----------------------------------------------------------------------------
// static functions
// ----------------------------------------------------------------------------
static void handle_heater_command(const char *data)
{
    if (!data)
        return;

    cJSON *json = cJSON_Parse(data);
    if (!json)
        return;

    cJSON *state_item = cJSON_GetObjectItem(json, "state");
    if (state_item && cJSON_IsString(state_item)) {
        if (strcmp(state_item->valuestring, "on") == 0) {
            cJSON *target_item = cJSON_GetObjectItem(json, "targetTemperature");
            float target       = (float)target_item->valuedouble;
            SRO_TemperatureControl_SetTarget(target);
            SRO_TemperatureControl_EnablePID(true);
            ESP_LOGI(TAG, "Heater ON with PID target: %.1f°C", target);
        }
        else {
            SRO_TemperatureControl_SetTarget(0.0f);
            SRO_TemperatureControl_EnablePID(false);
            ESP_LOGI(TAG, "Heater OFF");
        }
        xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_WEBSOCKET_BROADCAST);
    }

    cJSON_Delete(json);
}

static void handle_fan_command(const char *data)
{
    if (!data)
        return;

    SRO_TemperatureControl_SetManualFanRequest(strcmp(data, "on") == 0);

    xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_WEBSOCKET_BROADCAST);
}

static void handle_door_command(const char *data)
{
    if (!data)
        return;

    // Data is numeric position string
    int position = atoi(data);
    if (position >= 0 && position <= 100) {
        a2s_servo_move_to((uint8_t)position, A2S_SERVO_SMOOTH_FAST);
    }
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        add_client(httpd_req_to_sockfd(req)); // Add client on handshake
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    // Set max_len = 0 to get the frame len
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }

    if (ws_pkt.len) {
        // Allocate buffer for message
        // buf = calloc(1, ws_pkt.len + 1);
        buf = heap_caps_calloc(1, ws_pkt.len + 1, MALLOC_CAP_SPIRAM);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;

        // Get the frame payload
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
    }

    // Handle different frame types
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        
        // Add client if new connection
        // add_client(httpd_req_to_sockfd(req));
        
        // Handle the message
        handle_client_message(httpd_req_to_sockfd(req), (char*)ws_pkt.payload);
        
    } else if (ws_pkt.type == HTTPD_WS_TYPE_PONG) {
        ESP_LOGD(TAG, "Received PONG message");
        
        // Update client ping time
        int fd = httpd_req_to_sockfd(req);
        for (int i = 0; i < SRO_WS_MAX_CLIENTS; i++) {
            if (ws_clients[i].fd == fd && ws_clients[i].active) {
                ws_clients[i].last_ping = esp_timer_get_time() / 1000;
                break;
            }
        }
        
    } else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        remove_client(httpd_req_to_sockfd(req));
    }

    if (buf) {
        free(buf);
    }
    return ESP_OK;
}

static esp_err_t send_to_client(int fd, const char *message)
{
    
    if (!g_httpd_handle || !message) {
        ESP_LOGE(TAG, "Invalid httpd handle (%p) or message (%p)", g_httpd_handle, message);
        return ESP_FAIL;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)message;
    ws_pkt.len = strlen(message);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t result = httpd_ws_send_frame_async(g_httpd_handle, fd, &ws_pkt);
    
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "WebSocket send failed: %s", esp_err_to_name(result));
    } else {
    }
    
    return result;
}

static esp_err_t broadcast_to_all_clients(const char *message)
{
    
    if (active_client_count == 0 || !message) {
        ESP_LOGW(TAG, "No clients (%d) or null message", active_client_count);
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;
    int successful_sends = 0;
    
    // Debug: Show all client states
    for (int i = 0; i < SRO_WS_MAX_CLIENTS; i++) {
    }
    
    for (int i = 0; i < SRO_WS_MAX_CLIENTS; i++) {
        if (ws_clients[i].active) {
            esp_err_t send_ret = send_to_client(ws_clients[i].fd, message);
            if (send_ret == ESP_OK) {
                successful_sends++;
            } else {
                ESP_LOGE(TAG, "Failed to send to client %d: %s", 
                         ws_clients[i].fd, esp_err_to_name(send_ret));
                // Remove disconnected client
                ESP_LOGW(TAG, "Removing disconnected client %d", ws_clients[i].fd);
                remove_client(ws_clients[i].fd);
                ret = ESP_FAIL;
            }
        }
    }
    
    return ret;
    
}

static void add_client(int fd)
{

    // Find empty slot
    for (int i = 0; i < SRO_WS_MAX_CLIENTS; i++) {
        if (!ws_clients[i].active) {
            ws_clients[i].fd        = fd;
            ws_clients[i].active    = true;
            ws_clients[i].last_ping = esp_timer_get_time() / 1000;
            active_client_count++;

            ESP_LOGI(TAG, "Client connected (fd=%d), total clients: %d", fd, active_client_count);

            // *** Enable periodic broadcasts when first client connects ***
            if (active_client_count == 1) {
                ESP_LOGI(TAG, "First client connected - enabling periodic broadcasts");
                xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_BROADCAST_ENABLE);
                a2s_apa102_set_effect(2, A2S_APA102_STATUS_SOLID, (a2s_apa102_color_t)A2S_APA102_COLOR_BLUE, 0.20);
            }

            // Send initial status to new client
            send_system_status(fd);

            return;
        }
    }

    ESP_LOGW(TAG, "Maximum clients reached, rejecting connection");
}

static void remove_client(int fd)
{
    for (int i = 0; i < SRO_WS_MAX_CLIENTS; i++) {
        if (ws_clients[i].fd == fd && ws_clients[i].active) {
            ws_clients[i].fd        = -1;
            ws_clients[i].active    = false;
            ws_clients[i].last_ping = 0;
            active_client_count--;

            ESP_LOGI(TAG, "Client disconnected (fd=%d), total clients: %d", fd, active_client_count);

            // *** Disable periodic broadcasts when last client disconnects ***
            if (active_client_count == 0) {
                ESP_LOGI(TAG, "Last client disconnected - disabling periodic broadcasts");
                xEventGroupClearBits(sro_ctrl2_events, SRO_CE2_BROADCAST_ENABLE);
                a2s_apa102_set_effect(2, A2S_APA102_STATUS_BREATHING, (a2s_apa102_color_t)A2S_APA102_COLOR_BLUE, 0.20);
            }

            return;
        }
    }
}

static void handle_client_message(int fd, const char *message)
{
    if (!message || strlen(message) == 0) {
        return;
    }

    cJSON *json = cJSON_Parse(message);
    if (!json) {
        ESP_LOGW(TAG, "Failed to parse JSON message: %s", message);
        return;
    }

    cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
    if (!cmd_item || !cJSON_IsString(cmd_item)) {
        cJSON_Delete(json);
        return;
    }

    const char *cmd = cmd_item->valuestring;

    ESP_LOGI(TAG, "Received command from client: %s", cmd);

    // Handle built-in commands first
    if (strcmp(cmd, "getStatus") == 0) {
        send_system_status(fd);
        // call to obsolate function
        // send_temperature_update(fd);
    } 
    else if (strcmp(cmd, "getWifiStatus") == 0) {
        send_wifi_status(fd);
    } 
    else if (strcmp(cmd, "getSystemInfo") == 0) {
        send_system_status(fd);
    } 
    else if (strcmp(cmd, "ping") == 0) {
        // Send pong response
        const char *pong = "{\"event\":\"pong\"}";
        send_to_client(fd, pong);
    } 
    else {
        // Forward to registered command handler for profile operations
        if (command_handler) {
            // Convert command string to enum - EXPANDED for profile commands
            SRO_WebSocketCommand_t cmd_enum = SRO_WS_CMD_GET_STATUS; // Default
            
            // Control commands
            if (strcmp(cmd, "startProfile") == 0) {
                cmd_enum = SRO_WS_CMD_START_PROFILE;
            }
            else if (strcmp(cmd, "stopProfile") == 0) {
                cmd_enum = SRO_WS_CMD_STOP_PROFILE;
            }
            else if (strcmp(cmd, "cmdDoor") == 0) {
                cmd_enum = SRO_WS_CMD_DOOR_CONTROL;
            }
            // *** PROFILE MANAGEMENT COMMANDS ***
            else if (strcmp(cmd, "getProfiles") == 0) {
                cmd_enum = SRO_WS_CMD_GET_PROFILES;
            }
            else if (strcmp(cmd, "saveProfile") == 0) {
                cmd_enum = SRO_WS_CMD_SAVE_PROFILE;
            }
            else if (strcmp(cmd, "deleteProfile") == 0) {
                cmd_enum = SRO_WS_CMD_DELETE_PROFILE;
            }
            else if (strcmp(cmd, "getStatus") == 0) {
                cmd_enum = SRO_WS_CMD_GET_STATUS;
            }
            else if (strcmp(cmd, "cmdHeater") == 0) {
                cmd_enum = SRO_WS_CMD_HEATER_CONTROL;
            }
            else if (strcmp(cmd, "cmdFan") == 0) {
                cmd_enum = SRO_WS_CMD_FAN_CONTROL;
            }
            else if (strcmp(cmd, "startManualTuning") == 0) {
                cmd_enum = SRO_WS_CMD_START_MANUAL_TUNING;
            }
            else if (strcmp(cmd, "startAutoTuning") == 0) {
                cmd_enum = SRO_WS_CMD_START_AUTO_TUNING;
            }
            else if (strcmp(cmd, "stopTuning") == 0) {
                cmd_enum = SRO_WS_CMD_STOP_TUNING;
            }
            else if (strcmp(cmd, "saveTuningParams") == 0) {
                cmd_enum = SRO_WS_CMD_SAVE_TUNING_PARAMS;
            }
            else if (strcmp(cmd, "getTuningData") == 0) {
                cmd_enum = SRO_WS_CMD_GET_TUNING_DATA;
            }
            
            // Prepare data for command handler
            char *data_string = NULL;
            
            // For complex commands that need the full JSON, serialize it
            if (cmd_enum == SRO_WS_CMD_SAVE_PROFILE || 
                cmd_enum == SRO_WS_CMD_START_PROFILE ||
                cmd_enum == SRO_WS_CMD_DELETE_PROFILE ||
                cmd_enum == SRO_WS_CMD_START_MANUAL_TUNING ||
                cmd_enum == SRO_WS_CMD_START_AUTO_TUNING ||
                cmd_enum == SRO_WS_CMD_SAVE_TUNING_PARAMS) {
                data_string = cJSON_Print(json);
            }
            
            else if (cmd_enum == SRO_WS_CMD_HEATER_CONTROL) {
                data_string = cJSON_Print(json);
            }

            else if (cmd_enum == SRO_WS_CMD_FAN_CONTROL) {
                cJSON *state_item = cJSON_GetObjectItem(json, "state");
                if (cJSON_IsString(state_item)) {
                    // data_string = strdup(state_item->valuestring);
                    data_string = heap_caps_malloc(strlen(state_item->valuestring) + 1, MALLOC_CAP_SPIRAM);
                    if (data_string)
                        strcpy(data_string, state_item->valuestring);
                }
            }

            else if (cmd_enum == SRO_WS_CMD_DOOR_CONTROL) {
                cJSON *position_item = cJSON_GetObjectItem(json, "position");
                if (cJSON_IsNumber(position_item)) {
                    // data_string = malloc(16);
                    data_string = heap_caps_malloc(16, MALLOC_CAP_SPIRAM);
                    if (data_string) {
                        snprintf(data_string, 16, "%d", (int)position_item->valuedouble);
                    }
                }
            }
            // For other simple commands, use existing logic
            else {
                cJSON *data_item = cJSON_GetObjectItem(json, "id");
                if (!data_item) {
                    data_item = cJSON_GetObjectItem(json, "data");
                }
                if (!data_item) {
                    data_item = cJSON_GetObjectItem(json, "value");
                }
    
                if (data_item) {
                    if (cJSON_IsString(data_item)) {
                        // data_string = strdup(data_item->valuestring);
                        data_string = heap_caps_malloc(strlen(data_item->valuestring) + 1, MALLOC_CAP_SPIRAM);
                        if (data_string)
                            strcpy(data_string, data_item->valuestring);
                    } else if (cJSON_IsNumber(data_item)) {
                        // data_string = malloc(32);
                        data_string = heap_caps_malloc(32, MALLOC_CAP_SPIRAM);
                        if (data_string) {
                            snprintf(data_string, 32, "%.2f", data_item->valuedouble);
                        }
                    }
                }
            }

            ESP_LOGI(TAG, "Forwarding command %s (enum=%d) to handler", cmd, cmd_enum);
            
            // Call the registered command handler
            command_handler(fd, cmd_enum, data_string);
            
            // Clean up allocated data string
            if (data_string) {
                free(data_string);
            }
        } else {
            ESP_LOGW(TAG, "No command handler registered for: %s", cmd);
        }
    }

    cJSON_Delete(json);
}

static void send_system_status(int fd)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "event", "systemStatus");
    cJSON_AddNumberToObject(json, "timestamp", esp_timer_get_time() / 1000);
    cJSON_AddNumberToObject(json, "uptime", esp_timer_get_time() / 1000);
    cJSON_AddBoolToObject(json, "safe", true);
    cJSON_AddStringToObject(json, "version", SRO_VERSION_STRING);

    char *json_string = cJSON_Print(json);
    if (json_string) {
        send_to_client(fd, json_string);
        free(json_string);
    }
    cJSON_Delete(json);
}

static void send_wifi_status(int fd)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "event", "wifiStatus");
    cJSON_AddNumberToObject(json, "timestamp", esp_timer_get_time() / 1000);
    
    // Get WiFi state and information
    a2s_wifi_state_t wifi_state = a2s_wifi_get_state();
    bool connected = (wifi_state == A2S_WIFI_STATE_CONNECTED);
    
    cJSON_AddBoolToObject(json, "connected", connected);
    
    if (connected) {
        // Get current WiFi information
        a2s_wifi_config_t config;
        if (a2s_wifi_load_config(&config) == ESP_OK) {
            cJSON_AddStringToObject(json, "ssid", config.ssid);
        }
        
        const char* ip_str = a2s_wifi_get_ip_string();
        if (ip_str && strcmp(ip_str, "0.0.0.0") != 0) {
            cJSON_AddStringToObject(json, "ip_address", ip_str);
        }
        
        int rssi = a2s_wifi_get_rssi();
        if (rssi != -100) {
            cJSON_AddNumberToObject(json, "rssi", rssi);
        }
    }
    
    char *json_string = cJSON_Print(json);
    if (json_string) {
        send_to_client(fd, json_string);
        free(json_string);
    }
    cJSON_Delete(json);
}

static void send_ping_to_clients(void)
{
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    if (current_time - last_ping_time >= (SRO_WS_PING_INTERVAL_SEC * 1000)) {
        const char *ping_msg = "{\"event\":\"ping\"}";
        broadcast_to_all_clients(ping_msg);
        last_ping_time = current_time;
    }
    
    // Check for timeout clients
    for (int i = 0; i < SRO_WS_MAX_CLIENTS; i++) {
        if (ws_clients[i].active) {
            if (current_time - ws_clients[i].last_ping > (SRO_WS_TIMEOUT_SEC * 1000)) {
                ESP_LOGW(TAG, "Client timeout, disconnecting fd=%d", ws_clients[i].fd);
                remove_client(ws_clients[i].fd);
            }
        }
    }
}

static esp_err_t SRO_Profile_LoadFromFile(uint8_t profile_id, sro_profile_t *profile, char *error_msg, size_t error_msg_len)
{
    // 1. Validate profile ID
    if (profile_id > 9) {
        snprintf(error_msg, error_msg_len, "Invalid profile ID: %d", profile_id);
        return ESP_ERR_INVALID_ARG;
    }

    // 2. Build file path
    char filepath[64];
    snprintf(filepath, sizeof(filepath), "/fatfs/profiles/profile_%d.json", profile_id);

    // 3. Open file
    FILE *f = fopen(filepath, "r");
    if (!f) {
        snprintf(error_msg, error_msg_len, "Profile %d not found", profile_id);
        return ESP_ERR_NOT_FOUND;
    }

    // 4. Read file content
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *content = malloc(size + 1);
    if (!content) {
        fclose(f);
        snprintf(error_msg, error_msg_len, "Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }

    fread(content, 1, size, f);
    content[size] = '\0';
    fclose(f);

    // 5. Parse JSON
    cJSON *json = cJSON_Parse(content);
    free(content);

    if (!json) {
        snprintf(error_msg, error_msg_len, "Invalid JSON in profile %d", profile_id);
        return ESP_ERR_INVALID_STATE;
    }

    // 6. Validate and extract fields
    memset(profile, 0, sizeof(sro_profile_t));
    profile->id      = profile_id;

    esp_err_t result = parse_profile_fields(json, profile, error_msg, error_msg_len);

    cJSON_Delete(json);
    return result;
}

static esp_err_t parse_profile_fields(cJSON *json, sro_profile_t *profile, char *error_msg, size_t error_msg_len)
{
    // --- Top-level fields ---
    cJSON *name = cJSON_GetObjectItem(json, "name");
    if (!name || !cJSON_IsString(name)) {
        snprintf(error_msg, error_msg_len, "Missing: name");
        return ESP_ERR_INVALID_ARG;
    }
    strncpy(profile->name, name->valuestring, sizeof(profile->name) - 1);

    cJSON *melt_pt = cJSON_GetObjectItem(json, "solder_melting_point");
    if (!melt_pt || !cJSON_IsNumber(melt_pt)) {
        snprintf(error_msg, error_msg_len, "Missing: solder_melting_point");
        return ESP_ERR_INVALID_ARG;
    }
    profile->solder_melting_point = (float)melt_pt->valuedouble;

    cJSON *peak                   = cJSON_GetObjectItem(json, "absolute_peak_temp");
    if (!peak || !cJSON_IsNumber(peak)) {
        snprintf(error_msg, error_msg_len, "Missing: absolute_peak_temp");
        return ESP_ERR_INVALID_ARG;
    }
    profile->absolute_peak_temp = (float)peak->valuedouble;

    // --- Preheat phase ---
    cJSON *preheat = cJSON_GetObjectItem(json, "preheat");
    if (!preheat) {
        snprintf(error_msg, error_msg_len, "Missing: preheat section");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *ph_max  = cJSON_GetObjectItem(preheat, "max_temp");
    cJSON *ph_time = cJSON_GetObjectItem(preheat, "time_sec");
    if (!ph_max || !cJSON_IsNumber(ph_max)) {
        snprintf(error_msg, error_msg_len, "Missing: preheat.max_temp");
        return ESP_ERR_INVALID_ARG;
    }
    if (!ph_time || !cJSON_IsNumber(ph_time)) {
        snprintf(error_msg, error_msg_len, "Missing: preheat.time_sec");
        return ESP_ERR_INVALID_ARG;
    }
    profile->phases[SRO_PROFILE_PHASE_PREHEAT].min_temp = 0;
    profile->phases[SRO_PROFILE_PHASE_PREHEAT].max_temp = (float)ph_max->valuedouble;
    profile->phases[SRO_PROFILE_PHASE_PREHEAT].time_sec = (uint32_t)ph_time->valueint;

    // --- Soak phase ---
    cJSON *soak = cJSON_GetObjectItem(json, "soak");
    if (!soak) {
        snprintf(error_msg, error_msg_len, "Missing: soak section");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *sk_max  = cJSON_GetObjectItem(soak, "max_temp");
    cJSON *sk_time = cJSON_GetObjectItem(soak, "time_sec");
    if (!sk_max || !cJSON_IsNumber(sk_max)) {
        snprintf(error_msg, error_msg_len, "Missing: soak.max_temp");
        return ESP_ERR_INVALID_ARG;
    }
    if (!sk_time || !cJSON_IsNumber(sk_time)) {
        snprintf(error_msg, error_msg_len, "Missing: soak.time_sec");
        return ESP_ERR_INVALID_ARG;
    }
    profile->phases[SRO_PROFILE_PHASE_SOAK].min_temp = 0;
    profile->phases[SRO_PROFILE_PHASE_SOAK].max_temp = (float)sk_max->valuedouble;
    profile->phases[SRO_PROFILE_PHASE_SOAK].time_sec = (uint32_t)sk_time->valueint;

    // --- Reflow phase ---
    cJSON *reflow = cJSON_GetObjectItem(json, "reflow");
    if (!reflow) {
        snprintf(error_msg, error_msg_len, "Missing: reflow section");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *rf_max  = cJSON_GetObjectItem(reflow, "max_temp");
    cJSON *rf_min  = cJSON_GetObjectItem(reflow, "min_temp");
    cJSON *rf_time = cJSON_GetObjectItem(reflow, "time_sec");
    if (!rf_max || !cJSON_IsNumber(rf_max)) {
        snprintf(error_msg, error_msg_len, "Missing: reflow.max_temp");
        return ESP_ERR_INVALID_ARG;
    }
    if (!rf_min || !cJSON_IsNumber(rf_min)) {
        snprintf(error_msg, error_msg_len, "Missing: reflow.min_temp");
        return ESP_ERR_INVALID_ARG;
    }
    if (!rf_time || !cJSON_IsNumber(rf_time)) {
        snprintf(error_msg, error_msg_len, "Missing: reflow.time_sec");
        return ESP_ERR_INVALID_ARG;
    }
    profile->phases[SRO_PROFILE_PHASE_REFLOW].min_temp = (float)rf_min->valuedouble;
    profile->phases[SRO_PROFILE_PHASE_REFLOW].max_temp = (float)rf_max->valuedouble;
    profile->phases[SRO_PROFILE_PHASE_REFLOW].time_sec = (uint32_t)rf_time->valueint;

    // --- Cooling phase ---
    cJSON *cooling = cJSON_GetObjectItem(json, "cooling");
    if (!cooling) {
        snprintf(error_msg, error_msg_len, "Missing: cooling section");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *cl_target = cJSON_GetObjectItem(cooling, "target_temp");
    cJSON *cl_rate   = cJSON_GetObjectItem(cooling, "max_rate");
    if (!cl_target || !cJSON_IsNumber(cl_target)) {
        snprintf(error_msg, error_msg_len, "Missing: cooling.target_temp");
        return ESP_ERR_INVALID_ARG;
    }
    if (!cl_rate || !cJSON_IsNumber(cl_rate)) {
        snprintf(error_msg, error_msg_len, "Missing: cooling.max_rate");
        return ESP_ERR_INVALID_ARG;
    }

    // Store cooling rate (use absolute value since max_rate is negative in JSON)
    float rate_value      = (float)cl_rate->valuedouble;
    profile->cooling_rate = (rate_value < 0) ? -rate_value : rate_value;

    // Calculate cooling phase duration from rate
    float temp_drop                                     = profile->phases[SRO_PROFILE_PHASE_REFLOW].max_temp - (float)SRO_PROFILE_COOLING_END_TEMPERATURE;
    profile->phases[SRO_PROFILE_PHASE_COOLING].min_temp = 0;
    profile->phases[SRO_PROFILE_PHASE_COOLING].max_temp = (float)SRO_PROFILE_COOLING_END_TEMPERATURE;
    profile->phases[SRO_PROFILE_PHASE_COOLING].time_sec = (uint32_t)(temp_drop / profile->cooling_rate);

    // Optional: estimated duration
    cJSON *est_dur = cJSON_GetObjectItem(json, "estimated_duration_sec");
    if (est_dur && cJSON_IsNumber(est_dur)) {
        profile->estimated_duration_sec = (uint32_t)est_dur->valueint;
    }

    return ESP_OK;
}

static void handle_start_profile(int client_fd, cJSON *request_data)
{
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "event", "profileStarted");

    // Validate request
    if (!request_data) {
        cJSON_AddBoolToObject(response, "success", false);
        cJSON_AddStringToObject(response, "message", "No request data");
        send_json_to_client(client_fd, response);
        cJSON_Delete(response);
        return;
    }

    cJSON *id_item = cJSON_GetObjectItem(request_data, "id");
    if (!id_item || !cJSON_IsNumber(id_item)) {
        cJSON_AddBoolToObject(response, "success", false);
        cJSON_AddStringToObject(response, "message", "Missing profile id");
        send_json_to_client(client_fd, response);
        cJSON_Delete(response);
        return;
    }

    int profile_id = id_item->valueint;
    cJSON_AddNumberToObject(response, "profileId", profile_id);

    // Load and validate profile
    char error_msg[64];
    sro_profile_t profile;
    esp_err_t err = SRO_Profile_LoadFromFile((uint8_t)profile_id, &profile,
                                             error_msg, sizeof(error_msg));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Profile load failed: %s", error_msg);
        cJSON_AddBoolToObject(response, "success", false);
        cJSON_AddStringToObject(response, "message", error_msg);
        send_json_to_client(client_fd, response);
        cJSON_Delete(response);
        return;
    }

    // Start profile
    err = SRO_ProfileManager_Start(&profile);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Profile start failed: %s", esp_err_to_name(err));
        cJSON_AddBoolToObject(response, "success", false);
        cJSON_AddStringToObject(response, "message", esp_err_to_name(err));
        send_json_to_client(client_fd, response);
        cJSON_Delete(response);
        return;
    }

    // Success
    ESP_LOGI(TAG, "Profile '%s' started", profile.name);
    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddStringToObject(response, "message", "Profile started");
    send_json_to_client(client_fd, response);
    cJSON_Delete(response);

    xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_WEBSOCKET_BROADCAST);
}

static void send_json_to_client(int client_fd, cJSON *json)
{
    char *str = cJSON_PrintUnformatted(json);
    if (str) {
        send_to_client(client_fd, str);
        free(str);
    }
}

// ----------------------------------------------------------------------------
// end of SRO_WebSocketServer.c
// ----------------------------------------------------------------------------
