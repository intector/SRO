/**
 ******************************************************************************
 * @file           : SRO_WebServer.c
 * @brief          : HTTP Web Server implementation adapted for current SRO
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
#include "SRO_WebServer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "cJSON.h"
#include <dirent.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include "SRO_WebSocketServer.h"
#include "esp_heap_caps.h"

static const char *TAG = "SRO_WEBSERVER";

// ----------------------------------------------------------------------------
// static variable
// ----------------------------------------------------------------------------
static httpd_handle_t g_server = NULL;
static bool g_server_running = false;
static uint64_t g_server_start_time = 0;

/* File Extension to Content Type Mapping */
static const struct {
    const char *ext;
    const char *type;
} content_type_map[] = {
    {".html", "text/html"},
    {".htm", "text/html"},
    {".css", "text/css"},
    {".js", "application/javascript"},
    {".json", "application/json"},
    {".png", "image/png"},
    {".jpg", "image/jpeg"},
    {".jpeg", "image/jpeg"},
    {".ico", "image/x-icon"},
    {".txt", "text/plain"},
    {".xml", "application/xml"},
    {".svg", "image/svg+xml"},
    {".woff", "font/woff"},
    {".woff2", "font/woff2"},
    {".ttf", "font/ttf"}
};

// ----------------------------------------------------------------------------
// default web content
// ----------------------------------------------------------------------------
#pragma region // ***** Default HTML Content *****

static const char *default_index_html =
    "<!DOCTYPE html>\n"
    "<html lang='en'>\n"
    "<head>\n"
    "    <meta charset='UTF-8'>\n"
    "    <meta name='viewport' content='width=device-width, initial-scale=1.0'>\n"
    "    <title>SRO Controller</title>\n"
    "    <style>\n"
    "        body { font-family: Arial, sans-serif; margin: 40px; background: #f5f5f5; }\n"
    "        .container { max-width: 600px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }\n"
    "        h1 { color: #333; text-align: center; }\n"
    "        .status { background: #e8f5e8; padding: 15px; border-radius: 5px; margin: 20px 0; }\n"
    "        .temp-display { font-size: 2em; font-weight: bold; color: #007bff; text-align: center; }\n"
    "        .button { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; margin: 5px; }\n"
    "        .button:hover { background: #0056b3; }\n"
    "        .info { background: #d1ecf1; padding: 10px; border-radius: 5px; margin: 10px 0; }\n"
    "    </style>\n"
    "</head>\n"
    "<body>\n"
    "    <div class='container'>\n"
    "        <h1>SRO Controller</h1>\n"
    "        <div class='status'>\n"
    "            <h3>System Status</h3>\n"
    "            <div class='temp-display'>\n"
    "                <span id='temp'>Loading...</span>\n"
    "            </div>\n"
    "            <p><strong>Target:</strong> <span id='target'>--</span>°C</p>\n"
    "            <p><strong>Heater:</strong> <span id='heater'>--</span></p>\n"
    "            <p><strong>Safety:</strong> <span id='safety'>--</span></p>\n"
    "            <p><strong>Uptime:</strong> <span id='uptime'>--</span></p>\n"
    "        </div>\n"
    "        \n"
    "        <div class='info'>\n"
    "            <h4>Real-time Updates</h4>\n"
    "            <p>This page will update automatically via WebSocket connection.</p>\n"
    "            <p><strong>Connection:</strong> <span id='ws-status'>Connecting...</span></p>\n"
    "        </div>\n"
    "        \n"
    "        <div style='text-align: center;'>\n"
    "            <button class='button' onclick='location.reload()'>Refresh</button>\n"
    "            <button class='button' onclick='testWebSocket()'>Test WebSocket</button>\n"
    "        </div>\n"
    "    </div>\n"
    "    \n"
    "    <script>\n"
    "        let ws = null;\n"
    "        \n"
    "        function connectWebSocket() {\n"
    "            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';\n"
    "            const wsUrl = protocol + '//' + window.location.host + '/ws';\n"
    "            \n"
    "            ws = new WebSocket(wsUrl);\n"
    "            \n"
    "            ws.onopen = function() {\n"
    "                document.getElementById('ws-status').textContent = 'Connected';\n"
    "                document.getElementById('ws-status').style.color = 'green';\n"
    "            };\n"
    "            \n"
    "            ws.onmessage = function(event) {\n"
    "                try {\n"
    "                    const data = JSON.parse(event.data);\n"
    "                    console.log('WebSocket data received:', data);\n"
    "                    updateDisplay(data);\n"
    "                } catch (e) {\n"
    "                    console.error('WebSocket message parse error:', e);\n"
    "                }\n"
    "            };\n"
    "            \n"
    "            ws.onclose = function() {\n"
    "                document.getElementById('ws-status').textContent = 'Disconnected';\n"
    "                document.getElementById('ws-status').style.color = 'red';\n"
    "                setTimeout(connectWebSocket, 3000); // Reconnect after 3s\n"
    "            };\n"
    "            \n"
    "            ws.onerror = function() {\n"
    "                document.getElementById('ws-status').textContent = 'Error';\n"
    "                document.getElementById('ws-status').style.color = 'red';\n"
    "            };\n"
    "        }\n"
    "        \n"
    "        function updateDisplay(data) {\n"
    "            if (data.event === 'tempUpdate') {\n"
    "                if (data.temp !== undefined) {\n"
    "                    document.getElementById('temp').textContent = data.temp.toFixed(1) + '°C';\n"
    "                }\n"
    "                if (data.target !== undefined) {\n"
    "                    document.getElementById('target').textContent = data.target.toFixed(1);\n"
    "                }\n"
    "                if (data.heater !== undefined) {\n"
    "                    document.getElementById('heater').textContent = data.heater ? 'ON' : 'OFF';\n"
    "                }\n"
    "            }\n"
    "            \n"
    "            if (data.event === 'systemStatus') {\n"
    "                if (data.safe !== undefined) {\n"
    "                    document.getElementById('safety').textContent = data.safe ? 'OK' : 'FAULT';\n"
    "                }\n"
    "                if (data.uptime !== undefined) {\n"
    "                    document.getElementById('uptime').textContent = Math.floor(data.uptime / 1000) + 's';\n"
    "                }\n"
    "            }\n"
    "        }\n"
    "        \n"
    "        function testWebSocket() {\n"
    "            if (ws && ws.readyState === WebSocket.OPEN) {\n"
    "                ws.send(JSON.stringify({cmd: 'getStatus'}));\n"
    "                console.log('Test message sent to WebSocket');\n"
    "            } else {\n"
    "                console.log('WebSocket not connected');\n"
    "            }\n"
    "        }\n"
    "        \n"
    "        // Initial status fetch (fallback)\n"
    "        fetch('/api/status')\n"
    "            .then(response => response.json())\n"
    "            .then(data => {\n"
    "                console.log('Initial API status:', data);\n"
    "                if (data.current_temp !== undefined) {\n"
    "                    document.getElementById('temp').textContent = data.current_temp.toFixed(1) + '°C';\n"
    "                }\n"
    "                if (data.target_temp !== undefined) {\n"
    "                    document.getElementById('target').textContent = data.target_temp.toFixed(1);\n"
    "                }\n"
    "                if (data.heater_on !== undefined) {\n"
    "                    document.getElementById('heater').textContent = data.heater_on ? 'ON' : 'OFF';\n"
    "                }\n"
    "                if (data.safety_ok !== undefined) {\n"
    "                    document.getElementById('safety').textContent = data.safety_ok ? 'OK' : 'FAULT';\n"
    "                }\n"
    "                if (data.uptime !== undefined) {\n"
    "                    document.getElementById('uptime').textContent = Math.floor(data.uptime / 1000) + 's';\n"
    "                }\n"
    "            })\n"
    "            .catch(err => {\n"
    "                console.error('Failed to fetch initial status:', err);\n"
    "                document.getElementById('temp').textContent = 'API Error';\n"
    "            });\n"
    "        \n"
    "        // Connect WebSocket on load\n"
    "        connectWebSocket();\n"
    "    </script>\n"
    "</body>\n"
    "</html>";

#pragma endregion

// ----------------------------------------------------------------------------
// static function prototypes
// ----------------------------------------------------------------------------
static const char* get_content_type(const char *filename);
static esp_err_t send_file(httpd_req_t *req, const char *filepath, const char *content_type);
static esp_err_t send_json_response(httpd_req_t *req, cJSON *json);
static esp_err_t send_error_response(httpd_req_t *req, int code, const char *message);
static esp_err_t handler_index(httpd_req_t *req);
static esp_err_t handler_static_file(httpd_req_t *req);
static esp_err_t handler_favicon(httpd_req_t *req);
static esp_err_t handler_api_control(httpd_req_t *req);
static esp_err_t handler_api_status(httpd_req_t *req);
static esp_err_t handler_api_system_info(httpd_req_t *req);
static esp_err_t handler_api_profiles_list(httpd_req_t *req);
static esp_err_t handler_api_profiles_get(httpd_req_t *req);
static esp_err_t handler_api_profiles_save(httpd_req_t *req);
static esp_err_t handler_api_profiles_delete(httpd_req_t *req);
static httpd_handle_t start_server(void);
static esp_err_t stop_server(httpd_handle_t server);

// ----------------------------------------------------------------------------
// global function calls
// ----------------------------------------------------------------------------
esp_err_t SRO_WebServer_Init(void)
{
    
    // Create default index.html if needed
    if (!sro_fatfs_file_exists("/web/index.html")) {
        char full_path[256];
        snprintf(full_path, sizeof(full_path), "/fatfs/web/index.html");
        
        FILE *file = fopen(full_path, "w");
        if (file) {
            fwrite(default_index_html, 1, strlen(default_index_html), file);
            fclose(file);
        } else {
            ESP_LOGW(TAG, "Failed to create default index.html");
        }
    }
    
    return ESP_OK;
}

esp_err_t SRO_WebServer_Start(void)
{
    if (g_server_running) {
        ESP_LOGW(TAG, "Web server already running");
        return ESP_OK;
    }
    
    g_server = start_server();
    if (!g_server) {
        ESP_LOGE(TAG, "Failed to start web server");
        return ESP_FAIL;
    }
    
    g_server_running = true;
    g_server_start_time = esp_timer_get_time() / 1000;
    
    return ESP_OK;
}

esp_err_t SRO_WebServer_Stop(void)
{
    if (!g_server_running) {
        ESP_LOGW(TAG, "Web server not running");
        return ESP_OK;
    }
    
    esp_err_t ret = stop_server(g_server);
    if (ret == ESP_OK) {
        g_server = NULL;
        g_server_running = false;
    } else {
        ESP_LOGE(TAG, "Failed to stop web server: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t SRO_WebServer_RegisterApiHandlers(void)
{
    // API handlers are registered during server startup
    return ESP_OK;
}

const char* SRO_WebServer_GetMimeType(const char* filename)
{
    return get_content_type(filename);
}

esp_err_t SRO_WebServer_SendFile(httpd_req_t* req, const char* filepath)
{
    const char *content_type = get_content_type(filepath);
    return send_file(req, filepath, content_type);
}

esp_err_t SRO_WebServer_SendJson(httpd_req_t* req, const char* json_data)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    return httpd_resp_send(req, json_data, strlen(json_data));
}

esp_err_t SRO_WebServer_SendError(httpd_req_t* req, int status_code, const char* message)
{
    return send_error_response(req, status_code, message);
}

// ----------------------------------------------------------------------------
// static function calls
// ----------------------------------------------------------------------------
static const char* get_content_type(const char *filename)
{
    const char *ext = strrchr(filename, '.');
    if (!ext) return "application/octet-stream";
    
    for (int i = 0; i < sizeof(content_type_map) / sizeof(content_type_map[0]); i++) {
        if (strcasecmp(ext, content_type_map[i].ext) == 0) {
            return content_type_map[i].type;
        }
    }
    return "application/octet-stream";
}

static esp_err_t send_file(httpd_req_t *req, const char *filepath, const char *content_type)
{
    char full_path[256];
    snprintf(full_path, sizeof(full_path), "/fatfs/web%s", filepath);
    
    FILE *file = fopen(full_path, "r");
    if (!file) {
        ESP_LOGW(TAG, "File not found: %s", full_path);
        return send_error_response(req, 404, "File not found");
    }
    
    // Get file size
    fseek(file, 0, SEEK_END);
    size_t file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    //if (file_size > SRO_WEB_MAX_RESP_SIZE) {
    //    fclose(file);
    //    ESP_LOGE(TAG, "File too large: %s (%u bytes)", filepath, (unsigned int)file_size);
    //    return send_error_response(req, 413, "File too large");
    //}
    
    // Set headers
    httpd_resp_set_type(req, content_type);
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
    
    // Send file in chunks
    // char *buffer = malloc(1024);
    char *buffer = heap_caps_malloc(1024, MALLOC_CAP_SPIRAM);
    if (!buffer) {
        fclose(file);
        return send_error_response(req, 500, "Memory allocation failed");
    }
    
    size_t bytes_read;
    while ((bytes_read = fread(buffer, 1, 1024, file)) > 0) {
        if (httpd_resp_send_chunk(req, buffer, bytes_read) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send file chunk");
            break;
        }
    }
    
    httpd_resp_send_chunk(req, NULL, 0); // End response
    
    free(buffer);
    fclose(file);
    
    ESP_LOGD(TAG, "Sent file: %s (%u bytes)", filepath, (unsigned int)file_size);
    return ESP_OK;
}

static esp_err_t send_json_response(httpd_req_t *req, cJSON *json)
{
    char *json_string = cJSON_Print(json);
    if (!json_string) {
        cJSON_Delete(json);
        return send_error_response(req, 500, "JSON serialization failed");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    esp_err_t ret = httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    
    return ret;
}

static esp_err_t send_error_response(httpd_req_t *req, int code, const char *message)
{
    char status_str[32];
    snprintf(status_str, sizeof(status_str), "%d", code);
    
    cJSON *error_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(error_json, "error", code);
    cJSON_AddStringToObject(error_json, "message", message);
    cJSON_AddNumberToObject(error_json, "timestamp", esp_timer_get_time() / 1000);
    
    httpd_resp_set_status(req, status_str);
    esp_err_t ret = send_json_response(req, error_json);
    
    ESP_LOGW(TAG, "HTTP Error %d: %s", code, message);
    return ret;
}

static esp_err_t handler_index(httpd_req_t *req)
{
    ESP_LOGD(TAG, "Serving index page");
    
    // Try to serve index.html, fall back to default content
    if (sro_fatfs_file_exists("/web/index.html")) {
        return send_file(req, "/index.html", "text/html");
    } else {
        ESP_LOGW(TAG, "index.html not found, serving default content");
        httpd_resp_set_type(req, "text/html");
        return httpd_resp_send(req, default_index_html, strlen(default_index_html));
    }
}

static esp_err_t handler_static_file(httpd_req_t *req)
{
    const char *filepath = req->uri;
    const char *content_type = get_content_type(filepath);
    
    ESP_LOGD(TAG, "Static file request: %s", filepath);
    
    char full_path[768];
    snprintf(full_path, sizeof(full_path), "/fatfs/web%s", filepath);
    
    struct stat st;
    if (stat(full_path, &st) != 0) {
        ESP_LOGW(TAG, "File not found: %s", full_path);
        return send_error_response(req, 404, "File not found");
    }
    
    return send_file(req, filepath, content_type);
}

static esp_err_t handler_favicon(httpd_req_t *req)
{
    if (sro_fatfs_file_exists("/web/favicon.ico")) {
        return send_file(req, "/favicon.ico", "image/x-icon");
    } else {
        // Return 204 No Content for missing favicon
        httpd_resp_set_status(req, "204 No Content");
        return httpd_resp_send(req, NULL, 0);
    }
}

static esp_err_t handler_api_control(httpd_req_t *req)
{
    char buf[1024];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        return send_error_response(req, 400, "Invalid request body");
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        return send_error_response(req, 400, "Invalid JSON");
    }
    
    // Handle commands like setTarget, startProfile, emergencyStop
    cJSON *cmd = cJSON_GetObjectItem(json, "command");
    if (cmd && cmd->valuestring) {
        // Process commands here
    }
    
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    return send_json_response(req, response);
}

static esp_err_t handler_api_status(httpd_req_t *req)
{
    cJSON *json = cJSON_CreateObject();
    
    // Get real temperature data
    float current_temp = 25.0f; // Default fallback
    a2s_max6675_read_temperature(&current_temp);
    
    // Get real system states
    a2s_ssr_state_t heater_state = a2s_ssr_get_heater_state();
    a2s_ssr_state_t fan_state    = a2s_ssr_get_fan_state();
    bool safety_ok               = true;
    
    // System information
    cJSON_AddStringToObject(json, "system", "SRO Controller");
    cJSON_AddStringToObject(json, "version", SRO_VERSION_STRING);
    cJSON_AddNumberToObject(json, "uptime", esp_timer_get_time() / 1000);
    
    // Real data instead of placeholders
    cJSON_AddNumberToObject(json, "current_temp", current_temp);
    cJSON_AddNumberToObject(json, "target_temp", 0.0); // Get from PID controller
    cJSON_AddBoolToObject(json, "heater_on", heater_state == A2S_SSR_STATE_ON);
    cJSON_AddBoolToObject(json, "fan_on", fan_state == A2S_SSR_STATE_ON);
    cJSON_AddBoolToObject(json, "safety_ok", safety_ok);
    cJSON_AddBoolToObject(json, "emergency_stop", false);
    
    return send_json_response(req, json);
}

static esp_err_t handler_api_system_info(httpd_req_t *req)
{
    ESP_LOGD(TAG, "API System Info request");
    
    cJSON *json = cJSON_CreateObject();
    
    // ESP32 system information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    cJSON_AddStringToObject(json, "chip_model", "ESP32-S3");
    cJSON_AddNumberToObject(json, "chip_revision", chip_info.revision);
    cJSON_AddNumberToObject(json, "cpu_cores", chip_info.cores);
    
    // Memory information
    cJSON_AddNumberToObject(json, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(json, "min_free_heap", esp_get_minimum_free_heap_size());
    
    // Flash information
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    cJSON_AddNumberToObject(json, "flash_size", (unsigned int)flash_size);
    
    // Software information
    cJSON_AddStringToObject(json, "idf_version", esp_get_idf_version());
    cJSON_AddStringToObject(json, "app_version", "SRO v1.0.0");
    cJSON_AddStringToObject(json, "compile_time", __DATE__ " " __TIME__);
    
    // System status
    cJSON_AddNumberToObject(json, "uptime_ms", esp_timer_get_time() / 1000);
    cJSON_AddNumberToObject(json, "reset_reason", esp_reset_reason());
    
    return send_json_response(req, json);
}

static esp_err_t handler_api_profiles_list(httpd_req_t *req)
{
    DIR *dir = opendir("/fatfs/profiles");
    if (!dir) {
        return send_error_response(req, 500, "Failed to open profiles directory");
    }

    cJSON *profiles = cJSON_CreateArray();
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR || entry->d_name[0] == '.')
            continue;

        // Remove .json extension
        char *dot = strstr(entry->d_name, ".json");
        if (dot) {
            *dot = '\0';
            cJSON_AddItemToArray(profiles, cJSON_CreateString(entry->d_name));
        }
    }
    closedir(dir);

    cJSON *response = cJSON_CreateObject();
    cJSON_AddItemToObject(response, "profiles", profiles);

    return send_json_response(req, response);
}

static esp_err_t handler_api_profiles_get(httpd_req_t *req)
{
    // Parse query string for ?name=profile_0
    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return send_error_response(req, 400, "Missing profile name");
    }

    char name[64];
    if (httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK) {
        return send_error_response(req, 400, "Invalid profile name");
    }

    // Read profile file
    char filepath[128];
    snprintf(filepath, sizeof(filepath), "/fatfs/profiles/%s.json", name);

    FILE *f = fopen(filepath, "r");
    if (!f) {
        return send_error_response(req, 404, "Profile not found");
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *content = malloc(size + 1);
    fread(content, 1, size, f);
    content[size] = '\0';
    fclose(f);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, content, size);

    free(content);
    return ESP_OK;
}

/*
static esp_err_t handler_api_profiles_save(httpd_req_t *req)
{
    char *buf = malloc(req->content_len + 1);
    int ret   = httpd_req_recv(req, buf, req->content_len);
    if (ret <= 0) {
        free(buf);
        return send_error_response(req, 400, "Failed to receive data");
    }
    buf[ret] = '\0';

    // Parse JSON to get profile name
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        free(buf);
        return send_error_response(req, 400, "Invalid JSON");
    }

    cJSON *name_item = cJSON_GetObjectItem(json, "name");
    if (!name_item || !cJSON_IsString(name_item)) {
        cJSON_Delete(json);
        free(buf);
        return send_error_response(req, 400, "Missing profile name");
    }

    // Save to file
    char filepath[128];
    snprintf(filepath, sizeof(filepath), "/fatfs/profiles/%s.json", name_item->valuestring);

    FILE *f = fopen(filepath, "w");
    if (!f) {
        cJSON_Delete(json);
        free(buf);
        return send_error_response(req, 500, "Failed to save profile");
    }

    fwrite(buf, 1, ret, f);
    fclose(f);

    cJSON_Delete(json);
    free(buf);

    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddStringToObject(response, "message", "Profile saved");

    return send_json_response(req, response);
}
*/
static esp_err_t handler_api_profiles_save(httpd_req_t *req)
{
    char *buf = malloc(req->content_len + 1);
    int ret   = httpd_req_recv(req, buf, req->content_len);
    if (ret <= 0) {
        free(buf);
        return send_error_response(req, 400, "Failed to receive data");
    }
    buf[ret] = '\0';

    // Parse JSON to get fileName and profile data
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        free(buf);
        return send_error_response(req, 400, "Invalid JSON");
    }

    // Extract fileName for saving (e.g., "profile_0")
    cJSON *fileName_item = cJSON_GetObjectItem(json, "fileName");
    if (!fileName_item || !cJSON_IsString(fileName_item)) {
        cJSON_Delete(json);
        free(buf);
        return send_error_response(req, 400, "Missing fileName field");
    }

    const char *fileName = fileName_item->valuestring;

    char fileName_copy[32];
    strncpy(fileName_copy, fileName_item->valuestring, sizeof(fileName_copy) - 1);
    fileName_copy[sizeof(fileName_copy) - 1] = '\0';
    
    // Extract id, name, and description for profile_names.json update
    cJSON *id_item   = cJSON_GetObjectItem(json, "id");
    cJSON *name_item = cJSON_GetObjectItem(json, "name");
    cJSON *desc_item = cJSON_GetObjectItem(json, "description");

    if (!id_item || !cJSON_IsNumber(id_item)) {
        cJSON_Delete(json);
        free(buf);
        return send_error_response(req, 400, "Missing or invalid id field");
    }

    // Remove fileName from the JSON before saving (we don't want it in the profile file)
    cJSON_DeleteItemFromObject(json, "fileName");

    // Save profile data to file
    char profile_filepath[128];
    snprintf(profile_filepath, sizeof(profile_filepath), "/fatfs/profiles/%s.json", fileName_copy);

    // Convert modified JSON back to string
    char *json_string = cJSON_Print(json);
    if (!json_string) {
        cJSON_Delete(json);
        free(buf);
        return send_error_response(req, 500, "Failed to serialize JSON");
    }

    FILE *f = fopen(profile_filepath, "w");
    if (!f) {
        free(json_string);
        cJSON_Delete(json);
        free(buf);
        return send_error_response(req, 500, "Failed to save profile");
    }

    fwrite(json_string, 1, strlen(json_string), f);
    fclose(f);
    free(json_string);

    // -------------------------------------------------------------------------
    // Update profile_names.json
    // -------------------------------------------------------------------------

    // Read existing profile_names.json
    cJSON *names_json = NULL;
    FILE *names_file  = fopen("/fatfs/profiles/profile_names.json", "r");

    if (names_file) {
        fseek(names_file, 0, SEEK_END);
        long size = ftell(names_file);
        fseek(names_file, 0, SEEK_SET);

        char *names_buf = malloc(size + 1);
        if (names_buf) {
            fread(names_buf, 1, size, names_file);
            names_buf[size] = '\0';
            names_json      = cJSON_Parse(names_buf);
            free(names_buf);
        }
        fclose(names_file);
    }

    // If file doesn't exist or parse failed, create new object
    if (!names_json) {
        names_json = cJSON_CreateObject();
    }

    // Update or create entry for this profile
    char profile_key[32];
    snprintf(profile_key, sizeof(profile_key), "profile_%d", id_item->valueint);

    cJSON *profile_entry = cJSON_CreateObject();
    cJSON_AddNumberToObject(profile_entry, "id", id_item->valueint);
    cJSON_AddStringToObject(profile_entry, "name",
                            (name_item && cJSON_IsString(name_item)) ? name_item->valuestring : "Unnamed");
    cJSON_AddStringToObject(profile_entry, "description",
                            (desc_item && cJSON_IsString(desc_item)) ? desc_item->valuestring : "");

    // Add fileName with .json extension
    char file_name_with_ext[64];
    snprintf(file_name_with_ext, sizeof(file_name_with_ext), "%s.json", fileName_copy);
    cJSON_AddStringToObject(profile_entry, "file_name", file_name_with_ext);

    // Replace or add the entry
    cJSON_DeleteItemFromObject(names_json, profile_key);
    cJSON_AddItemToObject(names_json, profile_key, profile_entry);

    // Save updated profile_names.json
    char *names_string = cJSON_Print(names_json);
    if (names_string) {
        FILE *names_out = fopen("/fatfs/profiles/profile_names.json", "w");
        if (names_out) {
            fwrite(names_string, 1, strlen(names_string), names_out);
            fclose(names_out);
        }
        free(names_string);
    }

    cJSON_Delete(names_json);
    cJSON_Delete(json);
    free(buf);

    // Send success response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddStringToObject(response, "message", "Profile saved");

    return send_json_response(req, response);
}

static esp_err_t handler_api_profiles_delete(httpd_req_t *req)
{
    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return send_error_response(req, 400, "Missing profile name");
    }

    char name[64];
    if (httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK) {
        return send_error_response(req, 400, "Invalid profile name");
    }

    char filepath[128];
    snprintf(filepath, sizeof(filepath), "/fatfs/profiles/%s.json", name);

    if (unlink(filepath) != 0) {
        return send_error_response(req, 404, "Profile not found");
    }

    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);

    return send_json_response(req, response);
}

static httpd_handle_t start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    // Configure server settings
    config.server_port = SRO_WEB_SERVER_PORT;
    config.max_open_sockets = 15;
    config.max_uri_handlers = 15;
    config.max_resp_headers = 8;
    config.stack_size = 8192;
    config.task_priority = 5;
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;

    
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }
    
    // Register WebSocket handler FIRST
    esp_err_t ws_ret = SRO_WebSocketServer_RegisterHandler(server);
    if (ws_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register WebSocket handler: %s", esp_err_to_name(ws_ret));
    }
    
    // Register URI handlers
    
    // Root handler
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handler_index,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &index_uri);
    
    // Favicon handler
    httpd_uri_t favicon_uri = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = handler_favicon,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &favicon_uri);
    
    // API handlers
    httpd_uri_t api_control_uri = {
        .uri = "/api/control",
        .method = HTTP_POST,
        .handler = handler_api_control,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_control_uri);

    httpd_uri_t api_status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = handler_api_status,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_status_uri);
    
    httpd_uri_t api_system_uri = {
        .uri = "/api/system/info",
        .method = HTTP_GET,
        .handler = handler_api_system_info,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_system_uri);

    httpd_uri_t uri_api_profiles_list = {
        .uri      = "/api/profiles/list",
        .method   = HTTP_GET,
        .handler  = handler_api_profiles_list,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &uri_api_profiles_list);

    httpd_uri_t uri_api_profiles_get = {
        .uri      = "/api/profiles/get", // ?name=profile_0
        .method   = HTTP_GET,
        .handler  = handler_api_profiles_get,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &uri_api_profiles_get);

    httpd_uri_t uri_api_profiles_save = {
        .uri      = "/api/profiles/save",
        .method   = HTTP_POST,
        .handler  = handler_api_profiles_save,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &uri_api_profiles_save);

    httpd_uri_t uri_api_profiles_delete = {
        .uri      = "/api/profiles/delete", // ?name=profile_0
        .method   = HTTP_DELETE,
        .handler  = handler_api_profiles_delete,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &uri_api_profiles_delete);

    // Wildcard handler for static files (must be last)
    httpd_uri_t static_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = handler_static_file,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &static_uri);

    
    return server;
}

static esp_err_t stop_server(httpd_handle_t server)
{
    if (server) {
        return httpd_stop(server);
    }
    return ESP_OK;
}

// Helper function for filesystem checks
bool sro_fatfs_file_exists(const char *filepath)
{
    char full_path[256];
    snprintf(full_path, sizeof(full_path), "/fatfs%s", filepath);
    
    struct stat st;
    return (stat(full_path, &st) == 0);
}

// ----------------------------------------------------------------------------
// end of SRO_WebServer.c
// ----------------------------------------------------------------------------
