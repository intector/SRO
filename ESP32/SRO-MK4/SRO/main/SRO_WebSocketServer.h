/**
 ******************************************************************************
 * @file           : SRO_WebSocketServer.h
 * @brief          : WebSocket Server definitions
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

// Define to prevent recursive inclusion --------------------------------------
#ifndef __SRO_WEBSOCKETSERVER_H__
#define __SRO_WEBSOCKETSERVER_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include "main.h"

// ----------------------------------------------------------------------------
// constants and configuration
// ----------------------------------------------------------------------------
#define SRO_WS_MAX_CLIENTS       4
#define SRO_WS_MAX_MESSAGE_SIZE  1024
#define SRO_WS_PING_INTERVAL_SEC 30
#define SRO_WS_TIMEOUT_SEC       60

// ----------------------------------------------------------------------------
// type definitions
// ----------------------------------------------------------------------------
typedef enum {
    SRO_WS_EVENT_TEMP_UPDATE = 0,
    SRO_WS_EVENT_PROFILE_STATUS,
    SRO_WS_EVENT_WIFI_STATUS,
    SRO_WS_EVENT_SYSTEM_STATUS,
    SRO_WS_EVENT_ERROR,
    SRO_WS_EVENT_TUNING_PROGRESS,
    SRO_WS_EVENT_TUNING_RESULTS,
    SRO_WS_EVENT_TUNING_DATA
} SRO_WebSocketEvent_t;

typedef enum {
    SRO_WS_CMD_START_PROFILE = 0,
    SRO_WS_CMD_STOP_PROFILE,
    SRO_WS_CMD_DOOR_CONTROL,
    SRO_WS_CMD_GET_PROFILES,
    SRO_WS_CMD_SAVE_PROFILE,
    SRO_WS_CMD_DELETE_PROFILE,
    SRO_WS_CMD_GET_STATUS,
    SRO_WS_CMD_HEATER_CONTROL,
    SRO_WS_CMD_FAN_CONTROL,
    SRO_WS_CMD_START_MANUAL_TUNING,
    SRO_WS_CMD_START_AUTO_TUNING,
    SRO_WS_CMD_STOP_TUNING,
    SRO_WS_CMD_SAVE_TUNING_PARAMS,
    SRO_WS_CMD_GET_TUNING_DATA
} SRO_WebSocketCommand_t;

typedef struct {
    SRO_WebSocketEvent_t event;
    char* data;
    size_t data_len;
} SRO_WebSocketMessage_t;

typedef void (*SRO_WebSocketCommandHandler_t)(int client_fd, SRO_WebSocketCommand_t cmd, const char* data);

// ----------------------------------------------------------------------------
// global function prototypes
// ----------------------------------------------------------------------------
esp_err_t SRO_WebSocketServer_Init(void);
esp_err_t SRO_WebSocketServer_Start(void);
esp_err_t SRO_WebSocketServer_Stop(void);
esp_err_t SRO_WebSocketServer_SendEvent(SRO_WebSocketEvent_t event, const char* json_data);
esp_err_t SRO_WebSocketServer_SendToClient(int client_fd, const char* message);
esp_err_t SRO_WebSocketServer_BroadcastMessage(const char* message);
esp_err_t SRO_WebSocketServer_SetCommandHandler(SRO_WebSocketCommandHandler_t handler);
int SRO_WebSocketServer_GetClientCount(void);
esp_err_t SRO_WebSocketServer_DisconnectClient(int client_fd);
uint16_t SRO_WebSocketServer_GetActiveClientCount(void);
esp_err_t SRO_WebSocketServer_DisconnectAllClients(void);
void websocket_command_handler(int client_fd, SRO_WebSocketCommand_t cmd, const char *data);

// ----------------------------------------------------------------------------
// end of SRO_WebSocketServer.h
// ----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // __SRO_WEBSOCKETSERVER_H__