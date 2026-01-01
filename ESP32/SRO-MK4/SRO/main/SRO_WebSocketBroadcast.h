/**
 ******************************************************************************
 * @file           : SRO_WebSocketBroadcast.h
 * @brief          : Header for SRO_WebSocketBroadcast.c file.
 *                   This file contains the common defines of the application.
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
#ifndef __SRO_WEBSOCKET_BROADCAST_H
#define __SRO_WEBSOCKET_BROADCAST_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include "main.h"

// ----------------------------------------------------------------------------
// SRO_WebSocketBroadcast Configuration and Constants
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Type Definitions
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Public Function Prototypes
// ----------------------------------------------------------------------------
esp_err_t SRO_WebSocketBroadcast_Init(void);
void websocket_broadcast_task(void *pvParameters);
void websocket_broadcast_timer_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // __SRO_WEBSOCKET_BROADCAST_H
