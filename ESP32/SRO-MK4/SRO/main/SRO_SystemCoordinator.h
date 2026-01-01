/**
 ******************************************************************************
 * @file           : SRO_SystemCoordinator.h
 * @brief          : Header for SRO_SystemCoordinator.c file.
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
#ifndef __SRO_SYSTEM_COORDINATOR_H
#define __SRO_SYSTEM_COORDINATOR_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include "main.h"

// ----------------------------------------------------------------------------
// SRO_SystemCoordinator Configuration and Constants
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Type Definitions
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Public Function Prototypes
// ----------------------------------------------------------------------------
esp_err_t SRO_SystemCoordinator_Init(void);
void system_coordinator_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // __SRO_SYSTEM_COORDINATOR_H
