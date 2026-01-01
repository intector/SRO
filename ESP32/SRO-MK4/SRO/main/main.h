/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

// ----------------------------------------------------------------------------
// ESP-IDF Core Includes
// ----------------------------------------------------------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_flash.h"
#include "esp_vfs_fat.h"
#include "wear_levelling.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"

#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "esp_partition.h"

// ----------------------------------------------------------------------------
// SRO Project Includes - Peripheral Drivers (a2s_ prefix)
// ----------------------------------------------------------------------------
#include "a2s_wifi.h"
#include "a2s_max6675.h"
#include "a2s_apa102.h"
#include "a2s_servo.h"
#include "a2s_ssr.h"
#include "a2s_ws2812.h"
#include "a2s_pid.h"

// ----------------------------------------------------------------------------
// SRO Project Includes - Function Groups (SRO_ prefix)
// ----------------------------------------------------------------------------
#include "SRO_TemperatureControl.h"
#include "SRO_ProfileManager.h"
#include "SRO_SystemCoordinator.h"
#include "SRO_WebSocketBroadcast.h"
#include "SRO_WebSocketServer.h"
#include "SRO_WebServer.h"
#include "SRO_FtpServer.h"

// ----------------------------------------------------------------------------
// 3th party library Includes
// ----------------------------------------------------------------------------
#include "cJSON.h"

// ----------------------------------------------------------------------------
// Global Project Includes
// ----------------------------------------------------------------------------
#include "globals.h"

// ----------------------------------------------------------------------------
// The likely and unlikely macro pairs:
// These macros are useful to place when application knows the majority
// ocurrence of a decision paths, placing one of these macros can hint the
// compiler to reorder instructions producing more optimized code.
// ----------------------------------------------------------------------------

#if (CONFIG_COMPILER_OPTIMIZATION_PERF)
#ifndef likely
#define likely(x) __builtin_expect(!!(x), 1)
#endif
#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x), 0)
#endif
#else
#ifndef likely
#define likely(x) (x)
#endif
#ifndef unlikely
#define unlikely(x) (x)
#endif
#endif

// ----------------------------------------------------------------------------
// constants and configuration
// ----------------------------------------------------------------------------
#define SRO_VERSION_MAJOR           1
#define SRO_VERSION_MINOR           0
#define SRO_VERSION_PATCH           0
#define SRO_VERSION_STRING          "1.0.0"

// GPIO Pin Definitions
#define SRO_GPIO_WS2812_BOARD_LED   48  // Onboard LED (internal use only)
#define SRO_GPIO_APA102_MOSI        18  // Status LED strip data
#define SRO_GPIO_APA102_CLK         17  // Status LED strip clock
#define SRO_GPIO_MAX6675_MISO       11  // Temperature sensor data
#define SRO_GPIO_MAX6675_CLK        12  // Temperature sensor clock
#define SRO_GPIO_MAX6675_CS         13  // Temperature sensor chip select
#define SRO_GPIO_HEATER_SSR         4   // 25A SSR control for IR heaters
#define SRO_GPIO_FAN_SSR            5   // 5A SSR control for convection fan
#define SRO_GPIO_SERVO_PWM          10  // Door servo PWM control
#define SRO_GPIO_I2C_SDA            8   // I2C data (future expansion)
#define SRO_GPIO_I2C_SCL            9   // I2C clock (future expansion)
#define SRO_GPIO_OOB_RESET_BUTTON   15  // Factory reset button
#define SRO_GPIO_SPARE_2            16  // Reserved for expansion
#define SRO_GPIO_SPARE_3            21  // Reserved for expansion


// ----------------------------------------------------------------------------
// Task Stack Sizes (bytes)
// ----------------------------------------------------------------------------
#define SRO_STACK_SIZE_HEATER_PWM          1024 * 4
#define SRO_STACK_SIZE_TEMP_CONTROL        1024 * 6
#define SRO_STACK_SIZE_PROFILE             1024 * 6
#define SRO_STACK_SIZE_COORDINATOR         1024 * 6
#define SRO_STACK_SIZE_WEBSOCKET_BROADCAST 1024 * 6
#define SRO_STACK_SIZE_BROADCAST_TIMER     1024 * 4

// ----------------------------------------------------------------------------
// Task Priorities (0-25, higher = more important)
// ----------------------------------------------------------------------------
#define SRO_PRIORITY_HEATER_PWM          11     // Highest - must always run
#define SRO_PRIORITY_TEMP_CONTROL        10
#define SRO_PRIORITY_PROFILE             8
#define SRO_PRIORITY_COORDINATOR         6
#define SRO_PRIORITY_WEBSOCKET_BROADCAST 5
#define SRO_PRIORITY_BROADCAST_TIMER     2

// ----------------------------------------------------------------------------
// Task Core Affinity
// ----------------------------------------------------------------------------
#define SRO_CORE_REALTIME   1 // Core 1: Time-critical tasks
#define SRO_CORE_BACKGROUND 0 // Core 0: Network & I/O

// ----------------------------------------------------------------------------
// Timing Constants (milliseconds)
// ----------------------------------------------------------------------------
#define SRO_INTERVAL_TEMP_READING   1000
#define SRO_INTERVAL_PROFILE_UPDATE 100     // was 1000
#define SRO_INTERVAL_HEARTBEAT      5000
#define SRO_INTERVAL_MEMORY_CHECK   30000

// ----------------------------------------------------------------------------
// Event Groups
// ----------------------------------------------------------------------------
extern EventGroupHandle_t sro_ctrl1_events;
extern EventGroupHandle_t sro_ctrl2_events;
extern EventGroupHandle_t sro_status1_events;
extern EventGroupHandle_t sro_status2_events;

// Control Event Group 1 - Hardware Commands
#define SRO_CE1_TASKS_RELEASE      (1 << 0)
#define SRO_CE1_TEMP_READING_READY (1 << 1)
#define SRO_CE1_TEMP_UPDATE_DONE   (1 << 2)
#define SRO_CE1_HEATER_ON_CMD      (1 << 3)
#define SRO_CE1_HEATER_OFF_CMD     (1 << 4)
#define SRO_CE1_FAN_ON_CMD         (1 << 5)
#define SRO_CE1_FAN_OFF_CMD        (1 << 6)
#define SRO_CE1_DOOR_MOVE_CMD      (1 << 7)
#define SRO_CE1_EMERGENCY_STOP     (1 << 8)
#define SRO_CE1_PROFILE_UPDATE     (1 << 9)
#define SRO_CE1_PID_COMPUTE        (1 << 10)

// Control Event Group 2 - Data & Network Commands
#define SRO_CE2_WEBSOCKET_BROADCAST (1 << 0)
#define SRO_CE2_MEMORY_CHECK        (1 << 1)
#define SRO_CE2_SUSPEND_APA102      (1 << 2)
#define SRO_CE2_SUSPEND_TEMP        (1 << 3)
#define SRO_CE2_TUNING_COMPLETE     (1 << 4)
#define SRO_CE2_SOLDERING_COMPLETE  (1 << 5)
#define SRO_CE2_BROADCAST_ENABLE    (1 << 6)

// just a place holder during comissioning
#define SRO_CE1_DUMMY_EVENT         (1 << 23)

// Status Event Group 1 - Hardware Status
#define SRO_SE1_SPARE_0x000001    (1 << 0)
#define SRO_SE1_TEMP_VALID        (1 << 1)
#define SRO_SE1_TEMP_SENSOR_FAULT (1 << 2)
#define SRO_SE1_HEATER_ON         (1 << 3)
#define SRO_SE1_FAN_ON            (1 << 4)
#define SRO_SE1_DOOR_OPEN         (1 << 5)
#define SRO_SE1_DOOR_CLOSED       (1 << 6)
#define SRO_SE1_DOOR_MOVING       (1 << 7)
#define SRO_SE1_PROFILE_RUNNING   (1 << 8)
#define SRO_SE1_PID_ACTIVE        (1 << 9)

// Status Event Group 2 - System Status
#define SRO_SE2_SYSTEM_INITIALIZED (1 << 0)
#define SRO_SE2_WIFI_CONNECTED     (1 << 1)
#define SRO_SE2_WEBSERVER_RUNNING  (1 << 2)
#define SRO_SE2_FTP_RUNNING        (1 << 3)
#define SRO_SE2_LOW_MEMORY         (1 << 4)
#define SRO_SE2_FILESYSTEM_MOUNTED (1 << 5)
#define SRO_SE2_NVS_INITIALIZED    (1 << 6)
#define SRO_SE2_APA102_SUSPENDED   (1 << 7)
#define SRO_SE2_TEMP_SUSPENDED     (1 << 8)

// ----------------------------------------------------------------------------
// global variables
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// global function prototypes
// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif // __MAIN_H__