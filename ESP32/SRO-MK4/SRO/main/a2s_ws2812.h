/**
 ******************************************************************************
 * @file           : a2s_ws2812.h
 * @brief          : A2S - WS2812 Onboard Status LED Header File
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
#ifndef __A2S_WS2812_H__
#define __A2S_WS2812_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// ----------------------------------------------------------------------------
// WS2812 Configuration
// ----------------------------------------------------------------------------

#define A2S_WS2812_LED_COUNT      1   // Single onboard LED
#define A2S_WS2812_BRIGHTNESS_25  64  // 25% brightness (64/255)

// WS2812 timing (in nanoseconds)
#define A2S_WS2812_T0H_NS   350 // 0 bit high time
#define A2S_WS2812_T0L_NS   800 // 0 bit low time
#define A2S_WS2812_T1H_NS   700 // 1 bit high time
#define A2S_WS2812_T1L_NS   600 // 1 bit low time
#define A2S_WS2812_RESET_US 300 // Reset time in microseconds

// ----------------------------------------------------------------------------
// Boot Status Colors (RGB values at 25% brightness)
// ----------------------------------------------------------------------------

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} a2s_ws2812_rgb_t;

// Predefined boot status colors (all at 25% brightness)
#define A2S_WS2812_OFF       { 0,   0,   0   }  // LED off
#define A2S_WS2812_GREEN     { 0,   255, 0   }  // Normal boot / OOB complete
#define A2S_WS2812_YELLOW    { 255, 255, 0   }  // Waiting for button release
#define A2S_WS2812_RED       { 255, 0,   0   }  // OOB function executing
#define A2S_WS2812_BLUE      { 0,   0,   255 }  // Debug/test (if needed)

// Boot Status Enumeration
typedef enum
{
    A2S_WS2812_STATUS_OFF,             // LED off - system fully booted
    A2S_WS2812_STATUS_NORMAL_BOOT,     // Green - normal boot process
    A2S_WS2812_STATUS_WAITING_RELEASE, // Yellow - waiting for button release
    A2S_WS2812_STATUS_OOB_EXECUTING,   // Red - Out-Of-Box function executing
    A2S_WS2812_STATUS_OOB_COMPLETE     // Green - OOB complete, finishing boot
} a2s_ws2812_boot_status_t;

// ----------------------------------------------------------------------------
// Function Prototypes
// ----------------------------------------------------------------------------
esp_err_t a2s_ws2812_init(void);

esp_err_t a2s_ws2812_deinit(void);

esp_err_t a2s_ws2812_set_boot_status(a2s_ws2812_boot_status_t status);

esp_err_t a2s_ws2812_set_color(a2s_ws2812_rgb_t color, uint8_t brightness);

esp_err_t a2s_ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

esp_err_t a2s_ws2812_off(void);

void a2s_ws2812_set_led(bool state);

// ----------------------------------------------------------------------------
// end of a2s_ws2812.h
// ----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // __A2S_WS2812_H__
