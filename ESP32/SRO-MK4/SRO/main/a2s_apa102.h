/**
 ******************************************************************************
 * @file           : a2s_apa102.h
 * @brief          : A2S - APA102 LED Driver Header File
 ******************************************************************************
 *
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
#ifndef __A2S_APA102_H__
#define __A2S_APA102_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "main.h"

// ----------------------------------------------------------------------------
// APA102 Hardware Configuration
// ----------------------------------------------------------------------------

#define A2S_APA102_SPI_FREQUENCY    10000000 // 10 MHz
#define A2S_APA102_START_FRAME_SIZE 4        // 4 bytes start frame (0x00 0x00 0x00 0x00)
#define A2S_APA102_BYTES_PER_LED    4        // 4 bytes per LED (brightness + BGR)
#define A2S_APA102_END_FRAME_SIZE   4        // 32 bit end frame (0xFF 0xFF 0xFF 0xFF)

// ----------------------------------------------------------------------------
// Effect Task Configuration
// ----------------------------------------------------------------------------

#define A2S_APA102_EFFECT_TASK_PRIORITY  5    // Task priority (0-25)
#define A2S_APA102_EFFECT_TASK_STACK     4096 // Stack size in bytes
#define A2S_APA102_EFFECT_TASK_CORE      0    // CPU core (0 or 1)
#define A2S_APA102_EFFECT_UPDATE_RATE_MS 50   // Update rate in milliseconds
#define A2S_APA102_CMD_QUEUE_DEPTH       16   // Command queue depth

// ----------------------------------------------------------------------------
// Effect Timing Configuration
// ----------------------------------------------------------------------------

#define A2S_APA102_BREATHING_PERIOD_MS  2000 // Breathing cycle period
#define A2S_APA102_BLINK_SLOW_PERIOD_MS 1000 // Slow blink period
#define A2S_APA102_BLINK_FAST_PERIOD_MS 250  // Fast blink period
#define A2S_APA102_FADE_DURATION_MS     500  // Fade in/out duration

// ----------------------------------------------------------------------------
// Global Brightness Configuration
// ----------------------------------------------------------------------------

#define A2S_APA102_GLOBAL_BRIGHTNESS_FIXED 31 // Maximum global brightness (0-31)

// ----------------------------------------------------------------------------
// RGB Brightness Compensation
// ----------------------------------------------------------------------------

#define A2S_APA102_RED_COMPENSATION        0.78f // Reduce red by 22%
#define A2S_APA102_GREEN_COMPENSATION      1.00f // Reference level
#define A2S_APA102_BLUE_COMPENSATION       3.17f // Boost blue by 217%
#define A2S_APA102_ENABLE_RGB_COMPENSATION true  // Enable/disable compensation

// ----------------------------------------------------------------------------
// Type Definitions
// ----------------------------------------------------------------------------

// APA102 pixel structure (SPI wire format)
typedef struct __attribute__((packed))
{
    uint8_t brightness; // 0xE0 | (0-31 brightness level)
    uint8_t blue;       // Blue component (0-255)
    uint8_t green;      // Green component (0-255)
    uint8_t red;        // Red component (0-255)
} a2s_apa102_pixel_t;

// Color structure (user-friendly format)
typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;
} a2s_apa102_color_t;

// Effect modes
typedef enum
{
    A2S_APA102_STATUS_OFF = 0,    // LED off
    A2S_APA102_STATUS_SOLID,      // Solid color (no effect)
    A2S_APA102_STATUS_BREATHING,  // Breathing effect
    A2S_APA102_STATUS_BLINK_SLOW, // Slow blinking
    A2S_APA102_STATUS_BLINK_FAST, // Fast blinking
    A2S_APA102_STATUS_FADE_IN,    // Fade in (one-shot)
    A2S_APA102_STATUS_FADE_OUT    // Fade out (one-shot)
} a2s_apa102_status_mode_t;

// ----------------------------------------------------------------------------
// Predefined Colors (RGB format)
// ----------------------------------------------------------------------------

#define A2S_APA102_COLOR_OFF     {0, 0, 0, 0}
#define A2S_APA102_COLOR_RED     {255, 0, 0, A2S_APA102_GLOBAL_BRIGHTNESS_FIXED}
#define A2S_APA102_COLOR_ORANGE  {255, 128, 0, A2S_APA102_GLOBAL_BRIGHTNESS_FIXED}
#define A2S_APA102_COLOR_YELLOW  {255, 255, 0, A2S_APA102_GLOBAL_BRIGHTNESS_FIXED}
#define A2S_APA102_COLOR_GREEN   {0, 255, 0, A2S_APA102_GLOBAL_BRIGHTNESS_FIXED}
#define A2S_APA102_COLOR_AQUA    {0, 255, 255, A2S_APA102_GLOBAL_BRIGHTNESS_FIXED}
#define A2S_APA102_COLOR_BLUE    {0, 0, 255, A2S_APA102_GLOBAL_BRIGHTNESS_FIXED}
#define A2S_APA102_COLOR_MAGENTA {255, 0, 255, A2S_APA102_GLOBAL_BRIGHTNESS_FIXED}
#define A2S_APA102_COLOR_WHITE   {255, 255, 255, A2S_APA102_GLOBAL_BRIGHTNESS_FIXED}

// ----------------------------------------------------------------------------
// Function Prototypes
// ----------------------------------------------------------------------------

// Initialization and deinitialization
esp_err_t a2s_apa102_init(uint8_t num_leds, int mosi_pin, int clk_pin);
esp_err_t a2s_apa102_deinit(void);

// Direct LED control (immediate, stops any active effects)
esp_err_t a2s_apa102_set_led(uint8_t index, a2s_apa102_color_t color);
esp_err_t a2s_apa102_set_all_leds(a2s_apa102_color_t color);
esp_err_t a2s_apa102_clear_all(void);

// Effect control
esp_err_t a2s_apa102_set_effect(uint8_t index, a2s_apa102_status_mode_t mode, a2s_apa102_color_t color, float brightness_scale);
esp_err_t a2s_apa102_stop_effect(uint8_t index);

// Temperature-based color scaling (Blue → Yellow → Orange → Red)
a2s_apa102_color_t a2s_apa102_temperature_to_color(float min_temp, float max_temp, float actual_temp);
esp_err_t a2s_apa102_set_temperature_color(uint8_t led_index, float min_temp, float max_temp, float actual_temp);

// Utility functions
esp_err_t a2s_apa102_set_multiple(uint8_t *indices, uint8_t count, a2s_apa102_color_t color);

// ----------------------------------------------------------------------------
// end of a2s_apa102.h
// ----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // __A2S_APA102_H__
