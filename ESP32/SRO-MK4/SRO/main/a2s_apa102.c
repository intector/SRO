/**
 ******************************************************************************
 * @file           : a2s_apa102.c
 * @brief          : A2S - APA102 Status LED Driver
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
// Includes -------------------------------------------------------------------
#include "a2s_apa102.h"

static const char *TAG = "A2S_APA102";

// ----------------------------------------------------------------------------
// Internal Type Definitions
// ----------------------------------------------------------------------------

// Command types for queue
typedef enum
{
    CMD_SET_LED,
    CMD_SET_ALL_LEDS,
    CMD_CLEAR_ALL,
    CMD_SET_EFFECT,
    CMD_STOP_EFFECT,
    CMD_SHUTDOWN
} a2s_apa102_cmd_type_t;

// Command structure
typedef struct
{
    a2s_apa102_cmd_type_t type;
    uint8_t led_index;
    a2s_apa102_color_t color;
    a2s_apa102_status_mode_t mode;
} a2s_apa102_cmd_t;

// Effect state per LED
typedef struct
{
    a2s_apa102_status_mode_t mode;
    a2s_apa102_color_t target_color;
    uint32_t phase;      // Animation phase counter in milliseconds
    float current_value; // Current brightness/fade value (0.0-1.0)
    bool active;         // Effect is active
} a2s_apa102_effect_state_t;

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------

// Hardware configuration
static uint8_t num_leds = 0;
static int mosi_pin     = -1;
static int clk_pin      = -1;

// Buffers (dynamically allocated)
static a2s_apa102_pixel_t *current_pixels      = NULL;
static a2s_apa102_effect_state_t *effect_state = NULL;
static uint8_t *spi_buffer                     = NULL;
static size_t spi_buffer_size                  = 0;

// SPI and task handles
static spi_device_handle_t spi_device  = NULL;
static TaskHandle_t effect_task_handle = NULL;
static QueueHandle_t cmd_queue         = NULL;

// Driver state
static bool driver_initialized = false;

// ----------------------------------------------------------------------------
// Private Function Prototypes
// ----------------------------------------------------------------------------

static a2s_apa102_color_t apply_rgb_compensation(a2s_apa102_color_t input_color);
static uint8_t brightness_to_apa102(uint8_t brightness_percent);
static a2s_apa102_pixel_t color_to_pixel(a2s_apa102_color_t color);
static a2s_apa102_color_t scale_color_brightness(a2s_apa102_color_t color, float scale);
static void build_spi_buffer(void);
static esp_err_t transmit_spi_buffer(void);
static void update_breathing_effect(uint8_t index);
static void update_blink_effect(uint8_t index, uint32_t period_ms);
static void update_fade_effect(uint8_t index, bool fade_in);
static void update_all_effects(void);
static void handle_command(a2s_apa102_cmd_t *cmd);
static void effect_task(void *pvParameters);

// ----------------------------------------------------------------------------
// Private Functions
// ----------------------------------------------------------------------------

static a2s_apa102_color_t apply_rgb_compensation(a2s_apa102_color_t input_color)
{
    if (!A2S_APA102_ENABLE_RGB_COMPENSATION) {
        return input_color;
    }

    a2s_apa102_color_t compensated = {
        .red        = (uint8_t)fminf(255.0f, input_color.red * A2S_APA102_RED_COMPENSATION),
        .green      = (uint8_t)fminf(255.0f, input_color.green * A2S_APA102_GREEN_COMPENSATION),
        .blue       = (uint8_t)fminf(255.0f, input_color.blue * A2S_APA102_BLUE_COMPENSATION),
        .brightness = input_color.brightness};

    return compensated;
}

static uint8_t brightness_to_apa102(uint8_t brightness_percent)
{
    // Always use maximum global brightness for smoothest PWM operation
    // Dimming is handled by scaling RGB values
    return 0xE0 | A2S_APA102_GLOBAL_BRIGHTNESS_FIXED;
}

static a2s_apa102_pixel_t color_to_pixel(a2s_apa102_color_t color)
{
    // Apply RGB brightness compensation
    a2s_apa102_color_t compensated_color = apply_rgb_compensation(color);

    a2s_apa102_pixel_t pixel             = {
                    .brightness = brightness_to_apa102(compensated_color.brightness),
                    .red        = compensated_color.red,
                    .green      = compensated_color.green,
                    .blue       = compensated_color.blue};
    return pixel;
}

static a2s_apa102_color_t scale_color_brightness(a2s_apa102_color_t color, float scale)
{
    // Clamp scale to 0.0-1.0
    if (scale < 0.0f)
        scale = 0.0f;
    if (scale > 1.0f)
        scale = 1.0f;

    a2s_apa102_color_t scaled = {
        .red        = (uint8_t)(color.red * scale),
        .green      = (uint8_t)(color.green * scale),
        .blue       = (uint8_t)(color.blue * scale),
        .brightness = color.brightness};
    return scaled;
}

static void build_spi_buffer(void)
{
    if (!spi_buffer || !current_pixels) {
        return;
    }

    uint32_t offset = 0;

    // Start frame (4 bytes of 0x00)
    memset(spi_buffer + offset, 0x00, A2S_APA102_START_FRAME_SIZE);
    offset += A2S_APA102_START_FRAME_SIZE;

    // LED data
    for (int i = 0; i < num_leds; i++) {
        spi_buffer[offset++] = current_pixels[i].brightness;
        spi_buffer[offset++] = current_pixels[i].blue;
        spi_buffer[offset++] = current_pixels[i].green;
        spi_buffer[offset++] = current_pixels[i].red;
    }

    // End frame (4 bytes of 0xFF)
    memset(spi_buffer + offset, 0xFF, A2S_APA102_END_FRAME_SIZE);
}

static esp_err_t transmit_spi_buffer(void)
{
    if (!spi_device || !spi_buffer) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t transaction = {
        .length    = spi_buffer_size * 8, // Length in bits
        .tx_buffer = spi_buffer,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(spi_device, &transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmission failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static void update_breathing_effect(uint8_t index)
{
    a2s_apa102_effect_state_t *state = &effect_state[index];

    // Increment phase
    state->phase += A2S_APA102_EFFECT_UPDATE_RATE_MS;
    if (state->phase >= A2S_APA102_BREATHING_PERIOD_MS) {
        state->phase = 0;
    }

    // Calculate sine wave for smooth breathing (0.0 to 1.0)
    float normalized_phase = (float)state->phase / A2S_APA102_BREATHING_PERIOD_MS;
    state->current_value   = (sinf(normalized_phase * 2.0f * M_PI) + 1.0f) / 2.0f;

    // Scale minimum brightness to 10% to keep LED visible
    state->current_value = 0.1f + (state->current_value * 0.9f);

    // Apply to LED
    a2s_apa102_color_t scaled_color = scale_color_brightness(state->target_color, state->current_value);
    current_pixels[index]           = color_to_pixel(scaled_color);
}

static void update_blink_effect(uint8_t index, uint32_t period_ms)
{
    a2s_apa102_effect_state_t *state = &effect_state[index];

    // Increment phase
    state->phase += A2S_APA102_EFFECT_UPDATE_RATE_MS;
    if (state->phase >= period_ms) {
        state->phase = 0;
    }

    // Toggle on/off at half period
    bool led_on = (state->phase < (period_ms / 2));

    if (led_on) {
        current_pixels[index] = color_to_pixel(state->target_color);
    }
    else {
        a2s_apa102_color_t off = A2S_APA102_COLOR_OFF;
        current_pixels[index]  = color_to_pixel(off);
    }
}

static void update_fade_effect(uint8_t index, bool fade_in)
{
    a2s_apa102_effect_state_t *state = &effect_state[index];

    // Increment phase
    state->phase += A2S_APA102_EFFECT_UPDATE_RATE_MS;

    // Calculate fade progress (0.0 to 1.0)
    float progress = (float)state->phase / A2S_APA102_FADE_DURATION_MS;
    if (progress > 1.0f) {
        progress      = 1.0f;
        state->active = false; // One-shot effect completed
    }

    // Apply fade direction
    state->current_value = fade_in ? progress : (1.0f - progress);

    // Apply to LED
    a2s_apa102_color_t scaled_color = scale_color_brightness(state->target_color, state->current_value);
    current_pixels[index]           = color_to_pixel(scaled_color);

    // When fade completes, set to final state
    if (!state->active) {
        if (fade_in) {
            current_pixels[index] = color_to_pixel(state->target_color);
            state->mode           = A2S_APA102_STATUS_SOLID;
        }
        else {
            a2s_apa102_color_t off = A2S_APA102_COLOR_OFF;
            current_pixels[index]  = color_to_pixel(off);
            state->mode            = A2S_APA102_STATUS_OFF;
        }
    }
}

static void update_all_effects(void)
{
    if (!effect_state || !current_pixels) {
        return;
    }

    for (int i = 0; i < num_leds; i++) {
        if (!effect_state[i].active) {
            continue;
        }

        switch (effect_state[i].mode) {
            case A2S_APA102_STATUS_BREATHING:
                update_breathing_effect(i);
                break;

            case A2S_APA102_STATUS_BLINK_SLOW:
                update_blink_effect(i, A2S_APA102_BLINK_SLOW_PERIOD_MS);
                break;

            case A2S_APA102_STATUS_BLINK_FAST:
                update_blink_effect(i, A2S_APA102_BLINK_FAST_PERIOD_MS);
                break;

            case A2S_APA102_STATUS_FADE_IN:
                update_fade_effect(i, true);
                break;

            case A2S_APA102_STATUS_FADE_OUT:
                update_fade_effect(i, false);
                break;

            default:
                // No effect update needed for SOLID or OFF
                break;
        }
    }
}

static void handle_command(a2s_apa102_cmd_t *cmd)
{
    if (!cmd) {
        return;
    }

    switch (cmd->type) {
        case CMD_SET_LED:
            if (cmd->led_index < num_leds) {
                // Stop any active effect
                effect_state[cmd->led_index].active = false;
                effect_state[cmd->led_index].mode   = A2S_APA102_STATUS_SOLID;

                // Set LED immediately
                current_pixels[cmd->led_index] = color_to_pixel(cmd->color);
            }
            break;

        case CMD_SET_ALL_LEDS:
            for (int i = 0; i < num_leds; i++) {
                effect_state[i].active = false;
                effect_state[i].mode   = A2S_APA102_STATUS_SOLID;
                current_pixels[i]      = color_to_pixel(cmd->color);
            }
            break;

        case CMD_CLEAR_ALL: {
            a2s_apa102_color_t off = A2S_APA102_COLOR_OFF;
            for (int i = 0; i < num_leds; i++) {
                effect_state[i].active = false;
                effect_state[i].mode   = A2S_APA102_STATUS_OFF;
                current_pixels[i]      = color_to_pixel(off);
            }
        } break;

        case CMD_SET_EFFECT:
            if (cmd->led_index < num_leds) {
                effect_state[cmd->led_index].mode          = cmd->mode;
                effect_state[cmd->led_index].target_color  = cmd->color;
                effect_state[cmd->led_index].phase         = 0;
                effect_state[cmd->led_index].current_value = 0.0f;

                // Enable effect if not OFF or SOLID
                if (cmd->mode != A2S_APA102_STATUS_OFF && cmd->mode != A2S_APA102_STATUS_SOLID) {
                    effect_state[cmd->led_index].active = true;
                }
                else {
                    effect_state[cmd->led_index].active = false;
                    // Set immediate state for SOLID or OFF
                    if (cmd->mode == A2S_APA102_STATUS_SOLID) {
                        current_pixels[cmd->led_index] = color_to_pixel(cmd->color);
                    }
                    else {
                        a2s_apa102_color_t off         = A2S_APA102_COLOR_OFF;
                        current_pixels[cmd->led_index] = color_to_pixel(off);
                    }
                }
            }
            break;

        case CMD_STOP_EFFECT:
            if (cmd->led_index < num_leds) {
                effect_state[cmd->led_index].active = false;
                effect_state[cmd->led_index].mode   = A2S_APA102_STATUS_SOLID;
            }
            break;

        case CMD_SHUTDOWN:
            // Signal handled in effect_task
            break;

        default:
            ESP_LOGW(TAG, "Unknown command type: %d", cmd->type);
            break;
    }
}

static void effect_task(void *pvParameters)
{
    a2s_apa102_cmd_t cmd;
    bool running = true;

    ESP_LOGI(TAG, "Effect task started");

    while (running) {
        // ONLY check suspend if event groups exist
        if (sro_ctrl2_events != NULL && sro_status2_events != NULL) {
            EventBits_t bits = xEventGroupGetBits(sro_ctrl2_events);
            if (bits & SRO_CE2_SUSPEND_APA102) {
                ESP_LOGI("APA102", "SUSPENDING");

                xEventGroupSetBits(sro_status2_events, SRO_SE2_APA102_SUSPENDED);

                // Wait for resume (flag to be CLEARED)
                while (xEventGroupGetBits(sro_ctrl2_events) & SRO_CE2_SUSPEND_APA102) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                xEventGroupClearBits(sro_status2_events, SRO_SE2_APA102_SUSPENDED);
                ESP_LOGI("APA102", "RESUMED");
                continue;
            }
        }
        
        // Process all pending commands (non-blocking)
        while (xQueueReceive(cmd_queue, &cmd, 0) == pdTRUE) {
            if (cmd.type == CMD_SHUTDOWN) {
                running = false;
                break;
            }
            handle_command(&cmd);
        }

        if (!running) {
            break;
        }

        // Update all active effects
        update_all_effects();

        // Build SPI buffer and transmit
        build_spi_buffer();
        transmit_spi_buffer();

        // Wait for next update cycle
        vTaskDelay(pdMS_TO_TICKS(A2S_APA102_EFFECT_UPDATE_RATE_MS));
    }

    ESP_LOGI(TAG, "Effect task stopped");
    vTaskDelete(NULL);
}

// ----------------------------------------------------------------------------
// Public Functions
// ----------------------------------------------------------------------------

esp_err_t a2s_apa102_init(uint8_t num_leds_param, int mosi_pin_param, int clk_pin_param)
{
    if (driver_initialized) {
        ESP_LOGW(TAG, "APA102 already initialized");
        return ESP_OK;
    }

    if (num_leds_param == 0 || num_leds_param > 255) {
        ESP_LOGE(TAG, "Invalid number of LEDs: %d", num_leds_param);
        return ESP_ERR_INVALID_ARG;
    }

    // Store configuration
    num_leds = num_leds_param;
    mosi_pin = mosi_pin_param;
    clk_pin  = clk_pin_param;

    // Calculate SPI buffer size
    spi_buffer_size = A2S_APA102_START_FRAME_SIZE +
                      (num_leds * A2S_APA102_BYTES_PER_LED) +
                      A2S_APA102_END_FRAME_SIZE;

    // Allocate buffers
    current_pixels = (a2s_apa102_pixel_t *)calloc(num_leds, sizeof(a2s_apa102_pixel_t));
    effect_state   = (a2s_apa102_effect_state_t *)calloc(num_leds, sizeof(a2s_apa102_effect_state_t));
    spi_buffer     = (uint8_t *)calloc(spi_buffer_size, sizeof(uint8_t));

    if (!current_pixels || !effect_state || !spi_buffer) {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        free(current_pixels);
        free(effect_state);
        free(spi_buffer);
        return ESP_ERR_NO_MEM;
    }

    // Initialize all LEDs to off
    a2s_apa102_color_t off_color = A2S_APA102_COLOR_OFF;
    for (int i = 0; i < num_leds; i++) {
        current_pixels[i]      = color_to_pixel(off_color);
        effect_state[i].mode   = A2S_APA102_STATUS_OFF;
        effect_state[i].active = false;
    }

    // Configure SPI bus
    spi_bus_config_t bus_config = {
        .miso_io_num     = -1, // APA102 is output only
        .mosi_io_num     = mosi_pin,
        .sclk_io_num     = clk_pin,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = spi_buffer_size,
    };

    // Configure SPI device
    spi_device_interface_config_t device_config = {
        .clock_speed_hz = A2S_APA102_SPI_FREQUENCY,
        .mode           = 0,  // SPI mode 0 (CPOL=0, CPHA=0)
        .spics_io_num   = -1, // No CS pin needed
        .queue_size     = 1,
        .flags          = 0,
    };

    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        free(current_pixels);
        free(effect_state);
        free(spi_buffer);
        return ret;
    }

    // Add device to SPI bus
    ret = spi_bus_add_device(SPI3_HOST, &device_config, &spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(SPI3_HOST);
        free(current_pixels);
        free(effect_state);
        free(spi_buffer);
        return ret;
    }

    // Create command queue
    cmd_queue = xQueueCreate(A2S_APA102_CMD_QUEUE_DEPTH, sizeof(a2s_apa102_cmd_t));
    if (!cmd_queue) {
        ESP_LOGE(TAG, "Failed to create command queue");
        spi_bus_remove_device(spi_device);
        spi_bus_free(SPI3_HOST);
        free(current_pixels);
        free(effect_state);
        free(spi_buffer);
        return ESP_ERR_NO_MEM;
    }

    // Send initial clear to LEDs
    build_spi_buffer();
    transmit_spi_buffer();

    // Create effect task
    BaseType_t task_ret = xTaskCreatePinnedToCore(effect_task,
                                                  "apa102_effects",
                                                  A2S_APA102_EFFECT_TASK_STACK,
                                                  NULL,
                                                  A2S_APA102_EFFECT_TASK_PRIORITY,
                                                  &effect_task_handle,
                                                  A2S_APA102_EFFECT_TASK_CORE);

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create effect task");
        vQueueDelete(cmd_queue);
        spi_bus_remove_device(spi_device);
        spi_bus_free(SPI3_HOST);
        free(current_pixels);
        free(effect_state);
        free(spi_buffer);
        return ESP_FAIL;
    }

    driver_initialized = true;
    ESP_LOGI(TAG, "APA102 driver initialized: %d LEDs", num_leds);

    return ESP_OK;
}

esp_err_t a2s_apa102_deinit(void)
{
    if (!driver_initialized) {
        return ESP_OK;
    }

    // Send shutdown command to effect task
    a2s_apa102_cmd_t cmd = {.type = CMD_SHUTDOWN};
    xQueueSend(cmd_queue, &cmd, pdMS_TO_TICKS(100));

    // Wait for task to finish
    vTaskDelay(pdMS_TO_TICKS(200));

    // Clean up queue
    if (cmd_queue) {
        vQueueDelete(cmd_queue);
        cmd_queue = NULL;
    }

    // Clear all LEDs
    a2s_apa102_color_t off = A2S_APA102_COLOR_OFF;
    for (int i = 0; i < num_leds; i++) {
        current_pixels[i] = color_to_pixel(off);
    }
    build_spi_buffer();
    transmit_spi_buffer();

    // Clean up SPI
    if (spi_device) {
        spi_bus_remove_device(spi_device);
        spi_device = NULL;
    }
    spi_bus_free(SPI3_HOST);

    // Free buffers
    free(current_pixels);
    free(effect_state);
    free(spi_buffer);
    current_pixels     = NULL;
    effect_state       = NULL;
    spi_buffer         = NULL;

    driver_initialized = false;
    effect_task_handle = NULL;

    ESP_LOGI(TAG, "APA102 driver deinitialized");

    return ESP_OK;
}

esp_err_t a2s_apa102_set_led(uint8_t index, a2s_apa102_color_t color)
{
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (index >= num_leds) {
        return ESP_ERR_INVALID_ARG;
    }

    a2s_apa102_cmd_t cmd = {
        .type      = CMD_SET_LED,
        .led_index = index,
        .color     = color};

    if (xQueueSend(cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t a2s_apa102_set_all_leds(a2s_apa102_color_t color)
{
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    a2s_apa102_cmd_t cmd = {
        .type  = CMD_SET_ALL_LEDS,
        .color = color};

    if (xQueueSend(cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t a2s_apa102_clear_all(void)
{
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    a2s_apa102_cmd_t cmd = {
        .type = CMD_CLEAR_ALL};

    if (xQueueSend(cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t a2s_apa102_set_effect(uint8_t index, a2s_apa102_status_mode_t mode, a2s_apa102_color_t color, float brightness_scale)
{
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (index >= num_leds) {
        return ESP_ERR_INVALID_ARG;
    }

    a2s_apa102_color_t color_brightness = scale_color_brightness(color, brightness_scale);

    // LED color command
    a2s_apa102_cmd_t cmd = {
                       .type      = CMD_SET_EFFECT,
                       .led_index = index,
                       .mode      = mode,
                       .color     = color_brightness};

    if (xQueueSend(cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t a2s_apa102_stop_effect(uint8_t index)
{
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (index >= num_leds) {
        return ESP_ERR_INVALID_ARG;
    }

    a2s_apa102_cmd_t cmd = {
        .type      = CMD_STOP_EFFECT,
        .led_index = index};

    if (xQueueSend(cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

a2s_apa102_color_t a2s_apa102_temperature_to_color(float min_temp, float max_temp, float actual_temp)
{
    a2s_apa102_color_t color;
    color.brightness = A2S_APA102_GLOBAL_BRIGHTNESS_FIXED;

    // Clamp temperature to range
    if (actual_temp <= min_temp) {
        // Blue
        color.red   = 0;
        color.green = 0;
        color.blue  = 255;
        return color;
    }
    if (actual_temp >= max_temp) {
        // Red
        color.red   = 255;
        color.green = 0;
        color.blue  = 0;
        return color;
    }

    // Normalize to 0.0 - 1.0
    float normalized = (actual_temp - min_temp) / (max_temp - min_temp);

    // Three segments: Blue→Yellow (0-0.33), Yellow→Orange (0.33-0.66), Orange→Red (0.66-1.0)
    if (normalized < 0.333f) {
        // Blue {0,0,255} → Yellow {255,255,0}
        float t     = normalized / 0.333f;
        color.red   = (uint8_t)(255.0f * t);
        color.green = (uint8_t)(255.0f * t);
        color.blue  = (uint8_t)(255.0f * (1.0f - t));
    }
    else if (normalized < 0.666f) {
        // Yellow {255,255,0} → Orange {255,128,0}
        float t     = (normalized - 0.333f) / 0.333f;
        color.red   = 255;
        color.green = (uint8_t)(255.0f - 127.0f * t);
        color.blue  = 0;
    }
    else {
        // Orange {255,128,0} → Red {255,0,0}
        float t     = (normalized - 0.666f) / 0.334f;
        color.red   = 255;
        color.green = (uint8_t)(128.0f * (1.0f - t));
        color.blue  = 0;
    }

    return color;
}

esp_err_t a2s_apa102_set_temperature_color(uint8_t led_index, float min_temp, float max_temp, float actual_temp)
{
    a2s_apa102_color_t color = a2s_apa102_temperature_to_color(min_temp, max_temp, actual_temp);
    return a2s_apa102_set_led(led_index, color);
}

esp_err_t a2s_apa102_set_multiple(uint8_t *indices, uint8_t count, a2s_apa102_color_t color)
{
    if (!driver_initialized || !indices) {
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < count; i++) {
        esp_err_t ret = a2s_apa102_set_led(indices[i], color);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

// ----------------------------------------------------------------------------
// end of a2s_apa102.c
// ----------------------------------------------------------------------------
