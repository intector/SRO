/**
 ******************************************************************************
 * @file           : a2s_we2812.c
 * @brief          : A2S - WS2812 Status LED
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
#include "a2s_ws2812.h"
#include "main.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "A2S_WS2812";

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------

static rmt_channel_handle_t rmt_chan = NULL;
static rmt_encoder_handle_t ws2812_encoder = NULL;
static rmt_transmit_config_t tx_config = {
    .loop_count = 0, // no loop
};
static bool ws2812_initialized = false;

// ----------------------------------------------------------------------------
// WS2812 RMT Encoder
// ----------------------------------------------------------------------------

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_ws2812_encoder_t;

static size_t rmt_encode_ws2812(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = ws2812_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = ws2812_encoder->copy_encoder;
    rmt_encode_state_t session_state = 0;
    rmt_encode_state_t state = 0;
    size_t encoded_symbols = 0;

    switch (ws2812_encoder->state) {
        case 0: // send RGB data
            encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                ws2812_encoder->state = 1; // next state
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state |= RMT_ENCODING_MEM_FULL;
                goto out; // yield if there's no free space to put other encoding artifacts
            }
        // fall-through
        case 1: // send reset code
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &ws2812_encoder->reset_code,
                                                    sizeof(ws2812_encoder->reset_code), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                ws2812_encoder->state = 0; // next state
                state |= RMT_ENCODING_COMPLETE;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state |= RMT_ENCODING_MEM_FULL;
                goto out; // yield if there's no free space to put other encoding artifacts
            }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_ws2812_encoder(rmt_encoder_t *encoder)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_del_encoder(ws2812_encoder->bytes_encoder);
    rmt_del_encoder(ws2812_encoder->copy_encoder);
    free(ws2812_encoder);
    return ESP_OK;
}

static esp_err_t rmt_ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_encoder_reset(ws2812_encoder->bytes_encoder);
    rmt_encoder_reset(ws2812_encoder->copy_encoder);
    ws2812_encoder->state = 0;
    return ESP_OK;
}

static esp_err_t rmt_new_ws2812_encoder(const void *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_ws2812_encoder_t *ws2812_encoder = NULL;

    ws2812_encoder = calloc(1, sizeof(rmt_ws2812_encoder_t));
    if (!ws2812_encoder) {
        return ESP_ERR_NO_MEM;
    }

    ws2812_encoder->base.encode = rmt_encode_ws2812;
    ws2812_encoder->base.del = rmt_del_ws2812_encoder;
    ws2812_encoder->base.reset = rmt_ws2812_encoder_reset;

    // Different RMT symbols for 0 and 1 bits
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 0.35 * 80, // T0H = 350ns * 80MHz = 28 ticks
            .level1 = 0,
            .duration1 = 0.8 * 80, // T0L = 800ns * 80MHz = 64 ticks
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 0.7 * 80, // T1H = 700ns * 80MHz = 56 ticks
            .level1 = 0,
            .duration1 = 0.6 * 80, // T1L = 600ns * 80MHz = 48 ticks
        },
        .flags.msb_first = 1, // WS2812 uses MSB first
    };

    ret = rmt_new_bytes_encoder(&bytes_encoder_config, &ws2812_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        goto err;
    }

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &ws2812_encoder->copy_encoder);
    if (ret != ESP_OK) {
        goto err;
    }

    uint32_t reset_ticks = 80000000 / 1000000 * 300; // 300us reset time
    ws2812_encoder->reset_code = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = 0,
    };

    *ret_encoder = &ws2812_encoder->base;
    return ESP_OK;

err:
    if (ws2812_encoder) {
        if (ws2812_encoder->bytes_encoder) {
            rmt_del_encoder(ws2812_encoder->bytes_encoder);
        }
        if (ws2812_encoder->copy_encoder) {
            rmt_del_encoder(ws2812_encoder->copy_encoder);
        }
        free(ws2812_encoder);
    }
    return ret;
}

// ----------------------------------------------------------------------------
// Private Functions
// ----------------------------------------------------------------------------

// brightness calculation with gamma correction:
static void apply_brightness(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t brightness)
{
    if (brightness == 0) {
        *r = *g = *b = 0;
        return;
    }

    // Store original values for debug logging
    uint8_t orig_r = *r;
    uint8_t orig_g = *g; 
    uint8_t orig_b = *b;

    // Gamma correction for more natural brightness perception
    // Using gamma ~2.2 approximation
    float brightness_factor = (float)brightness / 255.0f;
    brightness_factor = brightness_factor * brightness_factor; // Square for gamma ~2.0
    
    *r = (uint8_t)(*r * brightness_factor);
    *g = (uint8_t)(*g * brightness_factor);
    *b = (uint8_t)(*b * brightness_factor);

    ESP_LOGD(TAG, "Brightness: %d/255, RGB: (%d,%d,%d) -> (%d,%d,%d)", 
             brightness, orig_r, orig_g, orig_b, *r, *g, *b);
}

// ----------------------------------------------------------------------------
// Public Functions
// ----------------------------------------------------------------------------

esp_err_t a2s_ws2812_init(void)
{
    if (ws2812_initialized) {
        ESP_LOGW(TAG, "WS2812 already initialized");
        return ESP_OK;
    }


    // Configure RMT TX channel
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = SRO_GPIO_WS2812_BOARD_LED,
        .mem_block_symbols = 64,
        .resolution_hz = 80000000, // 80MHz resolution
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false,
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &rmt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create WS2812 encoder
    ret = rmt_new_ws2812_encoder(NULL, &ws2812_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create WS2812 encoder: %s", esp_err_to_name(ret));
        rmt_del_channel(rmt_chan);
        rmt_chan = NULL;
        return ret;
    }

    // Enable RMT channel
    ret = rmt_enable(rmt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        rmt_del_encoder(ws2812_encoder);
        rmt_del_channel(rmt_chan);
        rmt_chan = NULL;
        ws2812_encoder = NULL;
        return ret;
    }

    ws2812_initialized = true;

    // Turn off LED initially
    a2s_ws2812_off();

    return ESP_OK;
}

esp_err_t a2s_ws2812_deinit(void)
{
    if (!ws2812_initialized) {
        return ESP_OK;
    }

    if (ws2812_encoder) {
        rmt_del_encoder(ws2812_encoder);
        ws2812_encoder = NULL;
    }

    if (rmt_chan) {
        rmt_disable(rmt_chan);
        rmt_del_channel(rmt_chan);
        rmt_chan = NULL;
    }

    ws2812_initialized = false;
    return ESP_OK;
}

esp_err_t a2s_ws2812_set_boot_status(a2s_ws2812_boot_status_t status)
{
    if (!ws2812_initialized) {
        ESP_LOGW(TAG, "WS2812 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_OK;

    switch (status) {
        case A2S_WS2812_STATUS_OFF:
            ESP_LOGD(TAG, "Boot status: OFF");
            ret = a2s_ws2812_off();
            break;

        case A2S_WS2812_STATUS_NORMAL_BOOT:
        case A2S_WS2812_STATUS_OOB_COMPLETE:
            ESP_LOGD(TAG, "Boot status: GREEN (normal/complete)");
            ret = a2s_ws2812_set_rgb(0, 255, 0, A2S_WS2812_BRIGHTNESS_25);
            break;

        case A2S_WS2812_STATUS_WAITING_RELEASE:
            ESP_LOGD(TAG, "Boot status: YELLOW (waiting for button release)");
            ret = a2s_ws2812_set_rgb(255, 255, 0, A2S_WS2812_BRIGHTNESS_25);
            break;

        case A2S_WS2812_STATUS_OOB_EXECUTING:
            ESP_LOGD(TAG, "Boot status: RED (OOB executing)");
            ret = a2s_ws2812_set_rgb(255, 0, 0, A2S_WS2812_BRIGHTNESS_25);
            break;

        default:
            ESP_LOGW(TAG, "Unknown boot status: %d", status);
            ret = ESP_ERR_INVALID_ARG;
            break;
    }

    return ret;
}

esp_err_t a2s_ws2812_set_color(a2s_ws2812_rgb_t color, uint8_t brightness)
{
    return a2s_ws2812_set_rgb(color.r, color.g, color.b, brightness);
}

esp_err_t a2s_ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (!ws2812_initialized || !rmt_chan || !ws2812_encoder) {
        ESP_LOGW(TAG, "WS2812 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Apply brightness scaling
    apply_brightness(&r, &g, &b, brightness);

    // WS2812 expects GRB format, not RGB
    uint8_t led_data[3] = {g, r, b};

    // Transmit data
    esp_err_t ret = rmt_transmit(rmt_chan, ws2812_encoder, led_data, sizeof(led_data), &tx_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit WS2812 data: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t a2s_ws2812_off(void)
{
    return a2s_ws2812_set_rgb(0, 0, 0, 0);
}

void a2s_ws2812_set_led(bool state)
{
    if (!ws2812_initialized) {
        ESP_LOGW(TAG, "WS2812 not initialized, ignoring LED command");
        return;
    }

    if (state) {
        a2s_ws2812_set_rgb(0, 255, 0, A2S_WS2812_BRIGHTNESS_25); // Green at 25% brightness
    } else {
        a2s_ws2812_off();
    }
}
