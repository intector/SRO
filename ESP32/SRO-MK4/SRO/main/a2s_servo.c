/**
 ******************************************************************************
 * @file           : a2s_servo.c
 * @brief          : A2S Servo Motor Driver - Implementation
 ******************************************************************************
 *
 * Copyright (c) 2024-2030 Intector Inc.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

// Includes -------------------------------------------------------------------
#include "a2s_servo.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

// ----------------------------------------------------------------------------
// Private Type Definitions
// ----------------------------------------------------------------------------

typedef struct
{
    float current_position;     // Current position (0-100%)
    float target_position;      // Target position (0-100%)
    float current_velocity;     // Current velocity (%/second)
    float max_speed;            // Maximum speed (%/second)
    float acceleration;         // Acceleration (%/second²)
    uint32_t last_update_ms;    // Last physics update timestamp
    bool movement_active;       // Movement in progress flag
    uint32_t movement_start_ms; // Movement start timestamp (for timeout)
} smooth_control_t;

typedef struct
{
    a2s_servo_state_t state;        // Current state
    smooth_control_t smooth;        // Smooth movement control
    a2s_servo_config_t config;      // Configuration
    mcpwm_timer_handle_t timer;     // MCPWM timer handle
    mcpwm_oper_handle_t operator;   // MCPWM operator handle
    mcpwm_cmpr_handle_t comparator; // MCPWM comparator handle
    mcpwm_gen_handle_t generator;   // MCPWM generator handle
} servo_driver_t;

// ----------------------------------------------------------------------------
// Static Variables
// ----------------------------------------------------------------------------

static servo_driver_t g_servo            = {0};
static SemaphoreHandle_t g_mutex         = NULL;
static TaskHandle_t g_update_task_handle = NULL;
static bool g_initialized                = false;

// ----------------------------------------------------------------------------
// Static Function Prototypes
// ----------------------------------------------------------------------------

static void update_task(void *pvParameters);
static void smooth_update_internal(void);
static void set_target_position_internal(float target, a2s_servo_smoothness_t smoothness);
static void configure_smoothness_params(a2s_servo_smoothness_t smoothness);
static esp_err_t apply_position_to_hardware(uint8_t position_percent);
static void fire_callback(a2s_servo_event_t event, uint8_t position);
static esp_err_t init_mcpwm(uint8_t gpio_pin);
static void cleanup_mcpwm(void);

// ----------------------------------------------------------------------------
// Update Task - Runs every 50ms
// ----------------------------------------------------------------------------

static void update_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (g_initialized && xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (g_servo.smooth.movement_active) {
                smooth_update_internal();
            }
            xSemaphoreGive(g_mutex);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(g_servo.config.update_interval_ms));
    }
}

// ----------------------------------------------------------------------------
// Smooth Movement Physics Engine
// ----------------------------------------------------------------------------

static void smooth_update_internal(void)
{
    smooth_control_t *sm  = &g_servo.smooth;
    uint32_t current_time = (uint32_t)(esp_timer_get_time() / 1000);

    // Check for timeout
    if ((current_time - sm->movement_start_ms) > A2S_SERVO_MOVEMENT_TIMEOUT_MS) {
        sm->movement_active = false;
        g_servo.state       = A2S_SERVO_STATE_ERROR;
        fire_callback(A2S_SERVO_EVENT_TIMEOUT, (uint8_t)(sm->current_position + 0.5f));
        return;
    }

    // Calculate time delta
    float dt = (current_time - sm->last_update_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) {
        sm->last_update_ms = current_time;
        return;
    }
    sm->last_update_ms = current_time;

    // Calculate position error
    float error     = sm->target_position - sm->current_position;
    float abs_error = fabsf(error);

    // Check if at target
    if (abs_error < A2S_SERVO_POSITION_TOLERANCE) {
        sm->current_position = sm->target_position;
        sm->current_velocity = 0.0f;
        sm->movement_active  = false;
        g_servo.state        = A2S_SERVO_STATE_IDLE;

        apply_position_to_hardware((uint8_t)(sm->current_position + 0.5f));
        fire_callback(A2S_SERVO_EVENT_MOVE_COMPLETE, (uint8_t)(sm->current_position + 0.5f));
        return;
    }

    // Determine movement direction
    float direction = (error > 0.0f) ? 1.0f : -1.0f;

    // Calculate required deceleration distance
    float decel_distance = (sm->current_velocity * sm->current_velocity) / (2.0f * sm->acceleration);

    // Acceleration/deceleration logic
    if (abs_error > decel_distance) {
        // Accelerate towards max speed
        sm->current_velocity += sm->acceleration * dt * direction;

        // Clamp to max speed
        if (fabsf(sm->current_velocity) > sm->max_speed) {
            sm->current_velocity = sm->max_speed * direction;
        }
    }
    else {
        // Decelerate to stop at target
        sm->current_velocity -= sm->acceleration * dt * direction;

        // Prevent velocity sign change
        if ((direction > 0.0f && sm->current_velocity < 0.0f) ||
            (direction < 0.0f && sm->current_velocity > 0.0f)) {
            sm->current_velocity = 0.0f;
        }
    }

    // Update position
    sm->current_position += sm->current_velocity * dt;

    // Clamp position to valid range
    if (sm->current_position < A2S_SERVO_POSITION_MIN) {
        sm->current_position = A2S_SERVO_POSITION_MIN;
    }
    if (sm->current_position > A2S_SERVO_POSITION_MAX) {
        sm->current_position = A2S_SERVO_POSITION_MAX;
    }

    // Apply to hardware
    apply_position_to_hardware((uint8_t)(sm->current_position + 0.5f));
}

// ----------------------------------------------------------------------------
// Movement Control Functions
// ----------------------------------------------------------------------------

static void set_target_position_internal(float target, a2s_servo_smoothness_t smoothness)
{
    smooth_control_t *sm = &g_servo.smooth;

    // Clamp target to valid range
    if (target < A2S_SERVO_POSITION_MIN)
        target = A2S_SERVO_POSITION_MIN;
    if (target > A2S_SERVO_POSITION_MAX)
        target = A2S_SERVO_POSITION_MAX;

    // Configure smoothness parameters
    configure_smoothness_params(smoothness);

    // Set target
    sm->target_position   = target;
    sm->last_update_ms    = (uint32_t)(esp_timer_get_time() / 1000);
    sm->movement_start_ms = sm->last_update_ms;

    // Check if movement needed
    float error = fabsf(target - sm->current_position);
    if (error > A2S_SERVO_POSITION_TOLERANCE) {
        sm->movement_active = true;
        g_servo.state       = A2S_SERVO_STATE_MOVING;
    }
    else {
        sm->movement_active = false;
        g_servo.state       = A2S_SERVO_STATE_IDLE;
    }
}

static void configure_smoothness_params(a2s_servo_smoothness_t smoothness)
{
    smooth_control_t *sm = &g_servo.smooth;

    switch (smoothness) {
        case A2S_SERVO_SMOOTH_INSTANT:
            sm->max_speed    = A2S_SERVO_SMOOTH_INSTANT_SPEED;
            sm->acceleration = A2S_SERVO_SMOOTH_INSTANT_ACCEL;
            break;

        case A2S_SERVO_SMOOTH_FAST:
            sm->max_speed    = A2S_SERVO_SMOOTH_FAST_SPEED;
            sm->acceleration = A2S_SERVO_SMOOTH_FAST_ACCEL;
            break;

        case A2S_SERVO_SMOOTH_NORMAL:
            sm->max_speed    = A2S_SERVO_SMOOTH_NORMAL_SPEED;
            sm->acceleration = A2S_SERVO_SMOOTH_NORMAL_ACCEL;
            break;

        case A2S_SERVO_SMOOTH_SLOW:
            sm->max_speed    = A2S_SERVO_SMOOTH_SLOW_SPEED;
            sm->acceleration = A2S_SERVO_SMOOTH_SLOW_ACCEL;
            break;

        case A2S_SERVO_SMOOTH_VERY_SLOW:
            sm->max_speed    = A2S_SERVO_SMOOTH_VERY_SLOW_SPEED;
            sm->acceleration = A2S_SERVO_SMOOTH_VERY_SLOW_ACCEL;
            break;

        default:
            sm->max_speed    = A2S_SERVO_SMOOTH_NORMAL_SPEED;
            sm->acceleration = A2S_SERVO_SMOOTH_NORMAL_ACCEL;
            break;
    }
}

// ----------------------------------------------------------------------------
// Hardware Control Functions
// ----------------------------------------------------------------------------

static esp_err_t apply_position_to_hardware(uint8_t position_percent)
{
    if (position_percent > 100) {
        position_percent = 100;
    }

    // Convert percentage to pulse width
    uint32_t pulse_width = g_servo.config.min_pulse_us +
                           (position_percent * (g_servo.config.max_pulse_us -
                                                g_servo.config.min_pulse_us)) /
                               100;

    esp_err_t err = mcpwm_comparator_set_compare_value(g_servo.comparator, pulse_width);
    if (err != ESP_OK) {
        g_servo.state = A2S_SERVO_STATE_ERROR;
        fire_callback(A2S_SERVO_EVENT_ERROR, position_percent);
        return err;
    }

    return ESP_OK;
}

static esp_err_t init_mcpwm(uint8_t gpio_pin)
{
    esp_err_t err;

    // Create MCPWM timer
    mcpwm_timer_config_t timer_config = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1MHz, 1us per tick
        .period_ticks  = 20000,   // 20ms period for 50Hz
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };

    err = mcpwm_new_timer(&timer_config, &g_servo.timer);
    if (err != ESP_OK)
        return err;

    // Create MCPWM operator
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };

    err = mcpwm_new_operator(&operator_config, &g_servo.operator);
    if (err != ESP_OK)
        goto cleanup_timer;

    // Connect timer to operator
    err = mcpwm_operator_connect_timer(g_servo.operator, g_servo.timer);
    if (err != ESP_OK)
        goto cleanup_operator;

    // Create MCPWM comparator
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };

    err = mcpwm_new_comparator(g_servo.operator, & comparator_config, &g_servo.comparator);
    if (err != ESP_OK)
        goto cleanup_operator;

    // Create MCPWM generator
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio_pin,
    };

    err = mcpwm_new_generator(g_servo.operator, & generator_config, &g_servo.generator);
    if (err != ESP_OK)
        goto cleanup_comparator;

    // Set generator actions
    err = mcpwm_generator_set_action_on_timer_event(g_servo.generator,
                                                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                                 MCPWM_TIMER_EVENT_EMPTY,
                                                                                 MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK)
        goto cleanup_generator;

    err = mcpwm_generator_set_action_on_compare_event(g_servo.generator,
                                                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                                     g_servo.comparator,
                                                                                     MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK)
        goto cleanup_generator;

    // Enable and start timer
    err = mcpwm_timer_enable(g_servo.timer);
    if (err != ESP_OK)
        goto cleanup_generator;

    err = mcpwm_timer_start_stop(g_servo.timer, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK)
        goto cleanup_generator;

    return ESP_OK;

cleanup_generator:
    mcpwm_del_generator(g_servo.generator);
    g_servo.generator = NULL;
cleanup_comparator:
    mcpwm_del_comparator(g_servo.comparator);
    g_servo.comparator = NULL;
cleanup_operator:
    mcpwm_del_operator(g_servo.operator);
    g_servo.operator= NULL;
cleanup_timer:
    mcpwm_del_timer(g_servo.timer);
    g_servo.timer = NULL;

    return err;
}

static void cleanup_mcpwm(void)
{
    if (g_servo.timer) {
        mcpwm_timer_disable(g_servo.timer);
    }

    if (g_servo.generator) {
        mcpwm_del_generator(g_servo.generator);
        g_servo.generator = NULL;
    }

    if (g_servo.comparator) {
        mcpwm_del_comparator(g_servo.comparator);
        g_servo.comparator = NULL;
    }

    if (g_servo.operator) {
        mcpwm_del_operator(g_servo.operator);
        g_servo.operator= NULL;
    }

    if (g_servo.timer) {
        mcpwm_del_timer(g_servo.timer);
        g_servo.timer = NULL;
    }
}

// ----------------------------------------------------------------------------
// Event Callback
// ----------------------------------------------------------------------------

static void fire_callback(a2s_servo_event_t event, uint8_t position)
{
    if (g_servo.config.event_callback != NULL) {
        g_servo.config.event_callback(event, position, g_servo.config.callback_user_data);
    }
}

// ----------------------------------------------------------------------------
// Public API Implementation
// ----------------------------------------------------------------------------

a2s_servo_config_t a2s_servo_default_config(uint8_t gpio_pin)
{
    a2s_servo_config_t config = {
        .gpio_pin           = gpio_pin,
        .min_pulse_us       = A2S_SERVO_MIN_PULSE_US,
        .max_pulse_us       = A2S_SERVO_MAX_PULSE_US,
        .task_priority      = A2S_SERVO_TASK_PRIORITY,
        .task_stack_size    = A2S_SERVO_TASK_STACK_SIZE,
        .update_interval_ms = A2S_SERVO_UPDATE_INTERVAL_MS,
        .event_callback     = NULL,
        .callback_user_data = NULL};

    return config;
}

esp_err_t a2s_servo_init(const a2s_servo_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutex
    g_mutex = xSemaphoreCreateMutex();
    if (g_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Copy configuration
    memcpy(&g_servo.config, config, sizeof(a2s_servo_config_t));

    // Initialize MCPWM
    esp_err_t err = init_mcpwm(config->gpio_pin);
    if (err != ESP_OK) {
        vSemaphoreDelete(g_mutex);
        g_mutex = NULL;
        return err;
    }

    // Initialize state
    g_servo.state                   = A2S_SERVO_STATE_IDLE;
    g_servo.smooth.current_position = 0.0f;
    g_servo.smooth.target_position  = 0.0f;
    g_servo.smooth.current_velocity = 0.0f;
    g_servo.smooth.movement_active  = false;
    g_servo.smooth.max_speed        = A2S_SERVO_SMOOTH_NORMAL_SPEED;
    g_servo.smooth.acceleration     = A2S_SERVO_SMOOTH_NORMAL_ACCEL;

    // Set initial position to hardware
    apply_position_to_hardware(0);

    g_initialized = true;

    // Create update task
    BaseType_t ret = xTaskCreate(
        update_task,
        "servo_upd",
        config->task_stack_size,
        NULL,
        config->task_priority,
        &g_update_task_handle);

    if (ret != pdPASS) {
        g_initialized = false;
        cleanup_mcpwm();
        vSemaphoreDelete(g_mutex);
        g_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t a2s_servo_deinit(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Delete update task
    if (g_update_task_handle != NULL) {
        vTaskDelete(g_update_task_handle);
        g_update_task_handle = NULL;
    }

    // Cleanup MCPWM
    cleanup_mcpwm();

    // Delete mutex
    if (g_mutex != NULL) {
        vSemaphoreDelete(g_mutex);
        g_mutex = NULL;
    }

    g_initialized = false;
    memset(&g_servo, 0, sizeof(servo_driver_t));

    return ESP_OK;
}

esp_err_t a2s_servo_move_to(uint8_t position, a2s_servo_smoothness_t smoothness)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (position > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        set_target_position_internal((float)position, smoothness);
        xSemaphoreGive(g_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

uint8_t a2s_servo_get_position(void)
{
    if (!g_initialized) {
        return 0;
    }

    uint8_t position = 0;

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        position = (uint8_t)(g_servo.smooth.current_position + 0.5f);
        xSemaphoreGive(g_mutex);
    }

    return position;
}

bool a2s_servo_is_moving(void)
{
    if (!g_initialized) {
        return false;
    }

    bool moving = false;

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        moving = g_servo.smooth.movement_active;
        xSemaphoreGive(g_mutex);
    }

    return moving;
}

esp_err_t a2s_servo_stop(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_servo.smooth.target_position  = g_servo.smooth.current_position;
        g_servo.smooth.current_velocity = 0.0f;
        g_servo.smooth.movement_active  = false;
        g_servo.state                   = A2S_SERVO_STATE_IDLE;

        fire_callback(A2S_SERVO_EVENT_STOPPED,
                      (uint8_t)(g_servo.smooth.current_position + 0.5f));

        xSemaphoreGive(g_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

a2s_servo_state_t a2s_servo_get_state(void)
{
    if (!g_initialized) {
        return A2S_SERVO_STATE_UNINITIALIZED;
    }

    a2s_servo_state_t state = A2S_SERVO_STATE_UNINITIALIZED;

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        state = g_servo.state;
        xSemaphoreGive(g_mutex);
    }

    return state;
}

esp_err_t a2s_servo_set_callback(a2s_servo_callback_t callback, void *user_data)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_servo.config.event_callback     = callback;
        g_servo.config.callback_user_data = user_data;
        xSemaphoreGive(g_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}
