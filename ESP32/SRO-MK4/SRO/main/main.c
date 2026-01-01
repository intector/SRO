/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"

static const char *TAG = "SRO_MAIN";

// ----------------------------------------------------------------------------
// local variables
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Global Event Groups
// ----------------------------------------------------------------------------
EventGroupHandle_t sro_ctrl1_events   = NULL;
EventGroupHandle_t sro_ctrl2_events   = NULL;
EventGroupHandle_t sro_status1_events = NULL;
EventGroupHandle_t sro_status2_events = NULL;

// static functions prototypes ------------------------------------------------
static esp_err_t init_nvs(void);
static esp_err_t init_filesystem(void);
static esp_err_t init_peripherals(void);
static esp_err_t init_network_services(void);
static void servo_event_callback(a2s_servo_event_t event, uint8_t position, void *user_data);
static void log_heap_status(const char *label);
static void print_task_stack_watermarks(void);

// ----------------------------------------------------------------------------
// app_main function
// ----------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "=== SRO System Starting ===");

    // ------------------------------------------------------------------------
    // PHASE 1: Hardware Initialization
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 1: Hardware Init");
    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(init_filesystem());
    ESP_ERROR_CHECK(init_peripherals());
    // ESP_ERROR_CHECK(init_gpio());

    // ------------------------------------------------------------------------
    // PHASE 2: Create Event Groups
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 2: Event Groups");
    sro_ctrl1_events = xEventGroupCreate();
    sro_ctrl2_events = xEventGroupCreate();
    sro_status1_events = xEventGroupCreate();
    sro_status2_events = xEventGroupCreate();
    
    if (!sro_ctrl1_events || !sro_ctrl2_events || 
        !sro_status1_events || !sro_status2_events) {
        ESP_LOGE(TAG, "FATAL: Event group creation failed");
        esp_restart();
    }

    // NOW set the Phase 1 completion bits
    xEventGroupSetBits(sro_status2_events, SRO_SE2_NVS_INITIALIZED);
    xEventGroupSetBits(sro_status2_events, SRO_SE2_FILESYSTEM_MOUNTED);

    // CRITICAL: Clear the release flag (paranoid safety check)
    xEventGroupClearBits(sro_ctrl1_events, SRO_CE1_TASKS_RELEASE);

    // ------------------------------------------------------------------------
    // PHASE 3: Initialize Subsystems
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 3: Subsystem Init");
    // ESP_ERROR_CHECK(SRO_MemoryMonitor_Init());
    ESP_ERROR_CHECK(a2s_max6675_init());
    ESP_ERROR_CHECK(a2s_ssr_init());

    // Initialize servo with callback
    a2s_servo_config_t servo_config = a2s_servo_default_config(SRO_GPIO_SERVO_PWM);
    servo_config.event_callback     = servo_event_callback;
    servo_config.callback_user_data = NULL;
    servo_config.min_pulse_us       = 620;                                    // set to actual physical door min position
    servo_config.max_pulse_us       = 1850; // set to actual physical door max position

    ESP_ERROR_CHECK(a2s_servo_init(&servo_config));

    // ensure door is closed at beginning
    a2s_servo_move_to(50, A2S_SERVO_SMOOTH_FAST);

    while (a2s_servo_is_moving()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    a2s_servo_move_to(0, A2S_SERVO_SMOOTH_INSTANT);
    while (a2s_servo_is_moving()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_ERROR_CHECK(SRO_TemperatureControl_Init());
    ESP_ERROR_CHECK(SRO_ProfileManager_Init());

    // ------------------------------------------------------------------------
    // PHASE 4: Create Tasks (order doesn't matter now - they're all gated)
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "**************************************************");
    ESP_LOGI(TAG, "Phase 4: Task Creation");
    ESP_LOGI(TAG, "canceled for testing");
    ESP_LOGI(TAG, "**************************************************");

    print_task_stack_watermarks();
    log_heap_status("Before any tasks");
    // Can create in any order - all will block on TASKS_RELEASE

    // Heater PWM Control
    log_heap_status("Before heater_pwm task");
    if (xTaskCreatePinnedToCore(heater_pwm_task, "heater_pwm",
                                SRO_STACK_SIZE_HEATER_PWM, NULL, SRO_PRIORITY_HEATER_PWM, NULL,
                                SRO_CORE_REALTIME) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create heater_pwm_task");
        esp_restart();
    }
    log_heap_status("After heater_pwm task");

    // Temperature Control
    log_heap_status("Before temperature_control_task");
    if (xTaskCreatePinnedToCore(temperature_control_task, "temp_ctrl",
                                SRO_STACK_SIZE_TEMP_CONTROL, NULL, SRO_PRIORITY_TEMP_CONTROL, NULL,
                                SRO_CORE_REALTIME) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create temp_control_task");
        esp_restart();
    }
    log_heap_status("After temperature_control_task");

    // Profile Execution
    log_heap_status("Before profile_execution_task");
    if (xTaskCreatePinnedToCore(profile_execution_task, "profile",
                                SRO_STACK_SIZE_PROFILE, NULL, SRO_PRIORITY_PROFILE, NULL,
                                SRO_CORE_REALTIME) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create profile_task");
        esp_restart();
    }
    log_heap_status("After profile_execution_task");

    // System Coordinator
    log_heap_status("Before system_coordinator_task");
    if (xTaskCreatePinnedToCore(system_coordinator_task, "coordinator",
                                SRO_STACK_SIZE_COORDINATOR, NULL, SRO_PRIORITY_COORDINATOR, NULL,
                                SRO_CORE_BACKGROUND) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create coordinator_task");
        esp_restart();
    }
    log_heap_status("After system_coordinator_task");

    // WebSocket Broadcast
    log_heap_status("Before websocket_broadcast_task");
    if (xTaskCreatePinnedToCore(websocket_broadcast_task, "websocket",
                                SRO_STACK_SIZE_WEBSOCKET_BROADCAST, NULL, SRO_PRIORITY_WEBSOCKET_BROADCAST, NULL,
                                SRO_CORE_BACKGROUND) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create websocket_task");
        esp_restart();
    }
    log_heap_status("After websocket_broadcast_task");

    // WebSocket Broadcast Timer
    log_heap_status("Before websocket_broadcast_timer_task");
    if (xTaskCreatePinnedToCore(websocket_broadcast_timer_task, "ws_timer",
                                SRO_STACK_SIZE_BROADCAST_TIMER, NULL, SRO_PRIORITY_BROADCAST_TIMER, NULL,
                                SRO_CORE_BACKGROUND) != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create websocket_timer_task");
        esp_restart();
    }
    log_heap_status("After websocket_broadcast_timer_task");

    ESP_LOGI(TAG, "All tasks created (waiting at startup gate)");

    log_heap_status("After all tasks created");
    print_task_stack_watermarks();

    // Small delay to let all tasks reach their wait points
    vTaskDelay(pdMS_TO_TICKS(100));

    // ------------------------------------------------------------------------
    // PHASE 5: Network Services (these also wait internally if needed)
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 5: Network Services");

    // Initialize network services (WiFi, WebSocket, HTTP, etc.)
    ESP_ERROR_CHECK(init_network_services());

    // Register WebSocket command handler
    SRO_WebSocketServer_SetCommandHandler(websocket_command_handler);

    // ------------------------------------------------------------------------
    // PHASE 6: RELEASE ALL TASKS
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "Phase 6: System Ready");

    // Mark system as fully initialized
    xEventGroupSetBits(sro_status2_events, SRO_SE2_SYSTEM_INITIALIZED);

    // RELEASE THE KRAKEN!
    ESP_LOGI(TAG, "Releasing all tasks - system going operational");

    xEventGroupSetBits(sro_ctrl1_events, SRO_CE1_TASKS_RELEASE);

    ESP_LOGI(TAG, "=== System Startup Complete ===");

    // Set WS2812 LED to GREEN
    a2s_ws2812_set_rgb(0, 10, 0, 255);

    // Set APA102 LED to GREEN
    a2s_apa102_set_effect(0, A2S_APA102_STATUS_SOLID, (a2s_apa102_color_t)A2S_APA102_COLOR_GREEN, 0.20);

    // ------------------------------------------------------------------------
    // PHASE 7: app_main becomes idle/watchdog
    // ------------------------------------------------------------------------
    while (1) {
        // running evey minute
        vTaskDelay(pdMS_TO_TICKS(60000));

        // logging memory status
        print_task_stack_watermarks();
        ESP_LOGI(TAG, "Free heap: %lu bytes (min ever: %lu bytes)",
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size());
    }
}

// ----------------------------------------------------------------------------
// Initialization Functions
// ----------------------------------------------------------------------------

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // xEventGroupSetBits(sro_status2_events, SRO_SE2_NVS_INITIALIZED);
    return ESP_OK;
}

static esp_err_t init_filesystem(void)
{
    ESP_LOGI(TAG, "Initializing FAT filesystem...");

    esp_err_t ret;

    // Find the storage partition
    const esp_partition_t *data_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");

    if (!data_partition) {
        ESP_LOGE(TAG, "Failed to find 'storage' partition");
        ESP_LOGE(TAG, "Check partitions.csv includes:");
        ESP_LOGE(TAG, "  storage, data, fat, , 512K,");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Found storage partition at offset 0x%lx, size %lu bytes",
             data_partition->address, data_partition->size);

    // Configure FAT filesystem mount
    esp_vfs_fat_mount_config_t mount_config = {
        .max_files              = 10,   // Max 10 open files
        .format_if_mount_failed = true, // Auto-format if corrupted
        .allocation_unit_size   = CONFIG_WL_SECTOR_SIZE};

    // Mount with wear leveling
    wl_handle_t wl_handle = WL_INVALID_HANDLE;
    ret = esp_vfs_fat_spiflash_mount_rw_wl("/fatfs", "storage", &mount_config, &wl_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FAT filesystem: %s", esp_err_to_name(ret));
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Possible causes:");
            ESP_LOGE(TAG, "  1. Partition 'storage' not found");
            ESP_LOGE(TAG, "  2. Partition size too small");
            ESP_LOGE(TAG, "  3. Flash corruption");
        }
        return ret;
    }

    ESP_LOGI(TAG, "FAT filesystem mounted successfully at /fatfs");

    // Optional: Create default directories
    struct stat st;
    if (stat("/fatfs/web", &st) != 0) {
        ESP_LOGI(TAG, "Creating /fatfs/web directory");
        mkdir("/fatfs/web", 0775);
    }

    if (stat("/fatfs/profiles", &st) != 0) {
        ESP_LOGI(TAG, "Creating /fatfs/profiles directory");
        mkdir("/fatfs/profiles", 0775);
    }

    // Optional: Create a test file to verify filesystem works
    FILE *f = fopen("/fatfs/README.txt", "w");
    if (f != NULL) {
        fprintf(f, "SRO Filesystem\n");
        fprintf(f, "Upload web files to /web/\n");
        fprintf(f, "Upload profiles to /profiles/\n");
        fclose(f);
        ESP_LOGI(TAG, "Created /fatfs/README.txt");
    }

    return ESP_OK;
}

static esp_err_t init_peripherals(void)
{
    ESP_LOGI(TAG, "Initializing peripherals");
    // APA102 LED strip
    ESP_ERROR_CHECK(a2s_apa102_init(5, SRO_GPIO_APA102_MOSI, SRO_GPIO_APA102_CLK));
    
    
    // WS2812 board LED
    ESP_ERROR_CHECK(a2s_ws2812_init());

    a2s_apa102_set_effect(0, A2S_APA102_STATUS_BLINK_FAST, (a2s_apa102_color_t)A2S_APA102_COLOR_YELLOW, 0.20);

    // Set initial boot status LED
    // a2s_ws2812_set_boot_status(A2S_WS2812_STATUS_NORMAL_BOOT);
    a2s_ws2812_set_rgb(16, 16, 0, 255);

    ESP_LOGI(TAG, "Peripherals initialized");
    return ESP_OK;
}

static esp_err_t init_network_services(void)
{
    ESP_LOGI(TAG, "Initializing network services");

    // WiFi
    ESP_ERROR_CHECK(a2s_wifi_init());

    // Web Server
    ESP_ERROR_CHECK(SRO_WebServer_Init());
    ESP_ERROR_CHECK(SRO_WebServer_Start());
    xEventGroupSetBits(sro_status2_events, SRO_SE2_WEBSERVER_RUNNING);

    // WebSocket Server
    ESP_ERROR_CHECK(SRO_WebSocketServer_Init());
    ESP_ERROR_CHECK(SRO_WebSocketServer_Start());

    // FTP Server
    ESP_ERROR_CHECK(SRO_FtpServer_Init());
    ESP_ERROR_CHECK(SRO_FtpServer_Start());
    xEventGroupSetBits(sro_status2_events, SRO_SE2_FTP_RUNNING);

    ESP_LOGI(TAG, "Network services started");
    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Servo Event Callback - Called when servo movement completes or errors
// ----------------------------------------------------------------------------

static void servo_event_callback(a2s_servo_event_t event, uint8_t position, void *user_data)
{
    // Trigger WebSocket broadcast on any servo event
    xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_WEBSOCKET_BROADCAST);

    switch (event) {
        case A2S_SERVO_EVENT_MOVE_COMPLETE:
            // Movement completed successfully
            break;

        case A2S_SERVO_EVENT_ERROR:
            // Hardware error occurred - could trigger safety event
            // Optional: Set error flag, trigger emergency stop
            break;

        case A2S_SERVO_EVENT_TIMEOUT:
            // Movement timed out - mechanical issue?
            // Optional: Log error, trigger safety check
            break;

        case A2S_SERVO_EVENT_STOPPED:
            // Movement was stopped by user
            break;
    }
}

// ----------------------------------------------------------------------------
// Utility Functions
// ----------------------------------------------------------------------------
static void log_heap_status(const char *label)
{
    ESP_LOGI(TAG, "=== %s ===", label);
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap ever: %lu bytes", (unsigned long)esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "Largest free block: %lu bytes", (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    // PSRAM info (if available)
    ESP_LOGI(TAG, "Free PSRAM: %lu bytes", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "================");
}

static void print_task_stack_watermarks(void)
{
    UBaseType_t watermark;
    TaskHandle_t task_handle;

    const char *task_names[] = {
        "heater_pwm", "temp_ctrl", "profile",
        "coordinator", "websocket", "ws_timer", "data_mgr"};

    ESP_LOGI(TAG, "=== Task Stack Watermarks ===");

    for (int i = 0; i < 7; i++) {
        task_handle = xTaskGetHandle(task_names[i]);
        if (task_handle != NULL) {
            watermark = uxTaskGetStackHighWaterMark(task_handle);
            ESP_LOGI(TAG, "%-12s: %4lu bytes free (high watermark)",
                     task_names[i], (unsigned long)(watermark * sizeof(StackType_t)));
        }
    }

    ESP_LOGI(TAG, "============================");
}

// ----------------------------------------------------------------------------
// end of main.c
// ----------------------------------------------------------------------------
