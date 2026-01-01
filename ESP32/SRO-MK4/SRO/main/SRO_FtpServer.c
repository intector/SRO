/**
 ******************************************************************************
 * @file           : SRO_WebServer.c
 * @brief          : SRO FTP Server Implementation
 *                   ESP32-S3 FTP Server for uploading web content to FAT-FS
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
#include "SRO_FtpServer.h"
#include "esp_timer.h"
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include "esp_netif.h"
#include "esp_wifi.h"

static const char *TAG = "SRO_FTP";

// ----------------------------------------------------------------------------
// Global FTP Server Context
// ----------------------------------------------------------------------------
sro_ftp_server_t g_ftp_server = {0};

// ----------------------------------------------------------------------------
// FTP Command Table
// ----------------------------------------------------------------------------
static const struct
{
    const char *name;
    ftp_cmd_t cmd;
} ftp_commands[] = {
    {"USER", FTP_CMD_USER},
    {"PASS", FTP_CMD_PASS},
    {"PWD", FTP_CMD_PWD},
    {"CWD", FTP_CMD_CWD},
    {"CDUP", FTP_CMD_CDUP},
    {"LIST", FTP_CMD_LIST},
    {"NLST", FTP_CMD_NLST},
    {"RETR", FTP_CMD_RETR},
    {"STOR", FTP_CMD_STOR},
    {"DELE", FTP_CMD_DELE},
    {"MKD", FTP_CMD_MKD},
    {"RMD", FTP_CMD_RMD},
    {"TYPE", FTP_CMD_TYPE},
    {"PASV", FTP_CMD_PASV},
    {"PORT", FTP_CMD_PORT},
    {"QUIT", FTP_CMD_QUIT},
    {"SYST", FTP_CMD_SYST},
    {"NOOP", FTP_CMD_NOOP},
    {"HELP", FTP_CMD_HELP},
    {"SIZE", FTP_CMD_SIZE},
    {"MDTM", FTP_CMD_MDTM},
    {"FEAT", FTP_CMD_FEAT}};

// ----------------------------------------------------------------------------
// Server Management Functions
// ----------------------------------------------------------------------------
esp_err_t SRO_FtpServer_Init(void)
{

    memset(&g_ftp_server, 0, sizeof(g_ftp_server));
    g_ftp_server.listen_socket  = -1;
    g_ftp_server.status         = SRO_FTP_STOPPED;
    g_ftp_server.next_data_port = SRO_FTP_DATA_PORT_START;

    // Create mutex for client management
    g_ftp_server.clients_mutex = xSemaphoreCreateMutex();
    if (!g_ftp_server.clients_mutex) {
        ESP_LOGE(TAG, "Failed to create clients mutex");
        return ESP_ERR_NO_MEM;
    }

    // FIXED: Initialize client structures with proper socket initialization
    for (int i = 0; i < SRO_FTP_MAX_CLIENTS; i++) {
        g_ftp_server.clients[i].control_socket         = -1;
        g_ftp_server.clients[i].data_conn.socket       = -1;
        g_ftp_server.clients[i].data_conn.listen_socket = -1;  // FIXED: Initialize listening socket
        g_ftp_server.clients[i].data_conn.port         = 0;
        g_ftp_server.clients[i].data_conn.active       = false;
        g_ftp_server.clients[i].client_id              = i;
        g_ftp_server.clients[i].task_handle            = NULL;
    }

    return ESP_OK;
}

esp_err_t SRO_FtpServer_Start(void)
{
    if (g_ftp_server.status == SRO_FTP_RUNNING) {
        ESP_LOGW(TAG, "FTP server already running");
        return ESP_OK;
    }


    g_ftp_server.status = SRO_FTP_STARTING;

    // Create listening socket
    g_ftp_server.listen_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (g_ftp_server.listen_socket < 0) {
        ESP_LOGE(TAG, "Failed to create listening socket");
        g_ftp_server.status = SRO_FTP_ERROR;
        return ESP_FAIL;
    }

    // Set socket options
    int opt = 1;
    setsockopt(g_ftp_server.listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind socket
    struct sockaddr_in server_addr = {0};
    server_addr.sin_family      = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port        = htons(SRO_FTP_PORT);

    if (bind(g_ftp_server.listen_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket");
        close(g_ftp_server.listen_socket);
        g_ftp_server.listen_socket = -1;
        g_ftp_server.status = SRO_FTP_ERROR;
        return ESP_FAIL;
    }

    // Start listening
    if (listen(g_ftp_server.listen_socket, SRO_FTP_MAX_CLIENTS) < 0) {
        ESP_LOGE(TAG, "Failed to listen on socket");
        close(g_ftp_server.listen_socket);
        g_ftp_server.listen_socket = -1;
        g_ftp_server.status = SRO_FTP_ERROR;
        return ESP_FAIL;
    }

    // Create server task
    g_ftp_server.running = true;
    if (xTaskCreate(sro_ftp_server_task, "ftp_server", SRO_FTP_TASK_STACK_SIZE,
                    NULL, SRO_FTP_TASK_PRIORITY, &g_ftp_server.server_task) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create server task");
        close(g_ftp_server.listen_socket);
        g_ftp_server.listen_socket = -1;
        g_ftp_server.running = false;
        g_ftp_server.status = SRO_FTP_ERROR;
        return ESP_FAIL;
    }

    g_ftp_server.status = SRO_FTP_RUNNING;
    return ESP_OK;
}

esp_err_t SRO_FtpServer_Stop(void)
{

    g_ftp_server.running = false;
    g_ftp_server.status  = SRO_FTP_STOPPED;

    // Close listening socket
    if (g_ftp_server.listen_socket >= 0) {
        close(g_ftp_server.listen_socket);
        g_ftp_server.listen_socket = -1;
    }

    // Cleanup all clients
    xSemaphoreTake(g_ftp_server.clients_mutex, portMAX_DELAY);
    for (int i = 0; i < SRO_FTP_MAX_CLIENTS; i++) {
        if (g_ftp_server.clients[i].control_socket >= 0) {
            sro_ftp_cleanup_client(&g_ftp_server.clients[i]);
        }
    }
    xSemaphoreGive(g_ftp_server.clients_mutex);

    // Wait for server task to finish
    if (g_ftp_server.server_task) {
        vTaskDelete(g_ftp_server.server_task);
        g_ftp_server.server_task = NULL;
    }

    return ESP_OK;
}

esp_err_t sro_ftp_deinit(void)
{
    // Stop server if running
    if (g_ftp_server.status == SRO_FTP_RUNNING) {
        SRO_FtpServer_Stop();
    }

    // Clean up mutex
    if (g_ftp_server.clients_mutex) {
        vSemaphoreDelete(g_ftp_server.clients_mutex);
        g_ftp_server.clients_mutex = NULL;
    }

    memset(&g_ftp_server, 0, sizeof(g_ftp_server));
    return ESP_OK;
}

sro_ftp_status_t sro_ftp_get_status(void)
{
    return g_ftp_server.status;
}

// ----------------------------------------------------------------------------
// Server Tasks
// ----------------------------------------------------------------------------
void sro_ftp_server_task(void *pvParameters)
{

    while (g_ftp_server.running) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        // Accept new connection
        int client_socket = accept(g_ftp_server.listen_socket,
                                   (struct sockaddr *)&client_addr, &client_len);

        if (client_socket < 0) {
            if (g_ftp_server.running) {
                ESP_LOGE(TAG, "Failed to accept client connection");
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            continue;
        }

        // Get client info
        char client_ip[16];
        inet_ntoa_r(client_addr.sin_addr, client_ip, sizeof(client_ip));


        // Find free client slot
        ftp_client_t *client = sro_ftp_get_free_client();
        if (!client) {
            ESP_LOGW(TAG, "No free client slots available");
            send(client_socket, "421 Too many connections\r\n", 26, 0);
            close(client_socket);
            continue;
        }

        // Initialize client
        sro_ftp_init_client(client, client_socket, client->client_id);
        strcpy(client->client_ip, client_ip);

        // Send welcome message
        char welcome[128];
        snprintf(welcome, sizeof(welcome), "220 %s\r\n", SRO_FTP_WELCOME_MSG);
        send(client_socket, welcome, strlen(welcome), 0);

        // Create client task with reduced stack size
        if (xTaskCreate(sro_ftp_client_task, "ftp_client", SRO_FTP_TASK_STACK_SIZE,
                        client, SRO_FTP_TASK_PRIORITY, &client->task_handle) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to create client task");
            sro_ftp_cleanup_client(client);
            continue;
        }

    }

    vTaskDelete(NULL);
}

void sro_ftp_client_task(void *pvParameters)
{
    ftp_client_t *client = (ftp_client_t *)pvParameters;
    char command_buffer[256];
    int bytes_received;


    while (g_ftp_server.running && client->control_socket >= 0) {
        // Check for timeout
        if (sro_ftp_client_timeout_check(client)) {
            ESP_LOGW(TAG, "[Client %d] Client timeout", client->client_id);
            break;
        }

        // Receive command
        bytes_received = recv(client->control_socket, command_buffer, 
                             sizeof(command_buffer) - 1, MSG_DONTWAIT);

        if (bytes_received > 0) {
            command_buffer[bytes_received] = '\0';

            // Remove trailing \r\n
            char *end = command_buffer + bytes_received - 1;
            while (end >= command_buffer && (*end == '\r' || *end == '\n')) {
                *end-- = '\0';
            }

            if (strlen(command_buffer) > 0) {
                client->last_activity = esp_timer_get_time() / 1000000;

                // Process command
                sro_ftp_process_command(client, command_buffer);
            }
        }
        else if (bytes_received == 0) {
            break;
        }
        else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGE(TAG, "[Client %d] Receive error: %d", client->client_id, errno);
            break;
        }

        // Small delay to feed watchdog and prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    sro_ftp_cleanup_client(client);
    vTaskDelete(NULL);
}

// ----------------------------------------------------------------------------
// Client Management
// ----------------------------------------------------------------------------
ftp_client_t *sro_ftp_get_free_client(void)
{
    xSemaphoreTake(g_ftp_server.clients_mutex, portMAX_DELAY);

    for (int i = 0; i < SRO_FTP_MAX_CLIENTS; i++) {
        if (g_ftp_server.clients[i].control_socket < 0) {
            xSemaphoreGive(g_ftp_server.clients_mutex);
            return &g_ftp_server.clients[i];
        }
    }

    xSemaphoreGive(g_ftp_server.clients_mutex);
    return NULL;
}

void sro_ftp_init_client(ftp_client_t *client, int socket, int client_id)
{
    memset(client, 0, sizeof(ftp_client_t));
    client->control_socket         = socket;
    client->data_conn.socket       = -1;  // Connected data socket
    client->data_conn.listen_socket = -1;  // FIXED: Initialize listening socket
    client->data_conn.port         = 0;
    client->data_conn.active       = false;
    client->state                  = FTP_STATE_IDLE;
    client->mode                   = FTP_MODE_PASSIVE;
    client->type                   = FTP_TYPE_BINARY;
    client->client_id              = client_id;
    client->last_activity          = esp_timer_get_time() / 1000000;
    strcpy(client->current_dir, "/");

    // Always suspend (idempotent if already suspended)
    if (suspend_spi_tasks() != ESP_OK) {
        ESP_LOGW(TAG, "Failed to suspend SPI tasks");
    }
}

void sro_ftp_cleanup_client(ftp_client_t *client)
{
    // CRITICAL FIX: Save client_id BEFORE memset clears it
    int saved_client_id = client->client_id;

    // Close control socket
    if (client->control_socket >= 0) {
        close(client->control_socket);
        client->control_socket = -1;
    }

    // FIXED: Enhanced data connection cleanup
    sro_ftp_close_data_connection(client);

    // Clear task handle (task will delete itself)
    if (client->task_handle) {
        client->task_handle = NULL;
    }

    // Clear all client data - AFTER logging
    memset(client, 0, sizeof(ftp_client_t));
    client->control_socket          = -1;
    client->data_conn.socket        = -1;
    client->data_conn.listen_socket = -1;

    // Check if this was the LAST client (after cleanup so count is 0)
    if (SRO_FtpServer_GetActiveClientCount() == 0) {
        ESP_LOGI(TAG, "Last FTP client disconnected - resuming SPI tasks");
        resume_spi_tasks();
    }
}

bool sro_ftp_client_timeout_check(ftp_client_t *client)
{
    uint32_t current_time = esp_timer_get_time() / 1000000;
    return (current_time - client->last_activity) > SRO_FTP_TIMEOUT_SEC;
}

// ----------------------------------------------------------------------------
// Data Connection Management - Enhanced socket resource management
// ----------------------------------------------------------------------------
esp_err_t sro_ftp_close_data_connection(ftp_client_t *client)
{
    bool any_closed = false;
    int client_id = client->client_id; // Save before any operations
    
    // FIXED: Close connected data socket (from accept())
    if (client->data_conn.socket >= 0) {
        close(client->data_conn.socket);
        client->data_conn.socket = -1;
        any_closed = true;
    }
    
    // FIXED: Close listening socket (from PASV setup) - This was the main leak!
    if (client->data_conn.listen_socket >= 0) {
        close(client->data_conn.listen_socket);
        client->data_conn.listen_socket = -1;
        any_closed = true;
    }
    
    // Reset data connection state
    client->data_conn.active = false;
    client->data_conn.port = 0;
    memset(&client->data_conn.addr, 0, sizeof(client->data_conn.addr));
    
    if (any_closed) {
    }
    
    return ESP_OK;
}

esp_err_t sro_ftp_setup_passive_mode(ftp_client_t *client)
{
    
    // FIXED: Close any existing data connections first (prevents accumulation)
    sro_ftp_close_data_connection(client);

    // Create listening socket
    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "[Client %d] Failed to create listening socket", client->client_id);
        return ESP_FAIL;
    }

    // Set socket options for better resource management
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // FIXED: Set socket timeout to prevent indefinite blocking
    struct timeval timeout;
    timeout.tv_sec = 30;  // 30 second timeout
    timeout.tv_usec = 0;
    setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // Bind to any available port
    struct sockaddr_in data_addr = {0};
    data_addr.sin_family         = AF_INET;
    data_addr.sin_addr.s_addr    = INADDR_ANY;
    data_addr.sin_port           = 0; // Let system choose port

    if (bind(listen_sock, (struct sockaddr *)&data_addr, sizeof(data_addr)) < 0) {
        ESP_LOGE(TAG, "[Client %d] Failed to bind listening socket", client->client_id);
        close(listen_sock);
        return ESP_FAIL;
    }

    // Get assigned port
    socklen_t addr_len = sizeof(data_addr);
    if (getsockname(listen_sock, (struct sockaddr *)&data_addr, &addr_len) < 0) {
        ESP_LOGE(TAG, "[Client %d] Failed to get data port", client->client_id);
        close(listen_sock);
        return ESP_FAIL;
    }

    client->data_conn.port = ntohs(data_addr.sin_port);

    // Listen for connection
    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "[Client %d] Failed to listen on data socket", client->client_id);
        close(listen_sock);
        return ESP_FAIL;
    }

    // FIXED: Store listening socket separately
    client->data_conn.listen_socket = listen_sock;
    client->data_conn.socket = -1;  // No connected socket yet
    client->mode = FTP_MODE_PASSIVE;
    client->data_conn.active = false;

    
    return ESP_OK;
}

esp_err_t sro_ftp_open_data_connection(ftp_client_t *client)
{
    if (client->mode == FTP_MODE_PASSIVE) {
        
        // FIXED: Check if we have a valid listening socket
        if (client->data_conn.listen_socket < 0) {
            ESP_LOGE(TAG, "[Client %d] No listening socket available", client->client_id);
            return ESP_FAIL;
        }
                 
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        
        // Use select() for timeout instead of blocking accept
        fd_set read_fds;
        struct timeval timeout;
        
        FD_ZERO(&read_fds);
        FD_SET(client->data_conn.listen_socket, &read_fds);
        
        timeout.tv_sec = 30;   // 30 second timeout
        timeout.tv_usec = 0;
        
        int select_result = select(client->data_conn.listen_socket + 1, &read_fds, NULL, NULL, &timeout);
        
        if (select_result <= 0) {
            ESP_LOGE(TAG, "[Client %d] Data connection timeout or error (select=%d)", 
                     client->client_id, select_result);
            return ESP_FAIL;
        }
        
        if (!FD_ISSET(client->data_conn.listen_socket, &read_fds)) {
            ESP_LOGE(TAG, "[Client %d] No data connection available", client->client_id);
            return ESP_FAIL;
        }

        // Accept the connection
        int data_socket = accept(client->data_conn.listen_socket, 
                                (struct sockaddr *)&client_addr, &addr_len);
        
        if (data_socket < 0) {
            ESP_LOGE(TAG, "[Client %d] Failed to accept data connection (errno: %d)", 
                     client->client_id, errno);
            return ESP_FAIL;
        }

        char client_ip[16];
        inet_ntoa_r(client_addr.sin_addr, client_ip, sizeof(client_ip));
        

        // FIXED: Close listening socket after successful accept (prevents accumulation)
        close(client->data_conn.listen_socket);
        client->data_conn.listen_socket = -1;
        
        // Store connected socket
        client->data_conn.socket = data_socket;
        client->data_conn.active = true;

        // Set data socket options for better performance
        int opt = 1;
        setsockopt(client->data_conn.socket, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));
        setsockopt(client->data_conn.socket, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

        return ESP_OK;
    }

    ESP_LOGE(TAG, "[Client %d] Active mode not implemented", client->client_id);
    return ESP_FAIL;
}

// ----------------------------------------------------------------------------
// Command Processing
// ----------------------------------------------------------------------------
ftp_cmd_t sro_ftp_parse_command(const char *cmd_line, char *arg, size_t arg_size)
{
    char cmd[8] = {0};

    // Extract command and argument
    if (sscanf(cmd_line, "%7s %255s", cmd, arg ? arg : cmd) < 1) {
        return FTP_CMD_UNKNOWN;
    }

    // Convert command to uppercase
    for (int i = 0; cmd[i]; i++) {
        cmd[i] = toupper((unsigned char)cmd[i]);
    }

    // Find command in table
    for (size_t i = 0; i < sizeof(ftp_commands) / sizeof(ftp_commands[0]); i++) {
        if (strcmp(cmd, ftp_commands[i].name) == 0) {
            return ftp_commands[i].cmd;
        }
    }

    return FTP_CMD_UNKNOWN;
}

esp_err_t sro_ftp_process_command(ftp_client_t *client, const char *cmd_line)
{
    char arg[256] = {0};
    ftp_cmd_t cmd = sro_ftp_parse_command(cmd_line, arg, sizeof(arg));

    switch (cmd) {
        case FTP_CMD_USER: return sro_ftp_cmd_user(client, arg);
        case FTP_CMD_PASS: return sro_ftp_cmd_pass(client, arg);
        case FTP_CMD_PWD:  return sro_ftp_cmd_pwd(client);
        case FTP_CMD_CWD:  return sro_ftp_cmd_cwd(client, arg);
        case FTP_CMD_CDUP: return sro_ftp_cmd_cdup(client);
        case FTP_CMD_LIST: return sro_ftp_cmd_list(client, arg);
        case FTP_CMD_NLST: return sro_ftp_cmd_nlst(client, arg);
        case FTP_CMD_RETR: return sro_ftp_cmd_retr(client, arg);
        case FTP_CMD_STOR: return sro_ftp_cmd_stor(client, arg);
        case FTP_CMD_DELE: return sro_ftp_cmd_dele(client, arg);
        case FTP_CMD_MKD:  return sro_ftp_cmd_mkd(client, arg);
        case FTP_CMD_RMD:  return sro_ftp_cmd_rmd(client, arg);
        case FTP_CMD_TYPE: return sro_ftp_cmd_type(client, arg);
        case FTP_CMD_PASV: return sro_ftp_cmd_pasv(client);
        case FTP_CMD_PORT: return sro_ftp_cmd_port(client, arg);
        case FTP_CMD_QUIT: return sro_ftp_cmd_quit(client);
        case FTP_CMD_SYST: return sro_ftp_cmd_syst(client);
        case FTP_CMD_NOOP: return sro_ftp_cmd_noop(client);
        case FTP_CMD_HELP: return sro_ftp_cmd_help(client);
        case FTP_CMD_SIZE: return sro_ftp_cmd_size(client, arg);
        case FTP_CMD_MDTM: return sro_ftp_cmd_mdtm(client, arg);
        case FTP_CMD_FEAT: return sro_ftp_cmd_feat(client);
        default:
            return sro_ftp_send_response(client, 500, "Command not recognized");
    }
}

// ----------------------------------------------------------------------------
// FTP Command Handlers
// ----------------------------------------------------------------------------
esp_err_t sro_ftp_cmd_user(ftp_client_t *client, const char *username)
{
    if (strlen(username) == 0) {
        return sro_ftp_send_response(client, 501, "Username required");
    }

    strcpy(client->username, username);
    client->state = FTP_STATE_USER;

    if (strcmp(username, SRO_FTP_USER) == 0) {
        return sro_ftp_send_response(client, FTP_RESP_USER_OK, "Password required");
    }
    else {
        return sro_ftp_send_response(client, 530, "Invalid username");
    }
}

esp_err_t sro_ftp_cmd_pass(ftp_client_t *client, const char *password)
{
    if (client->state != FTP_STATE_USER) {
        return sro_ftp_send_response(client, FTP_RESP_BAD_SEQUENCE, "Send USER first");
    }

    if (strcmp(password, SRO_FTP_PASS) == 0) {
        client->authenticated = true;
        client->state = FTP_STATE_LOGGED_IN;
        return sro_ftp_send_response(client, FTP_RESP_LOGIN_OK, "Login successful");
    }
    else {
        client->state = FTP_STATE_IDLE;
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Login incorrect");
    }
}

esp_err_t sro_ftp_cmd_pwd(ftp_client_t *client)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    char response[300];
    snprintf(response, sizeof(response), "\"%s\" is current directory", client->current_dir);
    
    // Use correct 257 response code for PWD
    return sro_ftp_send_response(client, 257, response);
}

esp_err_t sro_ftp_cmd_cwd(ftp_client_t *client, const char *path)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    char *abs_path = sro_ftp_get_absolute_path(client, path);
    if (!abs_path) {
        return sro_ftp_send_response(client, 550, "Invalid path");
    }

    char full_path[512];
    snprintf(full_path, sizeof(full_path), "%s%s", SRO_FTP_ROOT_DIR, abs_path);

    struct stat st;
    if (stat(full_path, &st) == 0 && S_ISDIR(st.st_mode)) {
        strcpy(client->current_dir, abs_path);
        free(abs_path);
        return sro_ftp_send_response(client, FTP_RESP_FILE_ACTION_OK, "Directory changed");
    }
    else {
        free(abs_path);
        return sro_ftp_send_response(client, FTP_RESP_FILE_NOT_FOUND, "Directory not found");
    }
}

esp_err_t sro_ftp_cmd_cdup(ftp_client_t *client)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }
    
    // CDUP is equivalent to CWD ..
    return sro_ftp_cmd_cwd(client, "..");
}

esp_err_t sro_ftp_cmd_list(ftp_client_t *client, const char *path)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    const char *list_path = (path && strlen(path) > 0) ? path : "current";


    // FIXED: Check if we have a valid data connection setup
    if (client->data_conn.listen_socket < 0) {
        ESP_LOGE(TAG, "[Client %d] No data connection available, PASV command required first", 
                 client->client_id);
        return sro_ftp_send_response(client, 425, "Use PASV first");
    }

    // Send start response
    esp_err_t start_result = sro_ftp_send_response(client, FTP_RESP_START_DATA, 
                                                   "Here comes the directory listing");
    if (start_result != ESP_OK) {
        sro_ftp_close_data_connection(client);  // FIXED: Cleanup on error
        return start_result;
    }
    
    // Small delay to ensure client receives response
    vTaskDelay(pdMS_TO_TICKS(100));

    // Open data connection with timeout
    esp_err_t data_result = sro_ftp_open_data_connection(client);
    if (data_result != ESP_OK) {
        ESP_LOGE(TAG, "[Client %d] Failed to open data connection", client->client_id);
        sro_ftp_close_data_connection(client);  // FIXED: Cleanup on error
        return sro_ftp_send_response(client, 425, "Can't open data connection");
    }

    // Send directory listing
    esp_err_t result = sro_ftp_send_directory_listing(client, path);

    // FIXED: Always close data connection after transfer
    sro_ftp_close_data_connection(client);

    if (result == ESP_OK) {
        return sro_ftp_send_response(client, FTP_RESP_CLOSE_DATA, "Directory send OK");
    }
    else {
        return sro_ftp_send_response(client, 450, "Transfer failed");
    }
}

esp_err_t sro_ftp_cmd_nlst(ftp_client_t *client, const char *path) 
{
    if (!client) return ESP_ERR_INVALID_ARG;
    
    return sro_ftp_send_response(client, 502, "NLST not implemented");
}

esp_err_t sro_ftp_cmd_retr(ftp_client_t *client, const char *filename)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    char *abs_path = sro_ftp_get_absolute_path(client, filename);
    if (!abs_path) {
        return ESP_FAIL;
    }

    char full_path[768];
    int path_len = snprintf(full_path, sizeof(full_path), "%s%s", SRO_FTP_ROOT_DIR, abs_path);
    
    if (path_len >= sizeof(full_path)) {
        ESP_LOGE(TAG, "File path too long: %s", abs_path);
        free(abs_path);
        return ESP_FAIL;
    }

    // Check if file exists
    struct stat file_stat;
    if (stat(full_path, &file_stat) != 0) {
        ESP_LOGW(TAG, "[Client %d] File not found: %s", client->client_id, abs_path);
        free(abs_path);
        return sro_ftp_send_response(client, FTP_RESP_FILE_NOT_FOUND, "File not found");
    }

    // Check if it's a regular file
    if (!S_ISREG(file_stat.st_mode)) {
        ESP_LOGW(TAG, "[Client %d] Not a regular file: %s", client->client_id, abs_path);
        free(abs_path);
        return sro_ftp_send_response(client, 550, "Not a regular file");
    }

    // FIXED: Check if we have a valid data connection setup
    if (client->data_conn.listen_socket < 0) {
        ESP_LOGE(TAG, "[Client %d] No data connection available, PASV command required first", 
                 client->client_id);
        free(abs_path);
        return sro_ftp_send_response(client, 425, "Use PASV first");
    }

    // FIXED: Send data transfer start response
    esp_err_t start_result = sro_ftp_send_response(client, FTP_RESP_START_DATA, "Opening data connection for file transfer");
    if (start_result != ESP_OK) {
        sro_ftp_close_data_connection(client);  // FIXED: Cleanup on error
        free(abs_path);
        return start_result;
    }

    // FIXED: Small delay to ensure client receives response
    vTaskDelay(pdMS_TO_TICKS(100));

    // FIXED: Open data connection AFTER sending 150 response
    esp_err_t data_result = sro_ftp_open_data_connection(client);
    if (data_result != ESP_OK) {
        ESP_LOGE(TAG, "[Client %d] Failed to open data connection", client->client_id);
        sro_ftp_close_data_connection(client);  // FIXED: Cleanup on error
        free(abs_path);
        return sro_ftp_send_response(client, 425, "Can't open data connection");
    }

    // Open file for reading
    FILE *file = fopen(full_path, "rb");
    if (!file) {
        ESP_LOGE(TAG, "[Client %d] Failed to open file: %s", client->client_id, abs_path);
        sro_ftp_close_data_connection(client);
        free(abs_path);
        return sro_ftp_send_response(client, 550, "Failed to open file");
    }

    // Send file contents
    // char buffer[SRO_FTP_BUFFER_SIZE];

    // Allocate buffer from SPIRAM
    char *buffer = heap_caps_malloc(SRO_FTP_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate FTP send buffer");
        fclose(file);
        sro_ftp_close_data_connection(client);
        free(abs_path);
        return sro_ftp_send_response(client, 550, "Memory allocation failed");
    }

    size_t total_bytes = 0;
    
    
    while (!feof(file)) {
        size_t bytes_read = fread(buffer, 1, sizeof(buffer), file);
        if (bytes_read > 0) {
            int sent = send(client->data_conn.socket, buffer, bytes_read, 0);
            if (sent <= 0) {
                ESP_LOGE(TAG, "[Client %d] Failed to send data", client->client_id);
                break;
            }
            total_bytes += sent;
        }
        // yield every 1KB
        if (total_bytes % 1024 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    free(buffer);
    fclose(file);
    
    // FIXED: Always close data connection after transfer
    sro_ftp_close_data_connection(client);
    
    
    free(abs_path);
    
    // FIXED: Send completion response
    return sro_ftp_send_response(client, FTP_RESP_CLOSE_DATA, "Transfer complete");
}

esp_err_t sro_ftp_cmd_stor(ftp_client_t *client, const char *filename)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    if (!filename || strlen(filename) == 0) {
        return sro_ftp_send_response(client, 501, "Filename required");
    }


    // FIXED: Check if we have a valid data connection setup
    if (client->data_conn.listen_socket < 0) {
        ESP_LOGE(TAG, "[Client %d] No data connection available, PASV command required first", 
                 client->client_id);
        return sro_ftp_send_response(client, 425, "Use PASV first");
    }

    // Send start response
    esp_err_t start_result = sro_ftp_send_response(client, FTP_RESP_START_DATA, 
                                                   "Ready for file data");
    if (start_result != ESP_OK) {
        sro_ftp_close_data_connection(client);  // FIXED: Cleanup on error
        return start_result;
    }

    // Small delay to ensure client receives response
    vTaskDelay(pdMS_TO_TICKS(100));

    // Open data connection
    esp_err_t data_result = sro_ftp_open_data_connection(client);
    if (data_result != ESP_OK) {
        ESP_LOGE(TAG, "[Client %d] Failed to open data connection", client->client_id);
        sro_ftp_close_data_connection(client);  // FIXED: Cleanup on error
        return sro_ftp_send_response(client, 425, "Can't open data connection");
    }

    // Receive file
    esp_err_t result = sro_ftp_receive_file(client, filename);

    // FIXED: Always close data connection after transfer
    sro_ftp_close_data_connection(client);

    if (result == ESP_OK) {
        return sro_ftp_send_response(client, FTP_RESP_CLOSE_DATA, "Transfer complete");
    }
    else {
        return sro_ftp_send_response(client, 450, "Transfer failed");
    }
}

esp_err_t sro_ftp_cmd_dele(ftp_client_t *client, const char *filename)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    char *abs_path = sro_ftp_get_absolute_path(client, filename);
    if (!abs_path) {
        return sro_ftp_send_response(client, 550, "Invalid filename");
    }

    char full_path[512];
    snprintf(full_path, sizeof(full_path), "%s%s", SRO_FTP_ROOT_DIR, abs_path);

    if (unlink(full_path) == 0) {
        SRO_FTP_LOG_CLIENT(client, "Deleted file: %s", abs_path);
        free(abs_path);
        return sro_ftp_send_response(client, FTP_RESP_FILE_ACTION_OK, "File deleted");
    }
    else {
        free(abs_path);
        return sro_ftp_send_response(client, FTP_RESP_FILE_NOT_FOUND, "Delete failed");
    }
}

esp_err_t sro_ftp_cmd_mkd(ftp_client_t *client, const char *dirname)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    char *abs_path = sro_ftp_get_absolute_path(client, dirname);
    if (!abs_path) {
        return sro_ftp_send_response(client, 550, "Invalid directory name");
    }

    char full_path[512];
    snprintf(full_path, sizeof(full_path), "%s%s", SRO_FTP_ROOT_DIR, abs_path);

    if (mkdir(full_path, 0755) == 0) {
        SRO_FTP_LOG_CLIENT(client, "Created directory: %s", abs_path);
        char response[300];
        snprintf(response, sizeof(response), "\"%s\" directory created", abs_path);
        free(abs_path);
        return sro_ftp_send_response(client, FTP_RESP_PATH_CREATED, response);
    }
    else {
        free(abs_path);
        return sro_ftp_send_response(client, 550, "Create directory failed");
    }
}

esp_err_t sro_ftp_cmd_rmd(ftp_client_t *client, const char *dirname) 
{
    if (!client || !dirname) return ESP_ERR_INVALID_ARG;
    
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    char *abs_path = sro_ftp_get_absolute_path(client, dirname);
    if (!abs_path) {
        return sro_ftp_send_response(client, 550, "Invalid directory name");
    }

    char full_path[512];
    snprintf(full_path, sizeof(full_path), "%s%s", SRO_FTP_ROOT_DIR, abs_path);

    // Check if directory exists and is actually a directory
    struct stat st;
    if (stat(full_path, &st) != 0) {
        free(abs_path);
        return sro_ftp_send_response(client, 550, "Directory not found");
    }
    
    if (!S_ISDIR(st.st_mode)) {
        free(abs_path);
        return sro_ftp_send_response(client, 550, "Not a directory");
    }

    // Check if directory is empty
    DIR *dir = opendir(full_path);
    if (!dir) {
        free(abs_path);
        return sro_ftp_send_response(client, 550, "Cannot access directory");
    }
    
    struct dirent *entry;
    bool is_empty = true;
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
            is_empty = false;
            break;
        }
    }
    closedir(dir);
    
    if (!is_empty) {
        free(abs_path);
        return sro_ftp_send_response(client, 550, "Directory not empty");
    }

    // Remove the directory
    if (rmdir(full_path) == 0) {
        SRO_FTP_LOG_CLIENT(client, "Removed directory: %s", abs_path);
        free(abs_path);
        return sro_ftp_send_response(client, FTP_RESP_FILE_ACTION_OK, "Directory removed");
    } else {
        free(abs_path);
        return sro_ftp_send_response(client, 550, "Remove directory operation failed");
    }
}

esp_err_t sro_ftp_cmd_type(ftp_client_t *client, const char *type)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    if (toupper((unsigned char)type[0]) == 'I') {
        client->type = FTP_TYPE_BINARY;
        return sro_ftp_send_response(client, 200, "Type set to I (Binary)");
    }
    else if (toupper((unsigned char)type[0]) == 'A') {
        client->type = FTP_TYPE_ASCII;
        return sro_ftp_send_response(client, 200, "Type set to A (ASCII)");
    }
    else {
        return sro_ftp_send_response(client, 504, "Command not implemented for that parameter");
    }
}

esp_err_t sro_ftp_cmd_pasv(ftp_client_t *client)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    
    // Setup passive mode
    esp_err_t result = sro_ftp_setup_passive_mode(client);
    if (result != ESP_OK) {
        return result;
    }

    // Get local IP address
    char local_ip[16];
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        snprintf(local_ip, sizeof(local_ip), IPSTR, IP2STR(&ip_info.ip));
    } else {
        // Fallback - try to get from control socket
        struct sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);
        if (getsockname(client->control_socket, (struct sockaddr*)&addr, &addr_len) == 0) {
            inet_ntoa_r(addr.sin_addr, local_ip, sizeof(local_ip));
        } else {
            strcpy(local_ip, "192.168.1.65"); // Final fallback
        }
    }
    
    
    // Convert IP to FTP format (replace . with ,)
    char ftp_ip[16];
    strcpy(ftp_ip, local_ip);
    for (int i = 0; ftp_ip[i]; i++) {
        if (ftp_ip[i] == '.') ftp_ip[i] = ',';
    }
    
    // Calculate port bytes
    uint16_t port = client->data_conn.port;
    uint8_t p1 = (port >> 8) & 0xFF;
    uint8_t p2 = port & 0xFF;
    
    char response[128];
    snprintf(response, sizeof(response), "Entering Passive Mode (%s,%d,%d)", ftp_ip, p1, p2);
    
    
    return sro_ftp_send_response(client, FTP_RESP_PASV_MODE, response);
}

esp_err_t sro_ftp_cmd_port(ftp_client_t *client, const char *params) 
{
    if (!client || !params) return ESP_ERR_INVALID_ARG;
    
    return sro_ftp_send_response(client, 502, "PORT command not implemented");
}

esp_err_t sro_ftp_cmd_quit(ftp_client_t *client)
{
    return sro_ftp_send_response(client, FTP_RESP_BYE, "Goodbye");
}

esp_err_t sro_ftp_cmd_syst(ftp_client_t *client)
{
    return sro_ftp_send_response(client, FTP_RESP_SYSTEM_TYPE, "UNIX Type: L8");
}

esp_err_t sro_ftp_cmd_noop(ftp_client_t *client)
{
    return sro_ftp_send_response(client, 200, "OK");
}

esp_err_t sro_ftp_cmd_help(ftp_client_t *client)
{
    return sro_ftp_send_response(client, FTP_RESP_HELP_MSG, "Help not available");
}

esp_err_t sro_ftp_cmd_size(ftp_client_t *client, const char *filename)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    char *abs_path = sro_ftp_get_absolute_path(client, filename);
    if (!abs_path) {
        return sro_ftp_send_response(client, 550, "Invalid filename");
    }

    char full_path[512];
    snprintf(full_path, sizeof(full_path), "%s%s", SRO_FTP_ROOT_DIR, abs_path);

    struct stat st;
    if (stat(full_path, &st) == 0 && S_ISREG(st.st_mode)) {
        char response[64];
        snprintf(response, sizeof(response), "%ld", st.st_size);
        free(abs_path);
        return sro_ftp_send_response(client, FTP_RESP_FILE_STATUS_OK, response);
    }
    else {
        free(abs_path);
        return sro_ftp_send_response(client, FTP_RESP_FILE_NOT_FOUND, "File not found");
    }
}

esp_err_t sro_ftp_cmd_mdtm(ftp_client_t *client, const char *filename)
{
    if (!client->authenticated) {
        return sro_ftp_send_response(client, FTP_RESP_NOT_LOGGED_IN, "Not logged in");
    }

    return sro_ftp_send_response(client, 502, "MDTM not implemented");
}

esp_err_t sro_ftp_cmd_feat(ftp_client_t *client)
{
    // Send properly formatted multi-line FEAT response
    const char *feat_response = 
        "211-Features:\r\n"
        " SIZE\r\n"
        " PASV\r\n"
        "211 End\r\n";
    
    int sent = send(client->control_socket, feat_response, strlen(feat_response), 0);
    if (sent != strlen(feat_response)) {
        ESP_LOGE(TAG, "[Client %d] Failed to send FEAT response", client->client_id);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// ----------------------------------------------------------------------------
// File Operations
// ----------------------------------------------------------------------------
esp_err_t sro_ftp_receive_file(ftp_client_t *client, const char *filename)
{
    char *abs_path = sro_ftp_get_absolute_path(client, filename);
    if (!abs_path) {
        return ESP_FAIL;
    }

    char full_path[768];  // Increased buffer size
    int path_len = snprintf(full_path, sizeof(full_path), "%s%s", SRO_FTP_ROOT_DIR, abs_path);
    
    // Check if path was truncated
    if (path_len >= sizeof(full_path)) {
        ESP_LOGE(TAG, "File path too long: %s", abs_path);
        free(abs_path);
        return ESP_FAIL;
    }

    // Create parent directories if needed
    esp_err_t dir_result = sro_ftp_create_parent_directories(full_path);
    if (dir_result != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create parent directories for: %s", full_path);
        // Continue anyway - might still work
    }

    SRO_FTP_LOG_CLIENT(client, "Receiving file: %s", abs_path);

    FILE *file = fopen(full_path, "wb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to create file: %s", full_path);
        free(abs_path);
        return ESP_FAIL;
    }

    // char buffer[SRO_FTP_BUFFER_SIZE];
    //  Allocate buffer from SPIRAM instead of stack
    char *buffer = heap_caps_malloc(SRO_FTP_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate FTP buffer");
        fclose(file);
        free(abs_path);
        return ESP_FAIL;
    }

    size_t total_bytes = 0;
    int bytes_received;
    uint32_t last_yield = 0;

    // Set shorter timeout for data reception
    struct timeval timeout;
    timeout.tv_sec  = 30;  // 30 second timeout for data
    timeout.tv_usec = 0;
    setsockopt(client->data_conn.socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    while ((bytes_received = recv(client->data_conn.socket, buffer, sizeof(buffer), 0)) > 0) {
        size_t bytes_written = fwrite(buffer, 1, bytes_received, file);
        if (bytes_written != bytes_received) {
            ESP_LOGE(TAG, "File write error");
            break;
        }
        total_bytes += bytes_written;

        // Feed watchdog every 1KB to prevent timeout during large transfers
        if (total_bytes - last_yield > 1024) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Feed watchdog
            last_yield = total_bytes;
        }
    }

    fflush(file);  // Force write to storage
    fsync(fileno(file));  // Sync to filesystem
    fclose(file);

    // Give filesystem time to update
    vTaskDelay(pdMS_TO_TICKS(100));

    SRO_FTP_LOG_CLIENT(client, "File received: %s (%u bytes)", abs_path, (unsigned int)total_bytes);
    free(abs_path);
    free(buffer);

    return (bytes_received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) ? ESP_FAIL : ESP_OK;
}

/*
esp_err_t sro_ftp_send_directory_listing(ftp_client_t *client, const char *path)
{
    const char *list_path = (path && strlen(path) > 0) ? path : "current";

    char *abs_path = (strcmp(list_path, "current") == 0) ? 
                     sro_ftp_get_absolute_path(client, "") : 
                     sro_ftp_get_absolute_path(client, list_path);
    
    if (!abs_path) {
        return ESP_FAIL;
    }

    char full_path[512];
    snprintf(full_path, sizeof(full_path), "%s%s", SRO_FTP_ROOT_DIR, abs_path);

    DIR *dir = opendir(full_path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", full_path);
        free(abs_path);
        return ESP_FAIL;
    }

    struct dirent *entry;
    char listing_line[512];
    
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        // Truncate very long filenames to prevent overflow
        char safe_filename[64];
        strncpy(safe_filename, entry->d_name, sizeof(safe_filename) - 1);
        safe_filename[sizeof(safe_filename) - 1] = '\0';

        char entry_path[1024];
        snprintf(entry_path, sizeof(entry_path), "%s/%s", full_path, safe_filename);

        struct stat st;
        if (stat(entry_path, &st) == 0) {
            // Format directory listing line (Unix style)
            char permissions[11] = "----------";
            if (S_ISDIR(st.st_mode)) {
                permissions[0] = 'd';
                permissions[1] = permissions[4] = permissions[7] = 'r';
                permissions[2] = permissions[5] = permissions[8] = 'x';
            } else {
                permissions[1] = permissions[4] = permissions[7] = 'r';
                permissions[2] = permissions[5] = 'w';
            }

            snprintf(listing_line, sizeof(listing_line),
                     "%s   1 owner group %8ld Jan 01 12:00 %s\r\n",
                    permissions, st.st_size, safe_filename);
            
            if (send(client->data_conn.socket, listing_line, strlen(listing_line), 0) < 0) {
                ESP_LOGE(TAG, "Failed to send listing line");
                break;
            }
        }
    }

    closedir(dir);
    free(abs_path);
    return ESP_OK;
}
*/
esp_err_t sro_ftp_send_directory_listing(ftp_client_t *client, const char *path)
{
    const char *list_path = (path && strlen(path) > 0) ? path : "current";

    char *abs_path        = (strcmp(list_path, "current") == 0) ? sro_ftp_get_absolute_path(client, "") : sro_ftp_get_absolute_path(client, list_path);

    if (!abs_path) {
        return ESP_FAIL;
    }

    // Allocate buffers from SPIRAM instead of stack
    char *full_path    = heap_caps_malloc(512, MALLOC_CAP_SPIRAM);
    char *listing_line = heap_caps_malloc(512, MALLOC_CAP_SPIRAM);
    char *entry_path   = heap_caps_malloc(1024, MALLOC_CAP_SPIRAM);

    if (!full_path || !listing_line || !entry_path) {
        ESP_LOGE(TAG, "Failed to allocate directory listing buffers");
        free(abs_path);
        if (full_path)
            free(full_path);
        if (listing_line)
            free(listing_line);
        if (entry_path)
            free(entry_path);
        return ESP_FAIL;
    }

    snprintf(full_path, 512, "%s%s", SRO_FTP_ROOT_DIR, abs_path);

    DIR *dir = opendir(full_path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", full_path);
        free(abs_path);
        free(full_path);
        free(listing_line);
        free(entry_path);
        return ESP_FAIL;
    }

    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        // Truncate very long filenames to prevent overflow
        char safe_filename[64];
        strncpy(safe_filename, entry->d_name, sizeof(safe_filename) - 1);
        safe_filename[sizeof(safe_filename) - 1] = '\0';

        snprintf(entry_path, 1024, "%s/%s", full_path, safe_filename);

        struct stat st;
        if (stat(entry_path, &st) == 0) {
            // Format directory listing line (Unix style)
            char permissions[11] = "----------";
            if (S_ISDIR(st.st_mode)) {
                permissions[0] = 'd';
                permissions[1] = permissions[4] = permissions[7] = 'r';
                permissions[2] = permissions[5] = permissions[8] = 'x';
            }
            else {
                permissions[1] = permissions[4] = permissions[7] = 'r';
                permissions[2] = permissions[5] = 'w';
            }

            snprintf(listing_line, 512,
                     "%s   1 owner group %8ld Jan 01 12:00 %s\r\n",
                     permissions, st.st_size, safe_filename);

            if (send(client->data_conn.socket, listing_line, strlen(listing_line), 0) < 0) {
                ESP_LOGE(TAG, "Failed to send listing line");
                break;
            }
        }
    }

    closedir(dir);
    free(abs_path);
    free(full_path);
    free(listing_line);
    free(entry_path);

    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Utility Functions
// ----------------------------------------------------------------------------
esp_err_t sro_ftp_send_response(ftp_client_t *client, int code, const char *message)
{
    char response[512];
    int len = snprintf(response, sizeof(response), "%d %s\r\n", code, message);
    

    int sent = send(client->control_socket, response, len, 0);
    int send_errno = errno;
    

    return (sent == len) ? ESP_OK : ESP_FAIL;
}

char *sro_ftp_get_absolute_path(ftp_client_t *client, const char *relative_path)
{
    if (!relative_path || strlen(relative_path) == 0) {
        char *result = malloc(strlen(client->current_dir) + 1);
        if (result) {
            strcpy(result, client->current_dir);
        }
        return result;
    }

    char *abs_path = malloc(512);
    if (!abs_path) {
        return NULL;
    }

    if (relative_path[0] == '/') {
        // Absolute path
        strcpy(abs_path, relative_path);
    }
    else {
        // Relative path
        if (strcmp(client->current_dir, "/") == 0) {
            snprintf(abs_path, 512, "/%s", relative_path);
        }
        else {
            snprintf(abs_path, 512, "%s/%s", client->current_dir, relative_path);
        }
    }

    // Normalize path (remove .., ., etc.)
    sro_ftp_normalize_path(abs_path);

    return abs_path;
}

esp_err_t sro_ftp_normalize_path(char *path)
{
    if (!path) return ESP_ERR_INVALID_ARG;
    
    // Convert to absolute path components
    char *components[32];  // Max 32 path components
    int component_count = 0;
    
    // Split path into components
    char *path_copy = malloc(strlen(path) + 1);
    strcpy(path_copy, path);
    
    char *token = strtok(path_copy, "/");
    while (token && component_count < 32) {
        if (strcmp(token, ".") == 0) {
            // Skip current directory
        }
        else if (strcmp(token, "..") == 0) {
            // Go to parent directory
            if (component_count > 0) {
                component_count--;
            }
        }
        else {
            components[component_count++] = token;
        }
        token = strtok(NULL, "/");
    }
    
    // Rebuild path
    strcpy(path, "/");
    for (int i = 0; i < component_count; i++) {
        if (strlen(path) > 1) {
            strcat(path, "/");
        }
        strcat(path, components[i]);
    }
    
    free(path_copy);
    return ESP_OK;
}

bool sro_ftp_path_is_valid(const char *path)
{
    if (!path || strlen(path) == 0) {
        return false;
    }

    // Check for invalid characters
    if (strstr(path, "//")) {
        return false;
    }

    return true;
}

uint16_t sro_ftp_get_next_data_port(void)
{
    uint16_t port = g_ftp_server.next_data_port++;
    if (g_ftp_server.next_data_port > SRO_FTP_DATA_PORT_END) {
        g_ftp_server.next_data_port = SRO_FTP_DATA_PORT_START;
    }
    return port;
}

esp_err_t sro_ftp_create_parent_directories(const char *file_path)
{
    if (!file_path) return ESP_ERR_INVALID_ARG;
    
    // Make a copy of the path since we'll modify it
    char path_copy[512];
    strncpy(path_copy, file_path, sizeof(path_copy) - 1);
    path_copy[sizeof(path_copy) - 1] = '\0';
    
    // Find the last slash to get parent directory
    char *last_slash = strrchr(path_copy, '/');
    if (!last_slash || last_slash == path_copy) {
        // No parent directory or root directory
        return ESP_OK;
    }
    
    // Null-terminate at the last slash to get parent directory
    *last_slash = '\0';
    
    // Check if parent directory exists
    struct stat st;
    if (stat(path_copy, &st) == 0) {
        // Directory already exists
        return ESP_OK;
    }
    
    // Use iterative creation instead of recursion to avoid stack issues
    // Split the path and create directories one by one
    char build_path[512] = {0};
    char temp_path[512];
    strcpy(temp_path, path_copy);
    
    // Start from root
    strcpy(build_path, "/");
    
    // Use safe tokenization
    char *saveptr;
    char *token = strtok_r(temp_path, "/", &saveptr);
    
    while (token != NULL) {
        // Skip empty tokens
        if (strlen(token) == 0) {
            token = strtok_r(NULL, "/", &saveptr);
            continue;
        }
        
        // Build next level path
        if (strlen(build_path) > 1) {
            strncat(build_path, "/", sizeof(build_path) - strlen(build_path) - 1);
        }
        strncat(build_path, token, sizeof(build_path) - strlen(build_path) - 1);
        
        // Check if this directory exists
        if (stat(build_path, &st) != 0) {
            // Directory doesn't exist, create it
            if (mkdir(build_path, 0755) != 0) {
                ESP_LOGE(TAG, "Failed to create directory: %s (errno: %d)", build_path, errno);
                return ESP_FAIL;
            }
            
            // Small delay to ensure filesystem updates
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        token = strtok_r(NULL, "/", &saveptr);
    }
    
    return ESP_OK;
}

esp_err_t suspend_spi_tasks(void)
{
    ESP_LOGI(TAG, "Suspending SPI tasks for FTP operation");

    xEventGroupSetBits(sro_ctrl2_events,
                       SRO_CE2_SUSPEND_APA102 | SRO_CE2_SUSPEND_TEMP);

    EventBits_t bits = xEventGroupWaitBits(sro_status2_events,
                                           SRO_SE2_APA102_SUSPENDED | SRO_SE2_TEMP_SUSPENDED,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(1000));

    if ((bits & (SRO_SE2_APA102_SUSPENDED | SRO_SE2_TEMP_SUSPENDED)) !=
        (SRO_SE2_APA102_SUSPENDED | SRO_SE2_TEMP_SUSPENDED)) {
        ESP_LOGE(TAG, "Timeout waiting for SPI tasks to suspend");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

void resume_spi_tasks(void)
{
    ESP_LOGI(TAG, "Resuming SPI tasks");
    xEventGroupClearBits(sro_ctrl2_events,
                         SRO_CE2_SUSPEND_APA102 | SRO_CE2_SUSPEND_TEMP);
}

// ----------------------------------------------------------------------------
// Resource Monitoring Functions
// ----------------------------------------------------------------------------
uint16_t SRO_FtpServer_GetActiveClientCount(void)
{
    uint16_t count = 0;
    
    if (!g_ftp_server.clients_mutex) {
        return 0;  // Server not initialized
    }
    
    // Take mutex to safely access clients array
    if (xSemaphoreTake(g_ftp_server.clients_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        
        for (int i = 0; i < SRO_FTP_MAX_CLIENTS; i++) {
            // Check if client has active control socket
            if (g_ftp_server.clients[i].control_socket >= 0) {
                count++;
            }
        }
        
        xSemaphoreGive(g_ftp_server.clients_mutex);
    } else {
        ESP_LOGW("SRO_FTP", "Failed to take mutex for client count");
    }
    
    return count;
}
esp_err_t SRO_FtpServer_DisconnectAllClients(void)
{
    ESP_LOGW("SRO_FTP", "Emergency: Disconnecting all FTP clients");
    
    if (!g_ftp_server.clients_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(g_ftp_server.clients_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        
        for (int i = 0; i < SRO_FTP_MAX_CLIENTS; i++) {
            if (g_ftp_server.clients[i].control_socket >= 0) {
                ESP_LOGW("SRO_FTP", "Force disconnecting client %d", i);
                sro_ftp_cleanup_client(&g_ftp_server.clients[i]);
            }
        }
        
        xSemaphoreGive(g_ftp_server.clients_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}
uint16_t SRO_FtpServer_GetActiveSocketCount(void)
{
    uint16_t count = 0;
    
    if (!g_ftp_server.clients_mutex) {
        return 0;
    }
    
    if (xSemaphoreTake(g_ftp_server.clients_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        
        for (int i = 0; i < SRO_FTP_MAX_CLIENTS; i++) {
            if (g_ftp_server.clients[i].control_socket >= 0) {
                count++; // Control socket
                
                if (g_ftp_server.clients[i].data_conn.socket >= 0) {
                    count++; // Connected data socket
                }
                
                // CRITICAL: This was missing in many implementations
                if (g_ftp_server.clients[i].data_conn.listen_socket >= 0) {
                    count++; // Listening data socket - THE MAIN LEAK SOURCE
                }
            }
        }
        
        xSemaphoreGive(g_ftp_server.clients_mutex);
    }
    
    return count;
}
esp_err_t SRO_FtpServer_ForceCleanupDataConnections(void)
{
    ESP_LOGW("SRO_FTP", "Force cleanup: Cleaning all data connections");
    
    if (!g_ftp_server.clients_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(g_ftp_server.clients_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        
        for (int i = 0; i < SRO_FTP_MAX_CLIENTS; i++) {
            if (g_ftp_server.clients[i].control_socket >= 0) {
                ESP_LOGW("SRO_FTP", "Force cleanup: Client %d data connections", i);
                sro_ftp_close_data_connection(&g_ftp_server.clients[i]);
            }
        }
        
        xSemaphoreGive(g_ftp_server.clients_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

// ----------------------------------------------------------------------------
// end of SRO_FTPServer.c
// ----------------------------------------------------------------------------
