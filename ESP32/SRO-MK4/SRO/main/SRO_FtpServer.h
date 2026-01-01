/**
 ******************************************************************************
 * @file           : SRO_FTPServer.h
 * @brief          : SRO FTP Server
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

// Define to prevent recursive inclusion --------------------------------------

#ifndef __SRO_FTPSERVER_H__
#define __SRO_FTPSERVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include <string.h>
#include <sys/stat.h>
#include "main.h"

// ----------------------------------------------------------------------------
// FTP Server Configuration
// ----------------------------------------------------------------------------
#define SRO_FTP_PORT            21
#define SRO_FTP_DATA_PORT_START 20000
#define SRO_FTP_DATA_PORT_END   20099
#define SRO_FTP_MAX_CLIENTS     2
#define SRO_FTP_TIMEOUT_SEC     300
#define SRO_FTP_BUFFER_SIZE     1024 // reduced from 2048 to 1024
#define SRO_FTP_TASK_STACK_SIZE (16 * 1024)
#define SRO_FTP_TASK_PRIORITY   2

// ----------------------------------------------------------------------------
// FTP Authentication
// ----------------------------------------------------------------------------
#define SRO_FTP_USER        "sro"
#define SRO_FTP_PASS        "upload"
#define SRO_FTP_WELCOME_MSG "ESP32-S3 SRO FTP Server Ready"
#define SRO_FTP_ROOT_DIR    "/fatfs"

// ----------------------------------------------------------------------------
// FTP Response Codes
// ----------------------------------------------------------------------------
#define FTP_RESP_READY           220
#define FTP_RESP_BYE             221
#define FTP_RESP_DATA_OPEN       225
#define FTP_RESP_CLOSE_DATA      226
#define FTP_RESP_PASV_MODE       227
#define FTP_RESP_LOGIN_OK        230
#define FTP_RESP_FILE_ACTION_OK  250
#define FTP_RESP_PATH_CREATED    257
#define FTP_RESP_USER_OK         331
#define FTP_RESP_NEED_LOGIN      332
#define FTP_RESP_START_DATA      150
#define FTP_RESP_FILE_STATUS_OK  213
#define FTP_RESP_SYSTEM_TYPE     215
#define FTP_RESP_HELP_MSG        214
#define FTP_RESP_NOT_IMPLEMENTED 502
#define FTP_RESP_BAD_SEQUENCE    503
#define FTP_RESP_NOT_LOGGED_IN   530
#define FTP_RESP_FILE_NOT_FOUND  550
#define FTP_RESP_FILE_EXISTS     553

// ----------------------------------------------------------------------------
// type definitions
// ----------------------------------------------------------------------------
typedef enum
{
    FTP_CMD_USER = 0,
    FTP_CMD_PASS,
    FTP_CMD_PWD,
    FTP_CMD_CWD,
    FTP_CMD_CDUP,
    FTP_CMD_LIST,
    FTP_CMD_NLST,
    FTP_CMD_RETR,
    FTP_CMD_STOR,
    FTP_CMD_DELE,
    FTP_CMD_MKD,
    FTP_CMD_RMD,
    FTP_CMD_TYPE,
    FTP_CMD_PASV,
    FTP_CMD_PORT,
    FTP_CMD_QUIT,
    FTP_CMD_SYST,
    FTP_CMD_NOOP,
    FTP_CMD_HELP,
    FTP_CMD_SIZE,
    FTP_CMD_MDTM,
    FTP_CMD_FEAT,
    FTP_CMD_UNKNOWN
} ftp_cmd_t;

typedef enum
{
    FTP_STATE_IDLE = 0,
    FTP_STATE_USER,
    FTP_STATE_PASS,
    FTP_STATE_LOGGED_IN,
    FTP_STATE_DATA_CONNECT,
    FTP_STATE_DATA_TRANSFER
} ftp_state_t;

typedef enum
{
    FTP_MODE_ACTIVE = 0,
    FTP_MODE_PASSIVE
} ftp_mode_t;

typedef enum
{
    FTP_TYPE_ASCII = 0,
    FTP_TYPE_BINARY
} ftp_type_t;

typedef struct
{
    int socket;         // Connected data socket (from accept())
    int listen_socket;  // Listening socket (from PASV setup)K
    int port;
    bool active;
    struct sockaddr_in addr;
} ftp_data_conn_t;

typedef struct
{
    int control_socket;
    ftp_data_conn_t data_conn;
    ftp_state_t state;
    ftp_mode_t mode;
    ftp_type_t type;
    char username[32];
    char current_dir[256];
    char client_ip[16];
    uint32_t last_activity;
    bool authenticated;
    TaskHandle_t task_handle;
    int client_id;
} ftp_client_t;

typedef enum
{
    SRO_FTP_STOPPED  = 0,
    SRO_FTP_STARTING = 1,
    SRO_FTP_RUNNING  = 2,
    SRO_FTP_ERROR    = 3
} sro_ftp_status_t;

typedef struct
{
    int listen_socket;
    TaskHandle_t server_task;
    ftp_client_t clients[SRO_FTP_MAX_CLIENTS];
    bool running;
    sro_ftp_status_t status;
    SemaphoreHandle_t clients_mutex;
    uint16_t next_data_port;
} sro_ftp_server_t;

// ----------------------------------------------------------------------------
// globalfunction prototypes
// ----------------------------------------------------------------------------
esp_err_t SRO_FtpServer_Init(void);
esp_err_t SRO_FtpServer_Start(void);
esp_err_t SRO_FtpServer_Stop(void);
esp_err_t sro_ftp_deinit(void);
sro_ftp_status_t sro_ftp_get_status(void);

void sro_ftp_server_task(void *pvParameters);
void sro_ftp_client_task(void *pvParameters);

ftp_client_t *sro_ftp_get_free_client(void);
void sro_ftp_init_client(ftp_client_t *client, int socket, int client_id);
void sro_ftp_cleanup_client(ftp_client_t *client);
bool sro_ftp_client_timeout_check(ftp_client_t *client);

ftp_cmd_t sro_ftp_parse_command(const char *cmd_line, char *arg, size_t arg_size);
esp_err_t sro_ftp_process_command(ftp_client_t *client, const char *cmd_line);

esp_err_t sro_ftp_cmd_user(ftp_client_t *client, const char *username);
esp_err_t sro_ftp_cmd_pass(ftp_client_t *client, const char *password);
esp_err_t sro_ftp_cmd_pwd(ftp_client_t *client);
esp_err_t sro_ftp_cmd_cwd(ftp_client_t *client, const char *path);
esp_err_t sro_ftp_cmd_cdup(ftp_client_t *client);
esp_err_t sro_ftp_cmd_list(ftp_client_t *client, const char *path);
esp_err_t sro_ftp_cmd_nlst(ftp_client_t *client, const char *path);
esp_err_t sro_ftp_cmd_retr(ftp_client_t *client, const char *filename);
esp_err_t sro_ftp_cmd_stor(ftp_client_t *client, const char *filename);
esp_err_t sro_ftp_cmd_dele(ftp_client_t *client, const char *filename);
esp_err_t sro_ftp_cmd_mkd(ftp_client_t *client, const char *dirname);
esp_err_t sro_ftp_cmd_rmd(ftp_client_t *client, const char *dirname);
esp_err_t sro_ftp_cmd_type(ftp_client_t *client, const char *type);
esp_err_t sro_ftp_cmd_pasv(ftp_client_t *client);
esp_err_t sro_ftp_cmd_port(ftp_client_t *client, const char *port_info);
esp_err_t sro_ftp_cmd_quit(ftp_client_t *client);
esp_err_t sro_ftp_cmd_syst(ftp_client_t *client);
esp_err_t sro_ftp_cmd_noop(ftp_client_t *client);
esp_err_t sro_ftp_cmd_help(ftp_client_t *client);
esp_err_t sro_ftp_cmd_size(ftp_client_t *client, const char *filename);
esp_err_t sro_ftp_cmd_mdtm(ftp_client_t *client, const char *filename);
esp_err_t sro_ftp_cmd_feat(ftp_client_t *client);

esp_err_t sro_ftp_open_data_connection(ftp_client_t *client);
esp_err_t sro_ftp_close_data_connection(ftp_client_t *client);
esp_err_t sro_ftp_setup_passive_mode(ftp_client_t *client);
esp_err_t sro_ftp_setup_active_mode(ftp_client_t *client, const char *port_info);

esp_err_t sro_ftp_send_file(ftp_client_t *client, const char *filename);
esp_err_t sro_ftp_receive_file(ftp_client_t *client, const char *filename);
esp_err_t sro_ftp_send_directory_listing(ftp_client_t *client, const char *path);

esp_err_t sro_ftp_send_response(ftp_client_t *client, int code, const char *message);
esp_err_t sro_ftp_send_data(ftp_client_t *client, const char *data, size_t length);
esp_err_t sro_ftp_receive_data(ftp_client_t *client, char *buffer, size_t *length);
char *sro_ftp_get_absolute_path(ftp_client_t *client, const char *relative_path);
bool sro_ftp_path_is_valid(const char *path);
esp_err_t sro_ftp_normalize_path(char *path);
uint16_t sro_ftp_get_next_data_port(void);
esp_err_t sro_ftp_create_parent_directories(const char *file_path);
esp_err_t suspend_spi_tasks(void);
void resume_spi_tasks(void);

uint16_t SRO_FtpServer_GetActiveClientCount(void);
esp_err_t SRO_FtpServer_DisconnectAllClients(void);
uint16_t SRO_FtpServer_GetActiveSocketCount(void);
esp_err_t SRO_FtpServer_ForceCleanupDataConnections(void);

// ----------------------------------------------------------------------------
// Helper Macros
// ----------------------------------------------------------------------------
#define SRO_FTP_CHECK(x)                                                                            \
    do {                                                                                            \
        esp_err_t __err = (x);                                                                      \
        if (__err != ESP_OK) {                                                                      \
            ESP_LOGE("SRO_FTP", "Error at %s:%d - %s", __FILE__, __LINE__, esp_err_to_name(__err)); \
            return __err;                                                                           \
        }                                                                                           \
    } while (0)

#define SRO_FTP_LOG_CLIENT(client, fmt, ...) \

#define SRO_FTP_LOG_ERROR(client, fmt, ...) \
    ESP_LOGE("SRO_FTP", "[Client %d] " fmt, client->client_id, ##__VA_ARGS__)

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
extern sro_ftp_server_t g_ftp_server;

// ----------------------------------------------------------------------------
// end of SRO_FTPServer.h
// ----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // __SRO_FTPSERVER_H__
