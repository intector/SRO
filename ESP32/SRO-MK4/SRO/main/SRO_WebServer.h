/**
 ******************************************************************************
 * @file           : SRO_WebServer.h
 * @brief          : HTTP Web Server definitions
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
#ifndef __SRO_WEBSERVER_H__
#define __SRO_WEBSERVER_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include "main.h"

// ----------------------------------------------------------------------------
// constants and configuration
// ----------------------------------------------------------------------------
#define SRO_WEB_SERVER_PORT         80
#define SRO_WEB_SERVER_MAX_URI_LEN  512
#define SRO_WEB_ROOT_PATH           "/storage"
#define SRO_WEB_INDEX_FILE          "index.html"
#define SRO_WEB_MAX_RESP_SIZE       4096

// ----------------------------------------------------------------------------
// type definitions
// ----------------------------------------------------------------------------
typedef enum {
    SRO_WEB_MIME_HTML = 0,
    SRO_WEB_MIME_CSS,
    SRO_WEB_MIME_JS,
    SRO_WEB_MIME_JSON,
    SRO_WEB_MIME_PNG,
    SRO_WEB_MIME_ICO,
    SRO_WEB_MIME_TEXT,
    SRO_WEB_MIME_OCTET
} SRO_WebMimeType_t;

typedef struct {
    const char* extension;
    const char* mime_type;
} SRO_WebMimeMap_t;

// ----------------------------------------------------------------------------
// global function prototypes
// ----------------------------------------------------------------------------
esp_err_t SRO_WebServer_Init(void);
esp_err_t SRO_WebServer_Start(void);
esp_err_t SRO_WebServer_Stop(void);
esp_err_t SRO_WebServer_RegisterApiHandlers(void);
const char* SRO_WebServer_GetMimeType(const char* filename);
esp_err_t SRO_WebServer_SendFile(httpd_req_t* req, const char* filepath);
esp_err_t SRO_WebServer_SendJson(httpd_req_t* req, const char* json_data);
esp_err_t SRO_WebServer_SendError(httpd_req_t* req, int status_code, const char* message);
esp_err_t SRO_WebSocketServer_RegisterHandler(httpd_handle_t server);
bool sro_fatfs_file_exists(const char *filepath);

// ----------------------------------------------------------------------------
// end of SRO_WebServer.h
// ----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // __SRO_WEBSERVER_H__