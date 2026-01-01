# ESP32-S3 SRO Project CMakeLists.txt
# Updated to include all project components

idf_component_register(
    SRCS
    	"main.c"
    	"a2s_wifi.c"
    	"a2s_max6675.c"
    	"a2s_apa102.c"
    	"a2s_servo.c"
    	"a2s_ssr.c"
    	"a2s_ws2812.c"
    	a2s_pid.c
    	SRO_FtpServer.c
    	SRO_WebServer.c
    	SRO_WebSocketServer.c
    	SRO_ProfileManager.c
    	SRO_SystemCoordinator.c
    	SRO_TemperatureControl.c
    	SRO_WebSocketBroadcast.c
    	wifi_credentials.h.example
    INCLUDE_DIRS "."
    REQUIRES
    	driver
    	esp_wifi
    	esp_event
    	esp_netif
    	esp_system
    	esp_http_server
    	fatfs
    	wear_levelling
    	json
    	esp_timer
    	freertos
    	spi_flash
    	nvs_flash
    	lwip
    	vfs
    	esp_https_ota
    	app_update)