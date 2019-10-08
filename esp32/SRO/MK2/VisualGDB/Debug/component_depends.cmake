# Component requirements generated by expand_requirements.cmake

set(BUILD_COMPONENTS soc;log;heap;xtensa-debug-module;app_trace;freertos;vfs;newlib;esp_ringbuf;driver;esp_event;ethernet;lwip;tcpip_adapter;partition_table;app_update;spi_flash;mbedtls;micro-ecc;bootloader_support;nvs_flash;pthread;smartconfig_ack;wpa_supplicant;esp32;cxx;asio;jsmn;aws_iot;bootloader;bt;coap;console;nghttp;esp-tls;esp_adc_cal;tcp_transport;esp_http_client;esp_http_server;esp_https_ota;esptool_py;expat;wear_levelling;sdmmc;fatfs;freemodbus;idf_test;json;libsodium;mdns;mqtt;openssl;protobuf-c;protocomm;spiffs;ulp;wifi_provisioning;main)
set(BUILD_COMPONENT_PATHS c:/SysGCC/esp32/esp-idf/v3.2/components/soc;c:/SysGCC/esp32/esp-idf/v3.2/components/log;c:/SysGCC/esp32/esp-idf/v3.2/components/heap;c:/SysGCC/esp32/esp-idf/v3.2/components/xtensa-debug-module;c:/SysGCC/esp32/esp-idf/v3.2/components/app_trace;c:/SysGCC/esp32/esp-idf/v3.2/components/freertos;c:/SysGCC/esp32/esp-idf/v3.2/components/vfs;c:/SysGCC/esp32/esp-idf/v3.2/components/newlib;c:/SysGCC/esp32/esp-idf/v3.2/components/esp_ringbuf;c:/SysGCC/esp32/esp-idf/v3.2/components/driver;c:/SysGCC/esp32/esp-idf/v3.2/components/esp_event;c:/SysGCC/esp32/esp-idf/v3.2/components/ethernet;c:/SysGCC/esp32/esp-idf/v3.2/components/lwip;c:/SysGCC/esp32/esp-idf/v3.2/components/tcpip_adapter;c:/SysGCC/esp32/esp-idf/v3.2/components/partition_table;c:/SysGCC/esp32/esp-idf/v3.2/components/app_update;c:/SysGCC/esp32/esp-idf/v3.2/components/spi_flash;c:/SysGCC/esp32/esp-idf/v3.2/components/mbedtls;c:/SysGCC/esp32/esp-idf/v3.2/components/micro-ecc;c:/SysGCC/esp32/esp-idf/v3.2/components/bootloader_support;c:/SysGCC/esp32/esp-idf/v3.2/components/nvs_flash;c:/SysGCC/esp32/esp-idf/v3.2/components/pthread;c:/SysGCC/esp32/esp-idf/v3.2/components/smartconfig_ack;c:/SysGCC/esp32/esp-idf/v3.2/components/wpa_supplicant;c:/SysGCC/esp32/esp-idf/v3.2/components/esp32;c:/SysGCC/esp32/esp-idf/v3.2/components/cxx;c:/SysGCC/esp32/esp-idf/v3.2/components/asio;c:/SysGCC/esp32/esp-idf/v3.2/components/jsmn;c:/SysGCC/esp32/esp-idf/v3.2/components/aws_iot;c:/SysGCC/esp32/esp-idf/v3.2/components/bootloader;c:/SysGCC/esp32/esp-idf/v3.2/components/bt;c:/SysGCC/esp32/esp-idf/v3.2/components/coap;c:/SysGCC/esp32/esp-idf/v3.2/components/console;c:/SysGCC/esp32/esp-idf/v3.2/components/nghttp;c:/SysGCC/esp32/esp-idf/v3.2/components/esp-tls;c:/SysGCC/esp32/esp-idf/v3.2/components/esp_adc_cal;c:/SysGCC/esp32/esp-idf/v3.2/components/tcp_transport;c:/SysGCC/esp32/esp-idf/v3.2/components/esp_http_client;c:/SysGCC/esp32/esp-idf/v3.2/components/esp_http_server;c:/SysGCC/esp32/esp-idf/v3.2/components/esp_https_ota;c:/SysGCC/esp32/esp-idf/v3.2/components/esptool_py;c:/SysGCC/esp32/esp-idf/v3.2/components/expat;c:/SysGCC/esp32/esp-idf/v3.2/components/wear_levelling;c:/SysGCC/esp32/esp-idf/v3.2/components/sdmmc;c:/SysGCC/esp32/esp-idf/v3.2/components/fatfs;c:/SysGCC/esp32/esp-idf/v3.2/components/freemodbus;c:/SysGCC/esp32/esp-idf/v3.2/components/idf_test;c:/SysGCC/esp32/esp-idf/v3.2/components/json;c:/SysGCC/esp32/esp-idf/v3.2/components/libsodium;c:/SysGCC/esp32/esp-idf/v3.2/components/mdns;c:/SysGCC/esp32/esp-idf/v3.2/components/mqtt;c:/SysGCC/esp32/esp-idf/v3.2/components/openssl;c:/SysGCC/esp32/esp-idf/v3.2/components/protobuf-c;c:/SysGCC/esp32/esp-idf/v3.2/components/protocomm;c:/SysGCC/esp32/esp-idf/v3.2/components/spiffs;c:/SysGCC/esp32/esp-idf/v3.2/components/ulp;c:/SysGCC/esp32/esp-idf/v3.2/components/wifi_provisioning;C:/Users/ms/source/repos/SRO/MK2/main)
set(BUILD_TEST_COMPONENTS )
set(BUILD_TEST_COMPONENT_PATHS )

# get_component_requirements: Generated function to read the dependencies of a given component.
#
# Parameters:
# - component: Name of component
# - var_requires: output variable name. Set to recursively expanded COMPONENT_REQUIRES 
#   for this component.
# - var_private_requires: output variable name. Set to recursively expanded COMPONENT_PRIV_REQUIRES 
#   for this component.
#
# Throws a fatal error if 'componeont' is not found (indicates a build system problem).
#
function(get_component_requirements component var_requires var_private_requires)
  if("${component}" STREQUAL "soc")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "log")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "heap")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "xtensa-debug-module")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "app_trace")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "xtensa-debug-module" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "freertos")
    set(${var_requires} "app_trace" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "vfs")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "newlib")
    set(${var_requires} "vfs" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esp_ringbuf")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "driver")
    set(${var_requires} "esp_ringbuf" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esp_event")
    set(${var_requires} "log" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "ethernet")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "tcpip_adapter;esp_event" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "lwip")
    set(${var_requires} "vfs" PARENT_SCOPE)
    set(${var_private_requires} "ethernet;tcpip_adapter" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "tcpip_adapter")
    set(${var_requires} "lwip" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "partition_table")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "app_update")
    set(${var_requires} "spi_flash;partition_table" PARENT_SCOPE)
    set(${var_private_requires} "bootloader_support" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "spi_flash")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "bootloader_support;app_update" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "mbedtls")
    set(${var_requires} "lwip" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "micro-ecc")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "bootloader_support")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "spi_flash;mbedtls;micro-ecc" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "nvs_flash")
    set(${var_requires} "spi_flash;mbedtls" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "pthread")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "smartconfig_ack")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "lwip;tcpip_adapter" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "wpa_supplicant")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "mbedtls" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esp32")
    set(${var_requires} "driver;tcpip_adapter;esp_event" PARENT_SCOPE)
    set(${var_private_requires} "app_trace;bootloader_support;ethernet;log;mbedtls;nvs_flash;pthread;smartconfig_ack;spi_flash;vfs;wpa_supplicant;xtensa-debug-module" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "cxx")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "asio")
    set(${var_requires} "lwip" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "jsmn")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "aws_iot")
    set(${var_requires} "mbedtls" PARENT_SCOPE)
    set(${var_private_requires} "jsmn" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "bootloader")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "bt")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "nvs_flash" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "coap")
    set(${var_requires} "lwip" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "console")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "nghttp")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esp-tls")
    set(${var_requires} "mbedtls" PARENT_SCOPE)
    set(${var_private_requires} "lwip;nghttp" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esp_adc_cal")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "tcp_transport")
    set(${var_requires} "lwip;esp-tls" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esp_http_client")
    set(${var_requires} "nghttp" PARENT_SCOPE)
    set(${var_private_requires} "mbedtls;lwip;esp-tls;tcp_transport" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esp_http_server")
    set(${var_requires} "nghttp" PARENT_SCOPE)
    set(${var_private_requires} "lwip" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esp_https_ota")
    set(${var_requires} "esp_http_client" PARENT_SCOPE)
    set(${var_private_requires} "log;app_update" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esptool_py")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "expat")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "wear_levelling")
    set(${var_requires} "spi_flash" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "sdmmc")
    set(${var_requires} "driver" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "fatfs")
    set(${var_requires} "wear_levelling;sdmmc" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "freemodbus")
    set(${var_requires} "driver" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "idf_test")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "json")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "libsodium")
    set(${var_requires} "mbedtls" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "mdns")
    set(${var_requires} "lwip;mbedtls;console;tcpip_adapter" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "mqtt")
    set(${var_requires} "lwip;nghttp;mbedtls;tcp_transport" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "openssl")
    set(${var_requires} "mbedtls" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "protobuf-c")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "protocomm")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "protobuf-c;mbedtls;console;esp_http_server;bt" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "spiffs")
    set(${var_requires} "spi_flash" PARENT_SCOPE)
    set(${var_private_requires} "bootloader_support" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "ulp")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "wifi_provisioning")
    set(${var_requires} "lwip" PARENT_SCOPE)
    set(${var_private_requires} "protobuf-c;protocomm" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "main")
    set(${var_requires} "app_trace;app_update;asio;aws_iot;bootloader;bootloader_support;bt;coap;console;cxx;driver;esp-tls;esp32;esp_adc_cal;esp_event;esp_http_client;esp_http_server;esp_https_ota;esp_ringbuf;esptool_py;ethernet;expat;fatfs;freemodbus;freertos;heap;idf_test;jsmn;json;libsodium;log;lwip;mbedtls;mdns;micro-ecc;mqtt;newlib;nghttp;nvs_flash;openssl;partition_table;protobuf-c;protocomm;pthread;sdmmc;smartconfig_ack;soc;spi_flash;spiffs;tcp_transport;tcpip_adapter;ulp;vfs;wear_levelling;wifi_provisioning;wpa_supplicant;xtensa-debug-module" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  message(FATAL_ERROR "Component not found: ${component}")
endfunction()
