@echo off
cd /D C:\Users\ms\source\repos\SRO_MK3\SRO_MK3\VisualGDB\Debug || (set FAIL_LINE=2& goto :ABORT)
python C:/SysGCC/esp32/esp-idf/v3.2/tools/kconfig_new/confgen.py --kconfig C:/SysGCC/esp32/esp-idf/v3.2/Kconfig --config C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/sdkconfig --defaults C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/sdkconfig.defaults --create-config-if-missing --env "COMPONENT_KCONFIGS= C:/SysGCC/esp32/esp-idf/v3.2/components/log/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/heap/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/app_trace/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/freertos/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/vfs/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/driver/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/esp_event/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/ethernet/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/lwip/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/tcpip_adapter/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/spi_flash/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/mbedtls/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/nvs_flash/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/pthread/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/esp32/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/aws_iot/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/bt/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/esp_adc_cal/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/esp_http_client/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/esp_http_server/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/wear_levelling/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/fatfs/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/freemodbus/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/libsodium/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/mdns/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/mqtt/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/openssl/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/spiffs/Kconfig" --env "COMPONENT_KCONFIGS_PROJBUILD= C:/SysGCC/esp32/esp-idf/v3.2/components/partition_table/Kconfig.projbuild C:/SysGCC/esp32/esp-idf/v3.2/components/bootloader/Kconfig.projbuild C:/SysGCC/esp32/esp-idf/v3.2/components/esptool_py/Kconfig.projbuild C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/main/Kconfig.projbuild" --env IDF_CMAKE=y --output config C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/sdkconfig || (set FAIL_LINE=3& goto :ABORT)
C:\Users\ms\AppData\Local\VisualGDB\CMake\bin\cmake.exe -E env "COMPONENT_KCONFIGS= C:/SysGCC/esp32/esp-idf/v3.2/components/log/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/heap/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/app_trace/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/freertos/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/vfs/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/driver/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/esp_event/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/ethernet/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/lwip/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/tcpip_adapter/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/spi_flash/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/mbedtls/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/nvs_flash/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/pthread/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/esp32/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/aws_iot/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/bt/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/esp_adc_cal/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/esp_http_client/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/esp_http_server/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/wear_levelling/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/fatfs/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/freemodbus/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/libsodium/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/mdns/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/mqtt/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/openssl/Kconfig C:/SysGCC/esp32/esp-idf/v3.2/components/spiffs/Kconfig" "COMPONENT_KCONFIGS_PROJBUILD= C:/SysGCC/esp32/esp-idf/v3.2/components/partition_table/Kconfig.projbuild C:/SysGCC/esp32/esp-idf/v3.2/components/bootloader/Kconfig.projbuild C:/SysGCC/esp32/esp-idf/v3.2/components/esptool_py/Kconfig.projbuild C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/main/Kconfig.projbuild" IDF_CMAKE=y KCONFIG_CONFIG=C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/sdkconfig kconfig_bin/mconf-idf C:/SysGCC/esp32/esp-idf/v3.2/Kconfig || (set FAIL_LINE=4& goto :ABORT)
goto :EOF

:ABORT
set ERROR_CODE=%ERRORLEVEL%
echo Batch file failed at line %FAIL_LINE% with errorcode %ERRORLEVEL%
exit /b %ERROR_CODE%