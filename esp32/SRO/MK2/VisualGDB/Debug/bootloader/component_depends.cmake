# Component requirements generated by expand_requirements.cmake

set(BUILD_COMPONENTS soc;bootloader;spi_flash;micro-ecc;bootloader_support;esptool_py;partition_table;esp32;log;main)
set(BUILD_COMPONENT_PATHS c:/SysGCC/esp32/esp-idf/v3.2/components/soc;c:/SysGCC/esp32/esp-idf/v3.2/components/bootloader;c:/SysGCC/esp32/esp-idf/v3.2/components/spi_flash;c:/SysGCC/esp32/esp-idf/v3.2/components/micro-ecc;c:/SysGCC/esp32/esp-idf/v3.2/components/bootloader_support;c:/SysGCC/esp32/esp-idf/v3.2/components/esptool_py;c:/SysGCC/esp32/esp-idf/v3.2/components/partition_table;c:/SysGCC/esp32/esp-idf/v3.2/components/esp32;c:/SysGCC/esp32/esp-idf/v3.2/components/log;C:/sysgcc/esp32/esp-idf/v3.2/components/bootloader/subproject/main)
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
  if("${component}" STREQUAL "bootloader")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "spi_flash")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "bootloader_support" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "micro-ecc")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "bootloader_support")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "spi_flash;micro-ecc" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esptool_py")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "partition_table")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "esp32")
    set(${var_requires} "main;bootloader;bootloader_support;esp32;esptool_py;log;micro-ecc;partition_table;soc;spi_flash" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "log")
    set(${var_requires} "" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  if("${component}" STREQUAL "main")
    set(${var_requires} "bootloader;bootloader_support" PARENT_SCOPE)
    set(${var_private_requires} "" PARENT_SCOPE)
    return()
  endif()
  message(FATAL_ERROR "Component not found: ${component}")
endfunction()
