# Install script for directory: C:/sysgcc/esp32/esp-idf/v3.2/components/bootloader/subproject

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/bootloader")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/soc/cmake_install.cmake")
  include("C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/bootloader/cmake_install.cmake")
  include("C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/spi_flash/cmake_install.cmake")
  include("C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/micro-ecc/cmake_install.cmake")
  include("C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/bootloader_support/cmake_install.cmake")
  include("C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/esptool_py/cmake_install.cmake")
  include("C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/partition_table/cmake_install.cmake")
  include("C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/esp32/cmake_install.cmake")
  include("C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/log/cmake_install.cmake")
  include("C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/main/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "C:/Users/ms/source/repos/SRO_MK3/SRO_MK3/VisualGDB/Debug/bootloader/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")