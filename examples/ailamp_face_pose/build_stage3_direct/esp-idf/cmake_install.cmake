# Install script for directory: D:/Espressif/frameworks/esp-idf-v5.4.2

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/human_face_recognition")
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

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "D:/Espressif/tools/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32s3-elf-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/xtensa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_gpio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_pm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/bootloader/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esptool_py/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/partition_table/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_app_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_bootloader_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/app_update/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_partition/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/efuse/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/bootloader_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_mm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/spi_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_system/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_common/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_rom/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/hal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/log/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/heap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_security/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_hw_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/freertos/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/newlib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/pthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/cxx/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_timer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_gptimer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_ringbuf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_uart/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/app_trace/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_event/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/nvs_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_pcnt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_spi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_mcpwm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_ana_cmpr/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_i2s/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/sdmmc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_sdmmc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_sdspi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_sdio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_dac/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_rmt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_tsens/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_sdm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_i2c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_ledc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_parlio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_usb_serial_jtag/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/driver/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_phy/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_vfs_console/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/vfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/lwip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_netif_stack/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_netif/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/wpa_supplicant/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_coex/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_wifi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_gdbstub/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/bt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/unity/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/cmock/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/console/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/http_parser/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp-tls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_adc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_isp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_cam/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_psram/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_jpeg/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_ppa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_driver_touch_sens/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_eth/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_hid/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/tcp_transport/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_http_client/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_http_server/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_https_ota/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_https_server/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_lcd/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/protobuf-c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/protocomm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/esp_local_ctrl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espcoredump/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/wear_levelling/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/idf_test/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/ieee802154/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/json/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/mqtt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/nvs_sec_provider/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/perfmon/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/rt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/spiffs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/touch_element/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/ulp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/usb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/wifi_provisioning/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/who_spiflash_fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/who_task/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__dl_fft/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__esp_new_jpeg/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__esp-dsp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__esp-dl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/who_usb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__usb_host_uvc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__esp_jpeg/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__esp32-camera/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/who_cam/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/who_frame_cap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/who_detect/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__human_face_detect/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__human_face_recognition/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/who_recognition/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/who_lcd/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/who_frame_lcd_disp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/who_recognition_app/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__cmake_utilities/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__button/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__esp_codec_dev/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/lvgl__lvgl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__esp_lvgl_port/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__qma6100p/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/espressif__esp32_s3_eye/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/codexproject/codex-lamp/esp-who/examples/ailamp_face_pose/build_stage3_direct/esp-idf/main/cmake_install.cmake")
endif()

