cmake_minimum_required(VERSION 3.13)

# Set the C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)


# ERM Build (ProGCC 3+)
project(PROGCCPLUS_ERM_RP2040 C CXX ASM)

pico_sdk_init()

add_subdirectory(lib/hoja-lib-rp2040)

include_directories(
	include
)

add_executable(${PROJECT_NAME}
	src/main.c
	src/app_rumble_erm.c 
	src/app_imu.c
	src/app_hwtest.c
)

target_compile_definitions(${PROJECT_NAME}
	PRIVATE
	HOJA_PRODUCT="ProGCC 3+"
	HOJA_DEVICE_ID=0xA002
	PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
	PICO_DEFAULT_UART_TX_PIN=12
	PICO_DEFAULT_UART_RX_PIN=13
	#PICO_DEFAULT_BOOT_STAGE2=boot2_generic_03h
	#PICO_FLASH_SPI_CLKDIV=8
	)

target_link_libraries(${PROJECT_NAME}
PRIVATE
hoja_lib
pico_stdlib
pico_bootrom
pico_multicore
pico_platform
pico_rand

hardware_gpio
hardware_adc
hardware_flash
hardware_pio
hardware_dma
hardware_pwm
hardware_spi
hardware_sync
hardware_timer

tinyusb_device
tinyusb_board
)

pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})

# Haptic Feedback Build Prototype
project(PROGCCPLUS_HAPTIC_RP2040 C CXX ASM)

include_directories(
	include
)

add_executable(${PROJECT_NAME}
	src/main.c
	src/app_rumble_haptic.c 
	src/app_imu.c
	src/app_hwtest.c
)

target_compile_definitions(${PROJECT_NAME}
	PRIVATE
	HOJA_DEVICE_ID=0xA003
	PICO_DEFAULT_UART_TX_PIN=12
	PICO_DEFAULT_UART_RX_PIN=13
	HOJA_RUMBLE_TYPE=1 #haptic
	PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
	#PICO_DEFAULT_BOOT_STAGE2=boot2_generic_03h
	#PICO_FLASH_SPI_CLKDIV=8
	)

target_link_libraries(${PROJECT_NAME}
PRIVATE
hoja_lib
pico_stdlib
pico_bootrom
pico_multicore
pico_platform
pico_rand

hardware_gpio
hardware_adc
hardware_flash
hardware_pio
hardware_dma
hardware_pwm
hardware_spi
hardware_sync
hardware_timer

tinyusb_device
tinyusb_board
)

pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})

# 3.1 build 
project(PROGCC3.1_RP2040 C CXX ASM)

include_directories(
	include
)

add_executable(${PROJECT_NAME}
	src/main.c
	src/app_rumble_haptic.c 
	src/app_imu.c
	src/app_hwtest.c
)

target_compile_definitions(${PROJECT_NAME}
	PRIVATE
	HOJA_DEVICE_ID=0xA004
	HOJA_RUMBLE_TYPE=1 #haptic
	PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
	#PICO_DEFAULT_BOOT_STAGE2=boot2_generic_03h
	#PICO_FLASH_SPI_CLKDIV=8
	)

target_link_libraries(${PROJECT_NAME}
PRIVATE
hoja_lib
pico_stdlib
pico_bootrom
pico_multicore
pico_platform
pico_rand

hardware_gpio
hardware_adc
hardware_flash
hardware_pio
hardware_dma
hardware_pwm
hardware_spi
hardware_sync
hardware_timer

tinyusb_device
tinyusb_board
)

pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME})

add_custom_command(TARGET ${PROJECT_NAME}
 					POST_BUILD
					COMMAND ${CMAKE_COMMAND} 
					-DTARGET_NAME=${PROJECT_NAME}
					-DTARGET_PATH=${CMAKE_CURRENT_SOURCE_DIR}
					-P ${CMAKE_SOURCE_DIR}/manifest.cmake
					COMMENT "Updating version in manifest.json"
					WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
					)