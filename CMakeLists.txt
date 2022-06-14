cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_geiger C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
	message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

#include(example_auto_set_url.cmake)


add_compile_options(-Wall
		-Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
		-Wno-unused-function # we have some for the docs that aren't called
		-Wno-maybe-uninitialized
		)
		
set(CMAKE_SYSTEM_NAME Generic)		

include_directories("./bitmap")

add_executable(pico-geiger
		pico-geiger.cpp
                pico-oled/pico-oled.cpp
		)

# pull in common dependencies
target_link_libraries(pico-geiger pico_stdlib pico_multicore hardware_i2c hardware_adc hardware_pwm)

# enable usb output, disable uart output
#pico_enable_stdio_usb(pico-geiger 1)
pico_enable_stdio_uart(pico-geiger 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(pico-geiger)

# add url via pico_set_program_url
#example_auto_set_url(pico-geiger)