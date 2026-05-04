# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

set(NAME servo_calibration)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)

project(${NAME} C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(${NAME} servo_cal.cpp )

pico_generate_pio_header(${NAME} ${CMAKE_CURRENT_LIST_DIR}/quadrature_out.pio)

#add_resource(pico_explorer_demo fox.tga)

pico_set_program_name(${NAME} "servo_calibration")
pico_set_program_version(${NAME} "0.1")

#include(common/pimoroni_i2c)
include(common/pimoroni_bus)
include(libraries/pico_explorer/pico_explorer)
#include(libraries/pico_graphics/pico_graphics)
include (libraries/pico_vector/pico_vector)
#include(libraries/bitmap_fonts/bitmap_fonts)
#include(drivers/st7789/st7789)
#include(drivers/analog/analog)
#include(drivers/as5600/as5600)
#include (drivers/tca9548a/tca9548a)
include(drivers/servo/servo)
include(libraries/servo2040/servo2040)
include(drivers/encoder/encoder)
include(drivers/button/button)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${NAME} 0)
pico_enable_stdio_usb(${NAME} 1)

# Add the standard library to the build
target_link_libraries(${NAME}
    #pimoroni_i2c
    pico_explorer # <-- List libraries here!
    button
    pico_stdlib
    servo2040
    hardware_pwm
    #hardware_i2c
    #hardware_spi
    #hardware_dma
    #hardware_pio
    encoder
    #as5600
    #bmp280
    pico_graphics
    bitmap_fonts
    pico_vector
    st7789
)

# Add the standard include files to the build
target_include_directories(${NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(${NAME})

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)