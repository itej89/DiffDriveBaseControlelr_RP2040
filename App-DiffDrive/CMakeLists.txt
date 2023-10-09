cmake_minimum_required(VERSION 3.14)


# Should we set app-level debugging?
# NOTE Equivalent of `#define DEBUG 1`
if(${DO_DEBUG})
    add_compile_definitions(DEBUG=1)
    message(STATUS "App-side debugging enabled for ${APP_DIFF_DRIVE_NAME}")
else()
    message(STATUS "App-side debugging disabled for ${APP_DIFF_DRIVE_NAME}")
endif()

# Make project data accessible to compiler
add_compile_definitions(APP_NAME="${APP_DIFF_DRIVE_NAME}")
add_compile_definitions(APP_VERSION="${APP_DIFF_DRIVE_VERSION_NUMBER}")
add_compile_definitions(BUILD_NUM=${BUILD_NUMBER})



# Include app source code file(s)
add_executable(${APP_DIFF_DRIVE_NAME}
    ${APP_DIFF_DRIVE_SRC_DIRECTORY}/src/main.cc
    ${APP_DIFF_DRIVE_SRC_DIRECTORY}/src/PID_v1.cpp
)

target_include_directories(${APP_DIFF_DRIVE_NAME} PUBLIC
    include
)

target_include_directories(${APP_DIFF_DRIVE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  # <prefix>/include/mylib
)

# Link to built libraries
target_link_libraries(${APP_DIFF_DRIVE_NAME} LINK_PUBLIC
    pico_stdlib 
    pico_multicore
    hardware_gpio
    hardware_pwm 
    hardware_i2c 
    FreeRTOS)

    

# Enable/disable STDIO via USB and UART
pico_enable_stdio_usb(${APP_DIFF_DRIVE_NAME} 1)
pico_enable_stdio_uart(${APP_DIFF_DRIVE_NAME} 0)

# Enable extra build products
pico_add_extra_outputs(${APP_DIFF_DRIVE_NAME})
