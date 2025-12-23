cmake_minimum_required(VERSION 3.10)
project(icm20948 C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(COMPILE_OPTIONS
    -Wall 
    -Wextra 
    -Wpedantic
    -Werror
    -O3
)

option(BUILD_DEMO "Build demo features" OFF)
find_package(GSL REQUIRED)
if(NOT GSL_FOUND)
    message(
        FATAL_ERROR
        "GSL not found - please install GSL development package"
    )
endif()

# add library
add_library(${PROJECT_NAME} src/icm20948.c)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Link libraries (if needed)
# For I2C operations, we typically don't need extra linking on Linux
# But we can add compiler flags if needed
target_compile_options(${PROJECT_NAME} PRIVATE ${COMPILE_OPTIONS})

if (BUILD_DEMO)
    # demo script
    add_executable(sensor_test test/test_sensor_read.c)
    target_link_libraries(
        sensor_test
        PRIVATE
        ${PROJECT_NAME}
        ${GSL_LIBRARIES}
    )

    # create executable for the calibration
    add_executable(calibrate test/test_calibration.c)
    target_link_libraries(
        calibrate
        PRIVATE
        ${PROJECT_NAME}
        ${GSL_LIBRARIES}
    )
endif()

