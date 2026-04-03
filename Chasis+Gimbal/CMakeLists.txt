cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Chasis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        bsp/Src/bsp_CAN.c
        bsp/Src/spi1.c
        bsp/Src/USB.c
        Algorithm/FIR.c
        Algorithm/PID.c
        Module/BMI088/BMI088Driver.c
        Module/BMI088/BMI088Middleware.c
        Module/AHRS/MahonyAHRS.c
        Module/DT7/DT7.c
        Module/GM3508/GM3508.c
        Module/GM6020/GM6020.c
        Module/USB_Vision/USB_Vision.c
        APP/RC/RC.c
        APP/Chasis/Chasis.c
        APP/INS/INS.c
        APP/Vision/Vison.c
        APP/GIMBAL/GIMBAL.c
)


# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        APP
        APP/RC
        APP/Chasis
        APP/GIMBAL
        Module/AHRS
        Module/BMI088
        Module/DT7
        Module/GM3508
        Module/GM6020
        Module/USB_Vision
        APP/INS
        APP/Vision
        Algorithm
        Data
        bsp/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
