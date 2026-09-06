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
set(CMAKE_PROJECT_NAME CAN_FD_IAP)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Release size tuning for the 20 KiB Bootloader partition.
#
# -fipa-pta improves pointer analysis during LTO, allowing GCC to remove more
# unreachable indirect-call alternatives.
#
# -fno-inline-small-functions avoids duplicating several small protocol helpers
# at their call sites. Explicit always_inline functions are not disabled.
#
# Apply the same options to the Bootloader/application sources and to the
# CubeMX HAL object library. Debug remains unchanged at -O0/-g3.
set(BOOT_RELEASE_SIZE_OPTIONS
    -fipa-pta
    -fno-inline-small-functions
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:${BOOT_RELEASE_SIZE_OPTIONS}>
)

target_compile_options(STM32_Drivers PRIVATE
    $<$<CONFIG:Release>:${BOOT_RELEASE_SIZE_OPTIONS}>
)

# The STM32 startup file already sets MSP and initializes .data/.bss.  This is
# a C-only firmware with no static constructors, so the generic GCC crt0 /
# crtbegin / crtend startup objects are unnecessary in the size build.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:-nostartfiles>
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/bootloader/bootloader.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/bootloader/boot_port_can_stm32g4.c

    
    
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/bootloader

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
