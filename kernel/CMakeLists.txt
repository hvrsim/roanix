cmake_minimum_required(VERSION 3.14)

project(
    roanix
    VERSION 0.1.0
    DESCRIPTION "Modern, cross-platform operating system."
    HOMEPAGE_URL "https://github.com/hvrsim/roanix"
    LANGUAGES CXX
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(FRIGG REQUIRED frigg)
pkg_check_modules(FREESTND_C REQUIRED freestnd-c-hdrs-${CMAKE_SYSTEM_PROCESSOR})
pkg_check_modules(FREESTND_CXX REQUIRED freestnd-cxx-hdrs-${CMAKE_SYSTEM_PROCESSOR})

set(
    FREESTND_FLAGS
    "-nostdinc"
    "-ffreestanding"
    "-fno-stack-protector"
    "-fno-stack-check"
    "-fno-lto"
    "-fno-PIC"
    "-ffunction-sections"
    "-fdata-sections"
)

set(KERNEL_CXX_FLAGS "-Wall" "-Wextra" "-fno-exceptions" "-fno-rtti")

set(
    KERNEL_LINK_FLAGS
    "-nostdlib" "-static" "-Wl,-z" 
    "-Wl,max-page-size=0x1000" "-Wl,-gc-sections"
)

set(
    KERNEL_SRC
    source/Main.cpp
    source/Runtime.cpp
)

list(TRANSFORM KERNEL_SRC PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

add_subdirectory(arch/${CMAKE_SYSTEM_PROCESSOR})