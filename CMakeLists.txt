cmake_minimum_required(VERSION 4.0)
project(Emulator C)

set(CMAKE_C_STANDARD 99)

set(RAYLIB_VERSION 5.5)
find_package(raylib ${RAYLIB_VERSION} QUIET) # QUIET or REQUIRED
if (NOT raylib_FOUND) # If there's none, fetch and build raylib
    include(FetchContent)
    FetchContent_Declare(
            raylib
            DOWNLOAD_EXTRACT_TIMESTAMP OFF
            URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
    )
    FetchContent_GetProperties(raylib)
    if (NOT raylib_POPULATED) # Have we downloaded raylib yet?
        set(FETCHCONTENT_QUIET NO)
        FetchContent_MakeAvailable(raylib)
    endif()
endif()


add_executable(Emulator main.c
        6502cpu.c
        6502cpu.h
        Bus.c
        Bus.h
        DebugAndTesting/nes_debug.c
        DebugAndTesting/nes_debug.h
        DebugAndTesting/nes_debug.c
        DebugAndTesting/nes_debug.h

)

target_link_libraries(Emulator PRIVATE raylib)