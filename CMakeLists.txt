cmake_minimum_required(VERSION 4.0)
project(NES_Emulator C)

set(CMAKE_C_STANDARD 99)

add_executable(NES_Emulator main.c
        6502cpu.c
        6502cpu.h
        Bus.c
        Bus.h)
