#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "Bus.h"
//THIS IS CURRENTLY HARDCODED
void set_memory(struct Bus* nes) {
    //this shold be memory mapped from the cpu
    //but... this also apepars to work somewhat
    uint16_t  start = 0xc000;

    for (size_t t = 0; t < 16384*nes->cartridge.header[4]; t++) {
        nes->ram[start + t] = *(nes->cartridge.program + t);

    }
//getchar();

}
void load_cartridge(char *location, struct Bus* nes) {
    //  nes->cartridge.header;

    FILE* rom_location = fopen(location, "rb");

    //implement this function

    for (size_t i = 0; i < 16; i++) {

        fread(&(nes->cartridge.header[i]), 1, sizeof(uint8_t), rom_location);
    }


    //if bit 3 of the 7th byte of this header is 1, the 512 byte traine needs to be loadded in 0x7000
    if (nes->cartridge.header[6] & (1 << 3)) {
        for (size_t i = 0; i < 512; i++) {

            fread(&(nes->cartridge.trainer[i]), 1, sizeof(uint8_t), rom_location);
        }
    }

    //the size of the program section of the rom is 16KiB * the value in header location 5 (index 4)
    nes->cartridge.program = malloc(sizeof(uint8_t) * 16384*nes->cartridge.header[4]);

    for (size_t i = 0; i < 16384 * nes->cartridge.header[4]; i++) {

        fread((nes->cartridge.program+i), 1, sizeof(uint8_t), rom_location);

    }

    //size of the character section of the rom is 8KiB * the value in header location 6 (index 5)
    nes->cartridge.character = malloc(sizeof(uint8_t) * 16384*nes->cartridge.header[5]);

    for (size_t i = 0; i < 16384 * nes->cartridge.header[5]; i++) {

        fread((nes->cartridge.character+i), 1, sizeof(uint8_t), rom_location);

    }

    set_memory(nes);
    //getchar();






}
