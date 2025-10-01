//
//

#ifndef EMULATOR_CARTRIDGE_H
#define EMULATOR_CARTRIDGE_H
#include <stddef.h>
#include <stdint.h>

#include "Bus.h"
struct Bus;


struct Cartridge {

    //first 16 bytes of the rom
    uint8_t header[16];

    //the next 512 bytes if bit 3 of the 7th bit in the header is 1
    uint8_t trainer[512];

    //program data
    uint8_t* program;

    //picture data
    uint8_t* character;

   //mapper number
   uint8_t mapper;
};
void load_cartridge(char* location, struct Bus* nes);



#endif //EMULATOR_CARTRIDGE_H