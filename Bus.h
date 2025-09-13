//
//

#ifndef NES_EMULATOR_BUS_H
#define NES_EMULATOR_BUS_H
#include <stdint.h>

#include "6502cpu.h"


struct Bus {

    //devices here

    struct cpu cpu;
    uint8_t ram[64*1024]; //may need to be dynamically allocated

};


void bus_init(struct Bus *bus);
void write_to_bus(struct Bus* bus, uint16_t address, uint8_t data);
uint8_t read_from_bus(struct Bus* bus, uint16_t address);




#endif //NES_EMULATOR_BUS_H