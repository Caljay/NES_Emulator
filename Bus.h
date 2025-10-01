#ifndef NES_EMULATOR_BUS_H
#define NES_EMULATOR_BUS_H

#include <stdint.h>
#include "cartridge.h"
#include "6502cpu.h"

struct Cartridge;

//this is mainly the NES
struct Bus {

    //the 6502 cpu
    struct cpu cpu;

    //picture processing unit
   // struct ppu_2C02 ppu;
    //audio prcessing unit
    //struct audio audio;

    //the game cartrige and information
    struct Cartridge cartridge;

    uint8_t ram[64*1024]; //may need to be dynamically allocated

};


void bus_init(struct Bus *bus);
void write_to_bus(struct Bus* bus, uint16_t address, uint8_t data);
uint8_t read_from_bus(struct Bus* bus, uint16_t address);
void clock_system(struct Bus* bus);




#endif //NES_EMULATOR_BUS_H