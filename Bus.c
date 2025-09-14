//
//

#include "Bus.h"
#include "6502cpu.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

//this also inits the cpu i guess?? lol
void bus_init(struct Bus *bus) {
    memset(bus->ram, 0, sizeof(bus->ram));
    bus->cpu.bus = bus; //ConnectBus
    init_cpu(&bus->cpu);
   // bus->cpu.lookup = malloc(sizeof(struct INSTRUCTION)*256);

    //filling up the lookup table manually... should be decently quick...

}

//all address should be 16 bit to address all usable memory
void write_to_bus(struct Bus* bus, uint16_t address, uint8_t data) {
    if (address <= 0xFFFF && address >= 0x0000) {
        bus->ram[address] = data;
    }
}

uint8_t read_from_bus(struct Bus* bus, uint16_t address) {
    if (address <= 0xFFFF && address >= 0x0000) {
        return bus->ram[address];
    }
    return 0x00;

}
