#include <stdio.h>
#include "6502cpu.h"
#include "Bus.h"
#include <raylib.h>

#include "cartridge.h"
#include "DebugAndTesting/nes_debug.h"

//helper function to load the testing program
//needs to be much better
void load_program(struct cpu* cpu) {


    //reset vectors
    cpu->bus->ram[0xFFFC] = 0x00;
    cpu->bus->ram[0xFFFD] = 0xC0;
    cpu_reset(cpu);

}


int main(void) {

    struct Bus nes;
    bus_init(&nes);
    load_cartridge("nestest.nes", &nes);
    struct cpu cpu = nes.cpu;
    load_program(&cpu);

   // init_debug_window();

    int i = 5000;
    size_t count = 0;
    while (i > 0) {
    //WaitTime(0.125f);
       // if (write_nes_debug(&cpu)) break;

        if (cpu.cycles == 0) {
            count++;
            printf("PC: %04x A:%02x X:%02x Y:%02x SP:%02x Count = %d\n", cpu.pc, cpu.a, cpu.x, cpu.y, cpu.sp, count);
            cpu_clock(&cpu);
            i--;

        }
        else {
            cpu_clock(&cpu);

        }

    }

    printf("A reg: %d\n", cpu.a );
    printf("X reg: %d\n", cpu.x );
    printf("Y reg: %d\n", cpu.y );

    printf("Hello, World!\n");
    return 0;
}

