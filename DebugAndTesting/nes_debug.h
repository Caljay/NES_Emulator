//
//

#ifndef NES_EMULATOR_NES_DEBUG_H
#define NES_EMULATOR_NES_DEBUG_H
#include "../6502cpu.h"

//a separate window that will hold some debug information
//this is going to create the thread that will work on drawing the debug information
void init_debug_window();
int write_nes_debug(struct cpu* cpu);



#endif //NES_EMULATOR_NES_DEBUG_H