#include <stdio.h>
#include "6502cpu.h"
#include "Bus.h"

int main(void) {
    struct Bus bus;
    bus_init(&bus);


    printf("Hello, World!\n");
    return 0;
}