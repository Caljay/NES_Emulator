//
//

#ifndef NES_EMULATOR_6502CPU_H
#define NES_EMULATOR_6502CPU_H
#include <stdint.h>

struct Bus;
struct cpu;
struct INSTRUCTION {
    //std::string name //name of the opcode for disassmbler
    uint8_t (*operation)(struct cpu* cpu);
    uint8_t (*addrmode)(struct cpu* cpu);
    uint8_t cycles;

} INSTRUCTION;
struct cpu {

    //registers
    uint16_t pc; //program coutner
    uint8_t sp; //stack pointer
    uint8_t a; //accumulator
    uint8_t x; //x register
    uint8_t y; //y register
    uint8_t status; //status flags

//device
    struct Bus* bus;
    uint8_t fetched;

    uint16_t addr_abs;
    uint16_t addr_rel;
    uint8_t opcode; //the current opcode that is being worked on
    uint8_t cycles; //amount of clock cycles left for the current instructions





    struct INSTRUCTION lookup[256];
};
enum FLAGS6502 {
    C = (1<<0), //carry
    Z = (1<<1), //zero
    I = (1<<2), //disable ints
    D = (1<<3), //decimal mode (unused currently
    B = (1<<4), //break
    U = (1<<5), //unused
    V = (1<<6), //overflow
    N = (1<<7), //negative





}; //flags for the status register




void init_cpu(struct cpu* cpu);

//CPU BUS FUNCTIONS
void cpu_connect_bus(struct cpu* cpu, struct Bus* bus); //NOT IMPLEMENTED
void connect_bus(struct cpu* cpu, struct Bus* bus);
uint8_t cpu_read_bus(struct cpu* cpu, uint16_t addr);
void cpu_write_bus(struct cpu* cpu, uint16_t addr, uint8_t data);


//CPU FUNCTIONS
void cpu_reset(struct cpu* cpu);
void cpu_clock(struct cpu* cpu);
void cpu_irq(struct cpu* cpu);
void cpu_nmi(struct cpu* cpu);

uint8_t get_flag(struct cpu* cpu, enum FLAGS6502 flag);
void set_flag(struct cpu* cpu, enum FLAGS6502 flag, uint8_t enable);

uint8_t fetch(struct cpu* cpu); //fetches the next instruction/data


// ADDRESSING MODES (look at datasheet)
uint8_t IMP(struct cpu* cpu); uint8_t IMM(struct cpu* cpu); uint8_t ZP0(struct cpu* cpu);
uint8_t ZPX(struct cpu* cpu); uint8_t ZPY(struct cpu* cpu); uint8_t REL(struct cpu* cpu);
uint8_t ABS(struct cpu* cpu); uint8_t ABX(struct cpu* cpu); uint8_t ABY(struct cpu* cpu);
uint8_t IND(struct cpu* cpu); uint8_t IZX(struct cpu* cpu); uint8_t IZY(struct cpu* cpu);


// OPCODES (look at datasheet)
uint8_t ADC(struct cpu* cpu); uint8_t AND(struct cpu* cpu); uint8_t ASL(struct cpu* cpu);
uint8_t BCC(struct cpu* cpu); uint8_t BCS(struct cpu* cpu); uint8_t BEQ(struct cpu* cpu);
uint8_t BIT(struct cpu* cpu); uint8_t BMI(struct cpu* cpu); uint8_t BNE(struct cpu* cpu);
uint8_t BPL(struct cpu* cpu); uint8_t BRK(struct cpu* cpu); uint8_t BVC(struct cpu* cpu);
uint8_t BVS(struct cpu* cpu); uint8_t CLC(struct cpu* cpu); uint8_t CLD(struct cpu* cpu);
uint8_t CLI(struct cpu* cpu); uint8_t CLV(struct cpu* cpu); uint8_t CMP(struct cpu* cpu);
uint8_t CPX(struct cpu* cpu); uint8_t CPY(struct cpu* cpu); uint8_t DEC(struct cpu* cpu);
uint8_t DEX(struct cpu* cpu); uint8_t DEY(struct cpu* cpu); uint8_t EOR(struct cpu* cpu);
uint8_t INC(struct cpu* cpu); uint8_t INX(struct cpu* cpu); uint8_t INY(struct cpu* cpu);
uint8_t JMP(struct cpu* cpu); uint8_t JSR(struct cpu* cpu); uint8_t LDA(struct cpu* cpu);
uint8_t LDX(struct cpu* cpu); uint8_t LDY(struct cpu* cpu); uint8_t LSR(struct cpu* cpu);
uint8_t NOP(struct cpu* cpu); uint8_t ORA(struct cpu* cpu); uint8_t PHA(struct cpu* cpu);
uint8_t PHP(struct cpu* cpu); uint8_t PLA(struct cpu* cpu); uint8_t PLP(struct cpu* cpu);
uint8_t ROL(struct cpu* cpu); uint8_t ROR(struct cpu* cpu); uint8_t RTI(struct cpu* cpu);
uint8_t RTS(struct cpu* cpu); uint8_t SBC(struct cpu* cpu); uint8_t SEC(struct cpu* cpu);
uint8_t SED(struct cpu* cpu); uint8_t SEI(struct cpu* cpu); uint8_t STA(struct cpu* cpu);
uint8_t STX(struct cpu* cpu); uint8_t STY(struct cpu* cpu); uint8_t TAX(struct cpu* cpu);
uint8_t TAY(struct cpu* cpu); uint8_t TSX(struct cpu* cpu); uint8_t TXA(struct cpu* cpu);
uint8_t TXS(struct cpu* cpu); uint8_t TYA(struct cpu* cpu);

uint8_t XXX(struct cpu* cpu); //catch all in case stuff breaks same as NOP

























#endif //NES_EMULATOR_6502CPU_H