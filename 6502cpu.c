//
//

#include "6502cpu.h"

#include <stdlib.h>

#include "Bus.h"



void init_cpu(struct cpu *cpu) {
cpu->lookup[0] = (struct INSTRUCTION){ &BRK, &IMM, 7 };
cpu->lookup[1] = (struct INSTRUCTION){ &ORA, &IZX, 6 };
cpu->lookup[2] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[3] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[4] = (struct INSTRUCTION){ &NOP, &IMP, 3 };
cpu->lookup[5] = (struct INSTRUCTION){ &ORA, &ZP0, 3 };
cpu->lookup[6] = (struct INSTRUCTION){ &ASL, &ZP0, 5 };
cpu->lookup[7] = (struct INSTRUCTION){ &XXX, &IMP, 5 };
cpu->lookup[8] = (struct INSTRUCTION){ &PHP, &IMP, 3 };
cpu->lookup[9] = (struct INSTRUCTION){ &ORA, &IMM, 2 };
cpu->lookup[10] = (struct INSTRUCTION){ &ASL, &IMP, 2 };
cpu->lookup[11] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[12] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[13] = (struct INSTRUCTION){ &ORA, &ABS, 4 };
cpu->lookup[14] = (struct INSTRUCTION){ &ASL, &ABS, 6 };
cpu->lookup[15] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[16] = (struct INSTRUCTION){ &BPL, &REL, 2 };
cpu->lookup[17] = (struct INSTRUCTION){ &ORA, &IZY, 5 };
cpu->lookup[18] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[19] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[20] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[21] = (struct INSTRUCTION){ &ORA, &ZPX, 4 };
cpu->lookup[22] = (struct INSTRUCTION){ &ASL, &ZPX, 6 };
cpu->lookup[23] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[24] = (struct INSTRUCTION){ &CLC, &IMP, 2 };
cpu->lookup[25] = (struct INSTRUCTION){ &ORA, &ABY, 4 };
cpu->lookup[26] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[27] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[28] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[29] = (struct INSTRUCTION){ &ORA, &ABX, 4 };
cpu->lookup[30] = (struct INSTRUCTION){ &ASL, &ABX, 7 };
cpu->lookup[31] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[32] = (struct INSTRUCTION){ &JSR, &ABS, 6 };
cpu->lookup[33] = (struct INSTRUCTION){ &AND, &IZX, 6 };
cpu->lookup[34] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[35] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[36] = (struct INSTRUCTION){ &BIT, &ZP0, 3 };
cpu->lookup[37] = (struct INSTRUCTION){ &AND, &ZP0, 3 };
cpu->lookup[38] = (struct INSTRUCTION){ &ROL, &ZP0, 5 };
cpu->lookup[39] = (struct INSTRUCTION){ &XXX, &IMP, 5 };
cpu->lookup[40] = (struct INSTRUCTION){ &PLP, &IMP, 4 };
cpu->lookup[41] = (struct INSTRUCTION){ &AND, &IMM, 2 };
cpu->lookup[42] = (struct INSTRUCTION){ &ROL, &IMP, 2 };
cpu->lookup[43] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[44] = (struct INSTRUCTION){ &BIT, &ABS, 4 };
cpu->lookup[45] = (struct INSTRUCTION){ &AND, &ABS, 4 };
cpu->lookup[46] = (struct INSTRUCTION){ &ROL, &ABS, 6 };
cpu->lookup[47] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[48] = (struct INSTRUCTION){ &BMI, &REL, 2 };
cpu->lookup[49] = (struct INSTRUCTION){ &AND, &IZY, 5 };
cpu->lookup[50] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[51] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[52] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[53] = (struct INSTRUCTION){ &AND, &ZPX, 4 };
cpu->lookup[54] = (struct INSTRUCTION){ &ROL, &ZPX, 6 };
cpu->lookup[55] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[56] = (struct INSTRUCTION){ &SEC, &IMP, 2 };
cpu->lookup[57] = (struct INSTRUCTION){ &AND, &ABY, 4 };
cpu->lookup[58] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[59] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[60] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[61] = (struct INSTRUCTION){ &AND, &ABX, 4 };
cpu->lookup[62] = (struct INSTRUCTION){ &ROL, &ABX, 7 };
cpu->lookup[63] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[64] = (struct INSTRUCTION){ &RTI, &IMP, 6 };
cpu->lookup[65] = (struct INSTRUCTION){ &EOR, &IZX, 6 };
cpu->lookup[66] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[67] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[68] = (struct INSTRUCTION){ &NOP, &IMP, 3 };
cpu->lookup[69] = (struct INSTRUCTION){ &EOR, &ZP0, 3 };
cpu->lookup[70] = (struct INSTRUCTION){ &LSR, &ZP0, 5 };
cpu->lookup[71] = (struct INSTRUCTION){ &XXX, &IMP, 5 };
cpu->lookup[72] = (struct INSTRUCTION){ &PHA, &IMP, 3 };
cpu->lookup[73] = (struct INSTRUCTION){ &EOR, &IMM, 2 };
cpu->lookup[74] = (struct INSTRUCTION){ &LSR, &IMP, 2 };
cpu->lookup[75] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[76] = (struct INSTRUCTION){ &JMP, &ABS, 3 };
cpu->lookup[77] = (struct INSTRUCTION){ &EOR, &ABS, 4 };
cpu->lookup[78] = (struct INSTRUCTION){ &LSR, &ABS, 6 };
cpu->lookup[79] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[80] = (struct INSTRUCTION){ &BVC, &REL, 2 };
cpu->lookup[81] = (struct INSTRUCTION){ &EOR, &IZY, 5 };
cpu->lookup[82] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[83] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[84] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[85] = (struct INSTRUCTION){ &EOR, &ZPX, 4 };
cpu->lookup[86] = (struct INSTRUCTION){ &LSR, &ZPX, 6 };
cpu->lookup[87] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[88] = (struct INSTRUCTION){ &CLI, &IMP, 2 };
cpu->lookup[89] = (struct INSTRUCTION){ &EOR, &ABY, 4 };
cpu->lookup[90] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[91] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[92] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[93] = (struct INSTRUCTION){ &EOR, &ABX, 4 };
cpu->lookup[94] = (struct INSTRUCTION){ &LSR, &ABX, 7 };
cpu->lookup[95] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[96] = (struct INSTRUCTION){ &RTS, &IMP, 6 };
cpu->lookup[97] = (struct INSTRUCTION){ &ADC, &IZX, 6 };
cpu->lookup[98] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[99] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[100] = (struct INSTRUCTION){ &NOP, &IMP, 3 };
cpu->lookup[101] = (struct INSTRUCTION){ &ADC, &ZP0, 3 };
cpu->lookup[102] = (struct INSTRUCTION){ &ROR, &ZP0, 5 };
cpu->lookup[103] = (struct INSTRUCTION){ &XXX, &IMP, 5 };
cpu->lookup[104] = (struct INSTRUCTION){ &PLA, &IMP, 4 };
cpu->lookup[105] = (struct INSTRUCTION){ &ADC, &IMM, 2 };
cpu->lookup[106] = (struct INSTRUCTION){ &ROR, &IMP, 2 };
cpu->lookup[107] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[108] = (struct INSTRUCTION){ &JMP, &IND, 5 };
cpu->lookup[109] = (struct INSTRUCTION){ &ADC, &ABS, 4 };
cpu->lookup[110] = (struct INSTRUCTION){ &ROR, &ABS, 6 };
cpu->lookup[111] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[112] = (struct INSTRUCTION){ &BVS, &REL, 2 };
cpu->lookup[113] = (struct INSTRUCTION){ &ADC, &IZY, 5 };
cpu->lookup[114] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[115] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[116] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[117] = (struct INSTRUCTION){ &ADC, &ZPX, 4 };
cpu->lookup[118] = (struct INSTRUCTION){ &ROR, &ZPX, 6 };
cpu->lookup[119] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[120] = (struct INSTRUCTION){ &SEI, &IMP, 2 };
cpu->lookup[121] = (struct INSTRUCTION){ &ADC, &ABY, 4 };
cpu->lookup[122] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[123] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[124] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[125] = (struct INSTRUCTION){ &ADC, &ABX, 4 };
cpu->lookup[126] = (struct INSTRUCTION){ &ROR, &ABX, 7 };
cpu->lookup[127] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[128] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[129] = (struct INSTRUCTION){ &STA, &IZX, 6 };
cpu->lookup[130] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[131] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[132] = (struct INSTRUCTION){ &STY, &ZP0, 3 };
cpu->lookup[133] = (struct INSTRUCTION){ &STA, &ZP0, 3 };
cpu->lookup[134] = (struct INSTRUCTION){ &STX, &ZP0, 3 };
cpu->lookup[135] = (struct INSTRUCTION){ &XXX, &IMP, 3 };
cpu->lookup[136] = (struct INSTRUCTION){ &DEY, &IMP, 2 };
cpu->lookup[137] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[138] = (struct INSTRUCTION){ &TXA, &IMP, 2 };
cpu->lookup[139] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[140] = (struct INSTRUCTION){ &STY, &ABS, 4 };
cpu->lookup[141] = (struct INSTRUCTION){ &STA, &ABS, 4 };
cpu->lookup[142] = (struct INSTRUCTION){ &STX, &ABS, 4 };
cpu->lookup[143] = (struct INSTRUCTION){ &XXX, &IMP, 4 };
cpu->lookup[144] = (struct INSTRUCTION){ &BCC, &REL, 2 };
cpu->lookup[145] = (struct INSTRUCTION){ &STA, &IZY, 6 };
cpu->lookup[146] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[147] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[148] = (struct INSTRUCTION){ &STY, &ZPX, 4 };
cpu->lookup[149] = (struct INSTRUCTION){ &STA, &ZPX, 4 };
cpu->lookup[150] = (struct INSTRUCTION){ &STX, &ZPY, 4 };
cpu->lookup[151] = (struct INSTRUCTION){ &XXX, &IMP, 4 };
cpu->lookup[152] = (struct INSTRUCTION){ &TYA, &IMP, 2 };
cpu->lookup[153] = (struct INSTRUCTION){ &STA, &ABY, 5 };
cpu->lookup[154] = (struct INSTRUCTION){ &TXS, &IMP, 2 };
cpu->lookup[155] = (struct INSTRUCTION){ &XXX, &IMP, 5 };
cpu->lookup[156] = (struct INSTRUCTION){ &NOP, &IMP, 5 };
cpu->lookup[157] = (struct INSTRUCTION){ &STA, &ABX, 5 };
cpu->lookup[158] = (struct INSTRUCTION){ &XXX, &IMP, 5 };
cpu->lookup[159] = (struct INSTRUCTION){ &XXX, &IMP, 5 };
cpu->lookup[160] = (struct INSTRUCTION){ &LDY, &IMM, 2 };
cpu->lookup[161] = (struct INSTRUCTION){ &LDA, &IZX, 6 };
cpu->lookup[162] = (struct INSTRUCTION){ &LDX, &IMM, 2 };
cpu->lookup[163] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[164] = (struct INSTRUCTION){ &LDY, &ZP0, 3 };
cpu->lookup[165] = (struct INSTRUCTION){ &LDA, &ZP0, 3 };
cpu->lookup[166] = (struct INSTRUCTION){ &LDX, &ZP0, 3 };
cpu->lookup[167] = (struct INSTRUCTION){ &XXX, &IMP, 3 };
cpu->lookup[168] = (struct INSTRUCTION){ &TAY, &IMP, 2 };
cpu->lookup[169] = (struct INSTRUCTION){ &LDA, &IMM, 2 };
cpu->lookup[170] = (struct INSTRUCTION){ &TAX, &IMP, 2 };
cpu->lookup[171] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[172] = (struct INSTRUCTION){ &LDY, &ABS, 4 };
cpu->lookup[173] = (struct INSTRUCTION){ &LDA, &ABS, 4 };
cpu->lookup[174] = (struct INSTRUCTION){ &LDX, &ABS, 4 };
cpu->lookup[175] = (struct INSTRUCTION){ &XXX, &IMP, 4 };
cpu->lookup[176] = (struct INSTRUCTION){ &BCS, &REL, 2 };
cpu->lookup[177] = (struct INSTRUCTION){ &LDA, &IZY, 5 };
cpu->lookup[178] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[179] = (struct INSTRUCTION){ &XXX, &IMP, 5 };
cpu->lookup[180] = (struct INSTRUCTION){ &LDY, &ZPX, 4 };
cpu->lookup[181] = (struct INSTRUCTION){ &LDA, &ZPX, 4 };
cpu->lookup[182] = (struct INSTRUCTION){ &LDX, &ZPY, 4 };
cpu->lookup[183] = (struct INSTRUCTION){ &XXX, &IMP, 4 };
cpu->lookup[184] = (struct INSTRUCTION){ &CLV, &IMP, 2 };
cpu->lookup[185] = (struct INSTRUCTION){ &LDA, &ABY, 4 };
cpu->lookup[186] = (struct INSTRUCTION){ &TSX, &IMP, 2 };
cpu->lookup[187] = (struct INSTRUCTION){ &XXX, &IMP, 4 };
cpu->lookup[188] = (struct INSTRUCTION){ &LDY, &ABX, 4 };
cpu->lookup[189] = (struct INSTRUCTION){ &LDA, &ABX, 4 };
cpu->lookup[190] = (struct INSTRUCTION){ &LDX, &ABY, 4 };
cpu->lookup[191] = (struct INSTRUCTION){ &XXX, &IMP, 4 };
cpu->lookup[192] = (struct INSTRUCTION){ &CPY, &IMM, 2 };
cpu->lookup[193] = (struct INSTRUCTION){ &CMP, &IZX, 6 };
cpu->lookup[194] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[195] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[196] = (struct INSTRUCTION){ &CPY, &ZP0, 3 };
cpu->lookup[197] = (struct INSTRUCTION){ &CMP, &ZP0, 3 };
cpu->lookup[198] = (struct INSTRUCTION){ &DEC, &ZP0, 5 };
cpu->lookup[199] = (struct INSTRUCTION){ &XXX, &IMP, 5 };
cpu->lookup[200] = (struct INSTRUCTION){ &INY, &IMP, 2 };
cpu->lookup[201] = (struct INSTRUCTION){ &CMP, &IMM, 2 };
cpu->lookup[202] = (struct INSTRUCTION){ &DEX, &IMP, 2 };
cpu->lookup[203] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[204] = (struct INSTRUCTION){ &CPY, &ABS, 4 };
cpu->lookup[205] = (struct INSTRUCTION){ &CMP, &ABS, 4 };
cpu->lookup[206] = (struct INSTRUCTION){ &DEC, &ABS, 6 };
cpu->lookup[207] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[208] = (struct INSTRUCTION){ &BNE, &REL, 2 };
cpu->lookup[209] = (struct INSTRUCTION){ &CMP, &IZY, 5 };
cpu->lookup[210] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[211] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[212] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[213] = (struct INSTRUCTION){ &CMP, &ZPX, 4 };
cpu->lookup[214] = (struct INSTRUCTION){ &DEC, &ZPX, 6 };
cpu->lookup[215] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[216] = (struct INSTRUCTION){ &CLD, &IMP, 2 };
cpu->lookup[217] = (struct INSTRUCTION){ &CMP, &ABY, 4 };
cpu->lookup[218] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[219] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[220] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[221] = (struct INSTRUCTION){ &CMP, &ABX, 4 };
cpu->lookup[222] = (struct INSTRUCTION){ &DEC, &ABX, 7 };
cpu->lookup[223] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[224] = (struct INSTRUCTION){ &CPX, &IMM, 2 };
cpu->lookup[225] = (struct INSTRUCTION){ &SBC, &IZX, 6 };
cpu->lookup[226] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[227] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[228] = (struct INSTRUCTION){ &CPX, &ZP0, 3 };
cpu->lookup[229] = (struct INSTRUCTION){ &SBC, &ZP0, 3 };
cpu->lookup[230] = (struct INSTRUCTION){ &INC, &ZP0, 5 };
cpu->lookup[231] = (struct INSTRUCTION){ &XXX, &IMP, 5 };
cpu->lookup[232] = (struct INSTRUCTION){ &INX, &IMP, 2 };
cpu->lookup[233] = (struct INSTRUCTION){ &SBC, &IMM, 2 };
cpu->lookup[234] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[235] = (struct INSTRUCTION){ &SBC, &IMP, 2 };
cpu->lookup[236] = (struct INSTRUCTION){ &CPX, &ABS, 4 };
cpu->lookup[237] = (struct INSTRUCTION){ &SBC, &ABS, 4 };
cpu->lookup[238] = (struct INSTRUCTION){ &INC, &ABS, 6 };
cpu->lookup[239] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[240] = (struct INSTRUCTION){ &BEQ, &REL, 2 };
cpu->lookup[241] = (struct INSTRUCTION){ &SBC, &IZY, 5 };
cpu->lookup[242] = (struct INSTRUCTION){ &XXX, &IMP, 2 };
cpu->lookup[243] = (struct INSTRUCTION){ &XXX, &IMP, 8 };
cpu->lookup[244] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[245] = (struct INSTRUCTION){ &SBC, &ZPX, 4 };
cpu->lookup[246] = (struct INSTRUCTION){ &INC, &ZPX, 6 };
cpu->lookup[247] = (struct INSTRUCTION){ &XXX, &IMP, 6 };
cpu->lookup[248] = (struct INSTRUCTION){ &SED, &IMP, 2 };
cpu->lookup[249] = (struct INSTRUCTION){ &SBC, &ABY, 4 };
cpu->lookup[250] = (struct INSTRUCTION){ &NOP, &IMP, 2 };
cpu->lookup[251] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
cpu->lookup[252] = (struct INSTRUCTION){ &NOP, &IMP, 4 };
cpu->lookup[253] = (struct INSTRUCTION){ &SBC, &ABX, 4 };
cpu->lookup[254] = (struct INSTRUCTION){ &INC, &ABX, 7 };
cpu->lookup[255] = (struct INSTRUCTION){ &XXX, &IMP, 7 };
    
}

void connect_bus(struct cpu *cpu, struct Bus *bus) {

}

uint8_t cpu_read_bus(struct cpu *cpu, uint16_t addr) {
return read_from_bus(cpu->bus, addr);
}

void cpu_write_bus(struct cpu *cpu, uint16_t addr, uint8_t data) {
    write_to_bus(cpu->bus, addr, data);
}

void cpu_reset(struct cpu *cpu) {
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;

    cpu->sp = 0xFD;
    cpu->status = 0x00 | U;

    cpu->addr_abs = 0xFFFC;
    uint16_t low = read_from_bus(cpu->bus, cpu->addr_abs);
    uint16_t high = read_from_bus(cpu->bus, cpu->addr_abs + 1);

    cpu->pc = (high << 8) | low;
    cpu->addr_rel = 0x0000;
    cpu->addr_abs = 0x0000;
    cpu->fetched = 0x00;

    cpu->cycles = 8;



}

//not clock cycle accurate
void cpu_clock(struct cpu *cpu) {
    //if the current instruction is "finished"
    if (cpu->cycles == 0) {
        //read the opcode
        cpu->opcode = cpu_read_bus(cpu, cpu->pc);

        //move the pc up
        cpu->pc++;

        //get the clock count for this instruction
        cpu->cycles = cpu->lookup[cpu->opcode].cycles;

        //this calls a function getting the addrmode
        uint8_t additional_clock1 = (cpu->lookup[cpu->opcode].addrmode)(cpu);
        //and the operation
       uint8_t additional_clock2 = (cpu->lookup[cpu->opcode].operation)(cpu);

        cpu->cycles += (additional_clock1 & additional_clock2);

    }
    cpu->cycles--;
}

void cpu_irq(struct cpu *cpu) {
    if (get_flag(cpu, I) == 0) {
        cpu_write_bus(cpu, 0x0100+cpu->sp,(cpu->pc >> 8)& 0x00FF);
        cpu->sp--;
        cpu_write_bus(cpu, 0x0100+cpu->sp,(cpu->pc >> 8)& 0x00FF);
        cpu->sp--;

        set_flag(cpu, B, 0);
        set_flag(cpu, U, 1);
        set_flag(cpu, I, 1);

        cpu_write_bus(cpu, 0x0100+cpu->sp,cpu->status);
        cpu->sp--;


        cpu->addr_abs = 0xFFFE;
        uint16_t low = read_from_bus(cpu->bus, cpu->addr_abs);
        uint16_t high = read_from_bus(cpu->bus, cpu->addr_abs + 1);
        cpu->pc = (high << 8) | low;

        cpu->cycles = 7;

    }



}

void cpu_nmi(struct cpu *cpu) {

    cpu_write_bus(cpu, 0x0100+cpu->sp,(cpu->pc >> 8)& 0x00FF);
    cpu->sp--;
    cpu_write_bus(cpu, 0x0100+cpu->sp,(cpu->pc >> 8)& 0x00FF);
    cpu->sp--;

    set_flag(cpu, B, 0);
    set_flag(cpu, U, 1);
    set_flag(cpu, I, 1);

    cpu_write_bus(cpu, 0x0100+cpu->sp,cpu->status);
    cpu->sp--;


    cpu->addr_abs = 0xFFFA;
    uint16_t low = read_from_bus(cpu->bus, cpu->addr_abs);
    uint16_t high = read_from_bus(cpu->bus, cpu->addr_abs + 1);
    cpu->pc = (high << 8) | low;

    cpu->cycles = 7;


}

uint8_t get_flag(struct cpu *cpu, enum FLAGS6502 flag) {
        #error  "get_flag is not yet implemented"
    }
void set_flag(struct cpu *cpu, enum FLAGS6502 flag, uint8_t enable) {
    uint8_t temp = cpu->status;
    cpu->status = 0x0000;
    cpu->status = (flag) | temp;
    #error  "set_flag is not yet implemented"

}

// ADDRESSING MODES (look at datasheet)
//open a function to see more details on the addressing mode
uint8_t IMP(struct cpu* cpu) {
//implied adressing - address containing operand is impliclty stated in the operation code of instruction
    cpu->fetched = cpu->a;
    return 0;



} //implied
uint8_t IMM(struct cpu* cpu) {
    //immediate mode - second byte contains the operand nothing else needed
    cpu->addr_abs = cpu->pc++;
    return 0;

} //immediate
uint8_t ZP0(struct cpu* cpu) {
//zero page addressing - assumes high byte of second instruction is 0x00 and gets the lower byte
//therefore on the first page, much faster

    cpu->addr_abs = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;
    cpu->addr_abs &= 0x00FF;
    return 0;


} //zero page
uint8_t ZPX(struct cpu* cpu) {
    //zero paging with X register offset



    cpu->addr_abs = cpu_read_bus(cpu, cpu->pc +cpu->x);
    cpu->pc++;
    cpu->addr_abs &= 0x00FF;
    return 0;

} //zero page x offset
uint8_t ZPY(struct cpu* cpu) {
    //zero page adressing with Y register offset

    cpu->addr_abs = cpu_read_bus(cpu, cpu->pc +cpu->y);
    cpu->pc++;
    cpu->addr_abs &= 0x00FF;
    return 0;
} //zero page y offset
uint8_t ABS(struct cpu* cpu) {
    //absolute adressing, second byte gives high bits, third gives low bits

    uint8_t high = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;
    uint8_t low = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;
    cpu->addr_abs = (high << 8) | low; //this is a really clever usage of bitwise operations

    return 0;


} //absolute
uint8_t ABX(struct cpu* cpu) {
    //absolute with x offset

    uint8_t high = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;
    uint8_t low = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;

    cpu->addr_abs = ((high << 8) | low) +cpu->x;

    if ((cpu->addr_abs & 0xFF00) != (high << 8)) {
        return 1;
    }
    else return 0;







} //aboslute x offset
uint8_t ABY(struct cpu* cpu) {
//aboslute with y offset
    uint8_t high = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;
    uint8_t low = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;

    cpu->addr_abs = ((high << 8) | low) +cpu->y;

    if ((cpu->addr_abs & 0xFF00) != (high << 8)) {
        return 1;
    }
    else return 0;
} //absolute y offset
uint8_t IND(struct cpu* cpu) {
    //indirect addressing
    //basically gives a pointer, to the location that holds the real location desired

    uint16_t ptr_low = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;
    uint16_t ptr_high = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;

    uint16_t ptr = (ptr_high << 8) |ptr_low;

    //paging changing hardware bug here...
    //this would change the page
    if (ptr_low == 0x00FF) { //simi;ate hardware bug
        cpu->addr_abs = (cpu_read_bus(cpu, ptr & 0xFF00) << 8) | (cpu_read_bus(cpu, ptr));
    }
    else {
    cpu->addr_abs = (cpu_read_bus(cpu, ptr + 1) << 8) | (cpu_read_bus(cpu, ptr));

    }
return 0;
} //indirect
uint8_t IZX(struct cpu* cpu) {
    //indriect zero paging with x offset
    uint16_t t = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;
    uint16_t ptr_low = cpu_read_bus(cpu, (uint16_t)(t+(uint16_t)cpu->x) & 0x00FF);
    uint16_t ptr_high = cpu_read_bus(cpu, (uint16_t)(t+(uint16_t)cpu->x+1) & 0x00FF);

    cpu->addr_abs = (ptr_high << 8) | ptr_low;
return 0;

} //indirect zero page w x offset
uint8_t IZY(struct cpu* cpu) {
    //indriect zero paging with y offset behaves differnetly though
    uint16_t t = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;
    uint16_t ptr_low = cpu_read_bus(cpu, (uint16_t)(t & 0x00FF));
    uint16_t ptr_high = cpu_read_bus(cpu, (uint16_t)(t+1) & 0x00FF);

    cpu->addr_abs = ((ptr_high << 8) | ptr_low) + cpu->y;

    if ((cpu->addr_abs & 0xFF00) != (ptr_high << 8)) {
    return 1;
    }
    else return 0;

    return 0;
} //indirect zero page w y offset
uint8_t REL(struct cpu* cpu) {
    //relative addressing only applies to branching can only jump 127 bytes
cpu->addr_rel = cpu_read_bus(cpu, cpu->pc);
    cpu->pc++;
    if (cpu->addr_rel & 0x80) {
        cpu->addr_rel |= 0xFF00;
    }
return 0;

} //relative



//sources data used for the instructions into a variable
//some may not need such data
//data resudes at addr_abs
uint8_t fetch(struct cpu* cpu) {
    //if this is not implied mode
if (!(cpu->lookup[cpu->opcode].addrmode == &IMP)) {
    cpu->fetched = cpu_read_bus(cpu, cpu->addr_abs);
}
    return cpu->fetched;



}

// OPCODES/Instructions (look at datasheet)

//bitwise
uint8_t AND(struct cpu* cpu) {
    //AND between a reg and fetched data
    fetch(cpu);
    cpu->a = cpu->a & cpu->fetched;

    //update the flags
    set_flag(cpu, Z, cpu->a == 0);
    set_flag(cpu, N, cpu->a & 0x80);

    //may require another clock
    return 1;




} //BITWISE AND
uint8_t ORA(struct cpu* cpu) {
//ORs memory value and a
    fetch(cpu);
    cpu->a = cpu->a | cpu->fetched;
    set_flag(cpu, Z, cpu->a == 0x00);
    set_flag(cpu, N, cpu->a & 0x80);
    return 1;



}//BITWISE OR
uint8_t EOR(struct cpu* cpu) {
    //XOR with memory address and a reg
    fetch(cpu);
    cpu->a = cpu->a ^ cpu->fetched;
    set_flag(cpu, Z, cpu->a == 0x00);
    set_flag(cpu, N, cpu->a & 0x80);
    return 1;

}//BITWISE XOR
uint8_t BIT(struct cpu* cpu) {
    fetch(cpu);
    uint8_t temp = cpu->a & cpu->fetched;

    set_flag(cpu, Z, temp == 0x00);

    set_flag(cpu, N, cpu->fetched & (1<<7));
    set_flag(cpu, V, cpu->fetched & (1<<6));


return 0;


} //BIT Test (this one may not work properly, not sure yet)



//BRANCHING INSTRUCTIONS
uint8_t BCS(struct cpu* cpu) {
    if (get_flag(cpu, C) == 1) {
        cpu->cycles++;
        cpu->addr_abs = cpu->pc + cpu->addr_rel;

        if ((cpu->addr_abs & 0xFF00) != (cpu->pc & 0xFF00)) {
            cpu->cycles++;
        }
        cpu->pc = cpu->addr_abs;
    }
return 0;
} //branch if carry set
uint8_t BCC(struct cpu* cpu) {
    if (get_flag(cpu, C) == 0) {
        cpu->cycles++;
        cpu->addr_abs = cpu->pc + cpu->addr_rel;

        if ((cpu->addr_abs & 0xFF00) != (cpu->pc & 0xFF00)) {
            cpu->cycles++;
        }
        cpu->pc = cpu->addr_abs;
    }
    return 0;
} //branch of carry clear
uint8_t BEQ(struct cpu* cpu) {
    if (get_flag(cpu, Z) == 1) {
        cpu->cycles++;
        cpu->addr_abs = cpu->pc + cpu->addr_rel;

        if ((cpu->addr_abs & 0xFF00) != (cpu->pc & 0xFF00)) {
            cpu->cycles++;
        }
        cpu->pc = cpu->addr_abs;
    }
    return 0;
} //branch of equal
uint8_t BMI(struct cpu* cpu) {
    if (get_flag(cpu, N) == 1) {
        cpu->cycles++;
        cpu->addr_abs = cpu->pc + cpu->addr_rel;

        if ((cpu->addr_abs & 0xFF00) != (cpu->pc & 0xFF00)) {
            cpu->cycles++;
        }
        cpu->pc = cpu->addr_abs;
    }
    return 0;
} //branch if minus
uint8_t BNE(struct cpu* cpu) {

    if (get_flag(cpu, Z) == 0) {
        cpu->cycles++;
        cpu->addr_abs = cpu->pc + cpu->addr_rel;

        if ((cpu->addr_abs & 0xFF00) != (cpu->pc & 0xFF00)) {
            cpu->cycles++;
        }
        cpu->pc = cpu->addr_abs;
    }
    return 0;
} //branch if not equal
uint8_t BPL(struct cpu* cpu) {
    if (get_flag(cpu, N) == 0) {
        cpu->cycles++;
        cpu->addr_abs = cpu->pc + cpu->addr_rel;

        if ((cpu->addr_abs & 0xFF00) != (cpu->pc & 0xFF00)) {
            cpu->cycles++;
        }
        cpu->pc = cpu->addr_abs;
    }
    return 0;
} //branch if plus
uint8_t BVC(struct cpu* cpu) {
    if (get_flag(cpu, V) == 0) {
        cpu->cycles++;
        cpu->addr_abs = cpu->pc + cpu->addr_rel;

        if ((cpu->addr_abs & 0xFF00) != (cpu->pc & 0xFF00)) {
            cpu->cycles++;
        }
        cpu->pc = cpu->addr_abs;
    }
    return 0;
} //branch if overflow clear
uint8_t BVS(struct cpu* cpu) {
    if (get_flag(cpu, V) == 1) {
        cpu->cycles++;
        cpu->addr_abs = cpu->pc + cpu->addr_rel;

        if ((cpu->addr_abs & 0xFF00) != (cpu->pc & 0xFF00)) {
            cpu->cycles++;
        }
        cpu->pc = cpu->addr_abs;
    }
    return 0;
} //branch if overflow set


//JUMPING
uint8_t JMP(struct cpu* cpu);
uint8_t BRK(struct cpu* cpu);
uint8_t JSR(struct cpu* cpu);
uint8_t RTS(struct cpu* cpu);

//INTERUPTS
uint8_t RTI(struct cpu* cpu) {
    cpu->sp++;
    cpu->status = cpu_read_bus(cpu, 0x0100+cpu->sp);
    cpu->status &= ~B;
    cpu->status &= ~U;
    cpu->sp++;
    cpu->pc = (uint16_t)cpu_read_bus(cpu, 0x0100+cpu->sp);
    cpu->sp++;
    cpu->pc |= (uint16_t)cpu_read_bus(cpu, 0x0100+cpu->sp) << 8;
    return 0;

} //return from int


//FLAG OPERATIONS
uint8_t CLC(struct cpu* cpu) {
    set_flag(cpu, C, 0);
    return 0;
} //clear carry flag
uint8_t CLD(struct cpu* cpu){return 0;} //not implemented currently
uint8_t CLI(struct cpu* cpu) {
    set_flag(cpu, I, 0);
    return 0;
} //clear interrupt dis
uint8_t CLV(struct cpu* cpu) {
    set_flag(cpu, V, 0);
    return 0;
} //clear V flag
uint8_t SEC(struct cpu* cpu) {
    set_flag(cpu, C, 1);
    return 0;

} //set carry flag
uint8_t SED(struct cpu* cpu) {
    set_flag(cpu, D, 1);
    return 0;
} //set D (not implemented)
uint8_t SEI(struct cpu* cpu){
    set_flag(cpu, I, 1);
    return 0;


} //set disable interrupt



//ADDITION AND SUBTRACTION AND ARITHMETIC
uint8_t ADC(struct cpu* cpu) {

    //addition
//add A, data in memory and the carry bit
    //setting the flags is complicated
    fetch(cpu);
    uint16_t temp = (uint16_t)cpu->a + (uint16_t)cpu->fetched + (uint16_t)get_flag(cpu, C);
    set_flag(cpu, C, temp> 255);
    set_flag(cpu, Z, (temp & 0x00FF) == 0);
    set_flag(cpu, N, temp & 0x80);
    set_flag(cpu, V, (~( ((uint16_t)cpu->a ^ (uint16_t)cpu->fetched)) & ((uint16_t)cpu->a ^ (uint16_t)temp)) & 0x0080);

    cpu->a = temp & 0x00FF;
    return 1;






}
uint8_t SBC(struct cpu* cpu) {

    fetch(cpu);
    uint16_t value = ((uint16_t)cpu->fetched) ^ 0x00FF;

    uint16_t temp = (uint16_t)cpu->a + (uint16_t)value + (uint16_t)get_flag(cpu, C);
    set_flag(cpu, C, temp & 0xFF00);
    set_flag(cpu, Z, (temp & 0x00FF) == 0);
    set_flag(cpu, N, temp & 0x0080);
    set_flag(cpu, V, (temp ^ (uint16_t)cpu->a & (temp ^ value) & 0x0080));

    cpu->a = temp & 0x00FF;
    return 1;


}
uint8_t DEC(struct cpu* cpu) {
    fetch(cpu);
    uint8_t temp = cpu->fetched;
    temp--;
    set_flag(cpu, Z, !temp);
    set_flag(cpu, N, temp&0x80);
    write_to_bus(cpu, cpu->addr_abs, temp);
    return 0;
} //decrement memory location
uint8_t DEX(struct cpu* cpu) {
    cpu->x--;
    set_flag(cpu, Z, cpu->x == 0x00);
    set_flag(cpu, N, cpu->x == 0x80);
    return 0;



}//decrement x
uint8_t DEY(struct cpu* cpu) {
    cpu->y--;
    set_flag(cpu, Z, cpu->y == 0x00);
    set_flag(cpu, N, cpu->y == 0x80);
    return 0;


}//decrement y
uint8_t INC(struct cpu* cpu) {
    fetch(cpu);
    uint8_t temp = cpu->fetched;
    temp++;
    set_flag(cpu, Z, !temp);
    set_flag(cpu, N, temp&0x80);
    write_to_bus(cpu, cpu->addr_abs, temp);
    return 0;






} //increment memory location
uint8_t INX(struct cpu* cpu) {
    cpu->x++;
    set_flag(cpu, Z, cpu->x == 0x00);
    set_flag(cpu, N, cpu->x & 0x80);


    return 0;
} //increment X
uint8_t INY(struct cpu* cpu) {
    cpu->y++;
    set_flag(cpu, Z, cpu->y == 0x00);
    set_flag(cpu, N, cpu->y & 0x80);
    return 0;
} //increment Y



//STACK 0x0100 is the base offset
uint8_t PHA(struct cpu* cpu) {
    cpu_write_bus(cpu,0x0100+cpu->sp,cpu->a );
    cpu->sp--;
    return 0;
} //push a
uint8_t PLA(struct cpu* cpu) {
    cpu->sp++;
    cpu->a = cpu_read_bus(cpu, 0x0100+cpu->sp);
    set_flag(cpu, Z, cpu->a == 0);
    set_flag(cpu, N, cpu->a & 0x80);
    return 0;



} //pull a
uint8_t PHP(struct cpu* cpu) {

    cpu_write_bus(cpu, 0x0100+cpu->sp, cpu->status | B | U);
    cpu->sp--;
    set_flag(cpu, B, 0);
    set_flag(cpu, U, 0);
    return 0;



}//push processor status
uint8_t PLP(struct cpu* cpu) {
    cpu->sp++;
    cpu->status = read_from_bus(cpu, 0x0100+cpu->sp);
    set_flag(cpu, B, 0);
    set_flag(cpu, U, 0);
    return 0;


} //pull processor status
uint8_t TXS(struct cpu* cpu) {
    cpu->sp = cpu->x;
    return 0;
} //transfer X to sp
uint8_t TSX(struct cpu* cpu) {
    cpu->x = cpu->sp;

    set_flag(cpu, Z, !(cpu->x));
    set_flag(cpu, N, cpu->x & 0x80);


    return 0;
} //transfer sp to X




//COMPARE
uint8_t CMP(struct cpu* cpu) {
    //compares A with memory value sets values no reg change
    fetch(cpu);
    set_flag(cpu, C, cpu->a >= cpu->fetched);
    set_flag(cpu, Z, cpu->a == cpu->fetched);
    set_flag(cpu, N, cpu->a < cpu->fetched);


return 1; //not sure how this is 1, doesnt say on datasheet or NESdev

} //Compare A (also not sure if this works but it should)
uint8_t CPX(struct cpu* cpu) {
    //compare x with memory value
    fetch(cpu);
    set_flag(cpu, C, cpu->x >= cpu->fetched);
    set_flag(cpu, Z, cpu->x == cpu->fetched);
    set_flag(cpu, N, cpu->x < cpu->fetched);

    return 0;
}//Compare X (also not sure if this works but it shold)
uint8_t CPY(struct cpu* cpu) {
    //comapres Y with memory val
    fetch(cpu);
    set_flag(cpu, C, cpu->y >= cpu->fetched);
    set_flag(cpu, Z, cpu->y == cpu->fetched);
    set_flag(cpu, N, cpu->y < cpu->fetched);

return 0;
} //Compare Y (also not sure if this works but it should)

//Bit shifting
uint8_t ASL(struct cpu* cpu) {
//arithmetic shift left
    //hifts bit of a memoryval of a reg left
    //may set C, Z or N flags
    fetch(cpu);
    //the actual instruction
    uint16_t temp = (uint16_t)cpu->fetched << 1;


    set_flag(cpu, C, (temp&0xFF00) > 0);
    //the temp & 0xFF00:
    // since this temp was placed into a 16 bit var (but in reality is 8)
    //checking to see if this overflow is simple, as AND 0xFf00 SHOULD clear all the high bits unless overflow occured
    set_flag(cpu, Z, temp == 0x00);
    set_flag(cpu, N, temp & 0x80);

    if (cpu->lookup[cpu->opcode].addrmode == IMP) {
        cpu->a = temp & 0x00FF; //need to clear the high bits just incase since we only want the 8 bit val
    }
    else {
        write_to_bus(cpu->bus, cpu->addr_abs, temp & 0x00FF);
    }

return 0;

} //ARTITHMETIC LEFT SHIFT
uint8_t LSR(struct cpu* cpu) {
    fetch(cpu);
    set_flag(cpu, C, (cpu->fetched&0x0001));

    uint16_t temp = (uint16_t)cpu->fetched >> 1;

    set_flag(cpu, Z, temp == 0x00);
    set_flag(cpu, N, temp & 0x80);

    if (cpu->lookup[cpu->opcode].addrmode == IMP) {
        cpu->a = temp & 0x00FF; //need to clear the high bits just incase since we only want the 8 bit val
    }
    else {
        write_to_bus(cpu->bus, cpu->addr_abs, temp & 0x00FF);
    }


} //LOGICAL SHIFT RIGHT
uint8_t ROL(struct cpu* cpu) {
    //C bit treated as 8th bit and -1th bit, circles back
    //get the carry bit and place it at 0th bit
    //get 8th bit place it in carry bit

    fetch(cpu);
    //place carry bit as 0th bit while shifting
    uint16_t temp = ((uint16_t)cpu->fetched << 1)| (get_flag(cpu, C));

    set_flag(cpu, Z, (temp == 0x00FF)==0x00);
    set_flag(cpu, N, temp & 0x80);
    set_flag(cpu, C, (temp & 0xFF00) > 0);

    if (cpu->lookup[cpu->opcode].addrmode == IMP) {
        cpu->a = temp & 0x00FF; //need to clear the high bits just incase since we only want the 8 bit val
    }
    else {
        write_to_bus(cpu->bus, cpu->addr_abs, temp & 0x00FF);
    }
    return 0;


} //ROTATE LEFT
uint8_t ROR(struct cpu* cpu) {
    //C bit treated as -1th bit and 9th bit, circles back
    //get the carry bit and place it at 8th bit
    //get 0th bit place it in carry bit
    fetch(cpu);
    uint16_t temp = ((uint16_t)cpu->fetched >> 1)| (get_flag(cpu, C) << 7); //i hope this is the same as these reversed i think it is
    set_flag(cpu, Z, (temp == 0x00FF) == 0x0);
    set_flag(cpu, N, temp & 0x80);
    set_flag(cpu, C, (temp & 0xFF00) > 0);

    if (cpu->lookup[cpu->opcode].addrmode == IMP) {
        cpu->a = temp & 0x00FF;

    }
    else {
        write_to_bus(cpu->bus, cpu->addr_abs, temp & 0x00FF);
    }
return 0;







}//ROTATE RIGHT

//ACCESS
uint8_t LDA(struct cpu* cpu);
uint8_t STA(struct cpu* cpu);
uint8_t LDX(struct cpu* cpu);
uint8_t STX(struct cpu* cpu);
uint8_t LDY(struct cpu* cpu);
uint8_t STY(struct cpu* cpu);

//TRANSFER
uint8_t TAX(struct cpu* cpu);
uint8_t TAY(struct cpu* cpu);
uint8_t TYA(struct cpu* cpu);
uint8_t TXA(struct cpu* cpu);







//NOP and Catch All
uint8_t NOP(struct cpu* cpu) { //this apparently contaisn otehr nops?
    return 0;

} //may require more
uint8_t XXX(struct cpu* cpu) {
    NOP(cpu);
}//catch all in case stuff breaks same as NOP

