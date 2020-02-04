#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "6502.hpp"

// Forward declare Bus
class Bus;

/**
 * @brief 6502 CPU class
 */
class CPU6502
{
public:
    CPU6502();
    ~CPU6502();

    void connectBus(Bus *inBus)
    {
        bus = inBus;
    }

    /**
     * @brief Enums for flag bit register
     */
    enum STATUS_FLAG
    {
        C = 0b00000001, // Carry
        Z = 0b00000010, // Result Zero
        I = 0b00000100, // IRQ Disable
        D = 0b00001000, // Decimal mode ** not used **
        B = 0b00010000, // BRK command
        //  0b00100000, // Unused
        O = 0b01000000, // Overflow
        N = 0b10000000, // Negative
    };

    //----------------------
    // Addressing modes
    //----------------------

    /** Doxy defined in .cpp */ 
    uint8_t IMP();
    uint8_t IMM();
    uint8_t ZP0();
    uint8_t ZPX();
    uint8_t ZPY();
    uint8_t REL();
    uint8_t ABS();
    uint8_t ABX();
    uint8_t ABY();
    uint8_t IND();
    uint8_t IZX();
    uint8_t IZY();

    //----------------------
    // Opcodes
    //----------------------
    
    /** Doxy defined in .cpp */ 
    uint8_t ADC();
    uint8_t AND();
    uint8_t ASL();
    uint8_t BCC();
    uint8_t BCS();
    uint8_t BEQ();
    uint8_t BIT();
    uint8_t BMI();
    uint8_t BNE();
    uint8_t BPL();
    uint8_t BRK();
    uint8_t BVC();
    uint8_t BVS();
    uint8_t CLC();
    uint8_t CLD();
    uint8_t CLI();
    uint8_t CLV();
    uint8_t CMP();
    uint8_t CPX();
    uint8_t CPY();
    uint8_t DEC();
    uint8_t DEX();
    uint8_t DEY();
    uint8_t EOR();
    uint8_t INC();
    uint8_t INX();
    uint8_t INY();
    uint8_t JMP();
    uint8_t JSR();
    uint8_t LDA();
    uint8_t LDX();
    uint8_t LDY();
    uint8_t LSR();
    uint8_t NOP();
    uint8_t ORA();
    uint8_t PHA();
    uint8_t PHP();
    uint8_t PLA();
    uint8_t PLP();
    uint8_t ROL();
    uint8_t ROR();
    uint8_t RTI();
    uint8_t RTS();
    uint8_t SBC();
    uint8_t SEC();
    uint8_t SED();
    uint8_t SEI();
    uint8_t STA();
    uint8_t STX();
    uint8_t STY();
    uint8_t TAX();
    uint8_t TAY();
    uint8_t TSX();
    uint8_t TXA();
    uint8_t TXS();
    uint8_t TYA();

    /**
     * @brief Catch illegal opcode
     * @return uint8_t 
     */
    uint8_t ILL();

    //----------------------
    // * Signals *
    //
    //
    // Asynchronous, can occur at any time
    // current instruction however will finish executing before these take effect
    //----------------------
    void clock();
    void reset();
    void irq();

    /**
     * @brief Non-masking interrupt
     * @note can not be disabled
     */
    void nmi();

    //----------------------
    // Data Section
    //----------------------
    uint8_t fetch();
    uint8_t fetched_data = 0x00;

    uint16_t target_addr = 0x0000;
    uint16_t rel_addr = 0x00;
    uint8_t current_opcode = 0x00;
    uint8_t cycles = 0;
    
    //----------------------
    // Register Definitions
    //----------------------
    uint8_t a = 0x00;       // Accumulator A
    uint8_t y = 0x00;       // Index reg Y
    uint8_t x = 0x00;       // Index reg X
    uint16_t pc = 0x0000;   // Program Counter
    uint8_t s = 0x00;       // Stack pointer
    uint8_t p = 0x00;       // Proc status reg (STATUS_FLAG enums)

private:
    Bus* bus = nullptr;

    using c = CPU6502;

    /**
     * @brief Instruction struct
     */
    struct instruction
    {
        // Neumonic, for tracing
        std::string name;

        uint8_t(c::*op)(void) = nullptr;
        uint8_t(c::*addr_mode)(void) = nullptr;

        // Number of cycles
        uint8_t cycles = 0;
    };

    const instruction illegal_inst = {"XXX", &c::ILL, &c::IMP, 2};

    /**
     * @brief Instruction lookup table
     * @note derived from http://archive.6502.org/datasheets/rockwell_r650x_r651x.pdf with aid from
     *       https://github.com/OneLoneCoder/olcNES/blob/master/Part%232%20-%20CPU/olc6502.cpp
     */
    std::vector<instruction> instructions = 
    {
        { "BRK", &c::BRK, &c::IMM, 7 },{ "ORA", &c::ORA, &c::IZX, 6 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 3 },{ "ORA", &c::ORA, &c::ZP0, 3 },{ "ASL", &c::ASL, &c::ZP0, 5 },illegal_inst                  ,{ "PHP", &c::PHP, &c::IMP, 3 },{ "ORA", &c::ORA, &c::IMM, 2 },{ "ASL", &c::ASL, &c::IMP, 2 },illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "ORA", &c::ORA, &c::ABS, 4 },{ "ASL", &c::ASL, &c::ABS, 6 },illegal_inst                  ,
		{ "BPL", &c::BPL, &c::REL, 2 },{ "ORA", &c::ORA, &c::IZY, 5 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "ORA", &c::ORA, &c::ZPX, 4 },{ "ASL", &c::ASL, &c::ZPX, 6 },illegal_inst                  ,{ "CLC", &c::CLC, &c::IMP, 2 },{ "ORA", &c::ORA, &c::ABY, 4 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "ORA", &c::ORA, &c::ABX, 4 },{ "ASL", &c::ASL, &c::ABX, 7 },illegal_inst                  ,
		{ "JSR", &c::JSR, &c::ABS, 6 },{ "AND", &c::AND, &c::IZX, 6 },illegal_inst                  ,illegal_inst                  ,{ "BIT", &c::BIT, &c::ZP0, 3 },{ "AND", &c::AND, &c::ZP0, 3 },{ "ROL", &c::ROL, &c::ZP0, 5 },illegal_inst                  ,{ "PLP", &c::PLP, &c::IMP, 4 },{ "AND", &c::AND, &c::IMM, 2 },{ "ROL", &c::ROL, &c::IMP, 2 },illegal_inst                  ,{ "BIT", &c::BIT, &c::ABS, 4 },{ "AND", &c::AND, &c::ABS, 4 },{ "ROL", &c::ROL, &c::ABS, 6 },illegal_inst                  ,
		{ "BMI", &c::BMI, &c::REL, 2 },{ "AND", &c::AND, &c::IZY, 5 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "AND", &c::AND, &c::ZPX, 4 },{ "ROL", &c::ROL, &c::ZPX, 6 },illegal_inst                  ,{ "SEC", &c::SEC, &c::IMP, 2 },{ "AND", &c::AND, &c::ABY, 4 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "AND", &c::AND, &c::ABX, 4 },{ "ROL", &c::ROL, &c::ABX, 7 },illegal_inst                  ,
		{ "RTI", &c::RTI, &c::IMP, 6 },{ "EOR", &c::EOR, &c::IZX, 6 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 3 },{ "EOR", &c::EOR, &c::ZP0, 3 },{ "LSR", &c::LSR, &c::ZP0, 5 },illegal_inst                  ,{ "PHA", &c::PHA, &c::IMP, 3 },{ "EOR", &c::EOR, &c::IMM, 2 },{ "LSR", &c::LSR, &c::IMP, 2 },illegal_inst                  ,{ "JMP", &c::JMP, &c::ABS, 3 },{ "EOR", &c::EOR, &c::ABS, 4 },{ "LSR", &c::LSR, &c::ABS, 6 },illegal_inst                  ,
		{ "BVC", &c::BVC, &c::REL, 2 },{ "EOR", &c::EOR, &c::IZY, 5 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "EOR", &c::EOR, &c::ZPX, 4 },{ "LSR", &c::LSR, &c::ZPX, 6 },illegal_inst                  ,{ "CLI", &c::CLI, &c::IMP, 2 },{ "EOR", &c::EOR, &c::ABY, 4 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "EOR", &c::EOR, &c::ABX, 4 },{ "LSR", &c::LSR, &c::ABX, 7 },illegal_inst                  ,
		{ "RTS", &c::RTS, &c::IMP, 6 },{ "ADC", &c::ADC, &c::IZX, 6 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 3 },{ "ADC", &c::ADC, &c::ZP0, 3 },{ "ROR", &c::ROR, &c::ZP0, 5 },illegal_inst                  ,{ "PLA", &c::PLA, &c::IMP, 4 },{ "ADC", &c::ADC, &c::IMM, 2 },{ "ROR", &c::ROR, &c::IMP, 2 },illegal_inst                  ,{ "JMP", &c::JMP, &c::IND, 5 },{ "ADC", &c::ADC, &c::ABS, 4 },{ "ROR", &c::ROR, &c::ABS, 6 },illegal_inst                  ,
		{ "BVS", &c::BVS, &c::REL, 2 },{ "ADC", &c::ADC, &c::IZY, 5 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "ADC", &c::ADC, &c::ZPX, 4 },{ "ROR", &c::ROR, &c::ZPX, 6 },illegal_inst                  ,{ "SEI", &c::SEI, &c::IMP, 2 },{ "ADC", &c::ADC, &c::ABY, 4 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "ADC", &c::ADC, &c::ABX, 4 },{ "ROR", &c::ROR, &c::ABX, 7 },illegal_inst                  ,
		illegal_inst                  ,{ "STA", &c::STA, &c::IZX, 6 },illegal_inst                  ,illegal_inst                  ,{ "STY", &c::STY, &c::ZP0, 3 },{ "STA", &c::STA, &c::ZP0, 3 },{ "STX", &c::STX, &c::ZP0, 3 },illegal_inst                  ,{ "DEY", &c::DEY, &c::IMP, 2 },illegal_inst                  ,{ "TXA", &c::TXA, &c::IMP, 2 },illegal_inst                  ,{ "STY", &c::STY, &c::ABS, 4 },{ "STA", &c::STA, &c::ABS, 4 },{ "STX", &c::STX, &c::ABS, 4 },illegal_inst                  ,
		{ "BCC", &c::BCC, &c::REL, 2 },{ "STA", &c::STA, &c::IZY, 6 },illegal_inst                  ,illegal_inst                  ,{ "STY", &c::STY, &c::ZPX, 4 },{ "STA", &c::STA, &c::ZPX, 4 },{ "STX", &c::STX, &c::ZPY, 4 },illegal_inst                  ,{ "TYA", &c::TYA, &c::IMP, 2 },{ "STA", &c::STA, &c::ABY, 5 },{ "TXS", &c::TXS, &c::IMP, 2 },illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 5 },{ "STA", &c::STA, &c::ABX, 5 },illegal_inst                  ,illegal_inst                  ,
		{ "LDY", &c::LDY, &c::IMM, 2 },{ "LDA", &c::LDA, &c::IZX, 6 },{ "LDX", &c::LDX, &c::IMM, 2 },illegal_inst                  ,{ "LDY", &c::LDY, &c::ZP0, 3 },{ "LDA", &c::LDA, &c::ZP0, 3 },{ "LDX", &c::LDX, &c::ZP0, 3 },illegal_inst                  ,{ "TAY", &c::TAY, &c::IMP, 2 },{ "LDA", &c::LDA, &c::IMM, 2 },{ "TAX", &c::TAX, &c::IMP, 2 },illegal_inst                  ,{ "LDY", &c::LDY, &c::ABS, 4 },{ "LDA", &c::LDA, &c::ABS, 4 },{ "LDX", &c::LDX, &c::ABS, 4 },illegal_inst                  ,
		{ "BCS", &c::BCS, &c::REL, 2 },{ "LDA", &c::LDA, &c::IZY, 5 },illegal_inst                  ,illegal_inst                  ,{ "LDY", &c::LDY, &c::ZPX, 4 },{ "LDA", &c::LDA, &c::ZPX, 4 },{ "LDX", &c::LDX, &c::ZPY, 4 },illegal_inst                  ,{ "CLV", &c::CLV, &c::IMP, 2 },{ "LDA", &c::LDA, &c::ABY, 4 },{ "TSX", &c::TSX, &c::IMP, 2 },illegal_inst                  ,{ "LDY", &c::LDY, &c::ABX, 4 },{ "LDA", &c::LDA, &c::ABX, 4 },{ "LDX", &c::LDX, &c::ABY, 4 },illegal_inst                  ,
		{ "CPY", &c::CPY, &c::IMM, 2 },{ "CMP", &c::CMP, &c::IZX, 6 },illegal_inst                  ,illegal_inst                  ,{ "CPY", &c::CPY, &c::ZP0, 3 },{ "CMP", &c::CMP, &c::ZP0, 3 },{ "DEC", &c::DEC, &c::ZP0, 5 },illegal_inst                  ,{ "INY", &c::INY, &c::IMP, 2 },{ "CMP", &c::CMP, &c::IMM, 2 },{ "DEX", &c::DEX, &c::IMP, 2 },illegal_inst                  ,{ "CPY", &c::CPY, &c::ABS, 4 },{ "CMP", &c::CMP, &c::ABS, 4 },{ "DEC", &c::DEC, &c::ABS, 6 },illegal_inst                  ,
		{ "BNE", &c::BNE, &c::REL, 2 },{ "CMP", &c::CMP, &c::IZY, 5 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "CMP", &c::CMP, &c::ZPX, 4 },{ "DEC", &c::DEC, &c::ZPX, 6 },illegal_inst                  ,{ "CLD", &c::CLD, &c::IMP, 2 },{ "CMP", &c::CMP, &c::ABY, 4 },{ "NOP", &c::NOP, &c::IMP, 2 },illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "CMP", &c::CMP, &c::ABX, 4 },{ "DEC", &c::DEC, &c::ABX, 7 },illegal_inst                  ,
		{ "CPX", &c::CPX, &c::IMM, 2 },{ "SBC", &c::SBC, &c::IZX, 6 },illegal_inst                  ,illegal_inst                  ,{ "CPX", &c::CPX, &c::ZP0, 3 },{ "SBC", &c::SBC, &c::ZP0, 3 },{ "INC", &c::INC, &c::ZP0, 5 },illegal_inst                  ,{ "INX", &c::INX, &c::IMP, 2 },{ "SBC", &c::SBC, &c::IMM, 2 },{ "NOP", &c::NOP, &c::IMP, 2 },illegal_inst                  ,{ "CPX", &c::CPX, &c::ABS, 4 },{ "SBC", &c::SBC, &c::ABS, 4 },{ "INC", &c::INC, &c::ABS, 6 },illegal_inst                  ,
		{ "BEQ", &c::BEQ, &c::REL, 2 },{ "SBC", &c::SBC, &c::IZY, 5 },illegal_inst                  ,illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "SBC", &c::SBC, &c::ZPX, 4 },{ "INC", &c::INC, &c::ZPX, 6 },illegal_inst                  ,{ "SED", &c::SED, &c::IMP, 2 },{ "SBC", &c::SBC, &c::ABY, 4 },{ "NOP", &c::NOP, &c::IMP, 2 },illegal_inst                  ,{ "???", &c::NOP, &c::IMP, 4 },{ "SBC", &c::SBC, &c::ABX, 4 },{ "INC", &c::INC, &c::ABX, 7 },illegal_inst                  ,
    };

    /**
     * @brief 6502 read function
     * 
     * @param addr 16 bit address
     * @return uint8_t data at addr
     * @note calls Bus::read
     */
    uint8_t read(uint16_t addr);

    /**
     * @brief 6502 write function
     * 
     * @param addr 16 bit address
     * @param data data to write
     * @note calls Bus::write
     */
    void write(uint16_t addr, uint8_t data);

    /**
     * @brief Get the STATUS_FLAG in the proc status register
     * 
     * @param flag flag to get
     * @return uint8_t HIGH/LOW
     */
    uint8_t getFlag(STATUS_FLAG flag);

    /**
     * @brief Set the STATUS_FLAG in the proc status register
     * 
     * @param flag flag to set
     * @param h HIGH/LOW
     */
    void setFlag(STATUS_FLAG flag, bool h);


};