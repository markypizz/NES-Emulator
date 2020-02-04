#include <stdio.h>
#include <signal.h>
#include "6502.hpp"
#include "Bus.hpp"
CPU6502::CPU6502()
{
}

CPU6502::~CPU6502()
{

}

/**
 * @brief 6502 read function
 * 
 * @param addr 16 bit address
 * @return uint8_t data at addr
 * @note calls Bus::read
 */
uint8_t CPU6502::read(uint16_t addr)
{
    return bus->read(addr);
}

/**
 * @brief 6502 write function
 * 
 * @param addr 16 bit address
 * @param data data to write
 * @note calls Bus::write
 */
void CPU6502::write(uint16_t addr, uint8_t data)
{
    bus->write(addr, data);
}

/**
 * @brief Get the STATUS_FLAG in the proc status register
 * 
 * @param flag flag to get
 * @return uint8_t HIGH/LOW
 */
uint8_t CPU6502::getFlag(STATUS_FLAG flag)
{
    return ((p & flag) > 0);
}

/**
 * @brief Set the STATUS_FLAG in the proc status register
 * 
 * @param flag flag to set
 * @param h HIGH/LOW
 */
void CPU6502::setFlag(STATUS_FLAG flag, bool h)
{
    if (h)
    {
        p |= flag;
    }
    else
    {
        p &= ~flag;
    }
    
}

/**
 * @brief Clock cycle function
 */
void CPU6502::clock()
{
    // We will run an instruction fully on each loop, no pipelining or anything fancy
    if (cycles == 0)
    {
        // Get next instruction opcode
        current_opcode = read(pc);
        pc++;

        // Set our local cycles var
        cycles = instructions[current_opcode].cycles;

        // Call addressing mode function, and operation function associated with this operation
        // Increment extra cycles if the instruction claims to need it
        cycles += (this->*instructions[current_opcode].addr_mode)();
        cycles += (this->*instructions[current_opcode].op)();
    }

    // Decrement
    cycles--;
}

//----------------------
// Addressing modes
//----------------------

/**
 * @brief Implicit, do something, fill in a, though not required
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::IMP()
{
    target_addr = a;
    return 0;
}

/**
 * @brief Immediate mode addressing, supplied as next byte
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::IMM()
{
    // Fill in address, then post increment
    target_addr = pc++;
    return 0;
}

/**
 * @brief Zero Page, only 8 bit address operand in addition to $00
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::ZP0()
{
    target_addr = read(pc);
    pc++;
    target_addr &= 0x00FF;
    return 0;
}

/**
 * @brief Zero Page X, 8 bit address + contents of X
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::ZPX()
{
    target_addr = read(pc + x);
    pc++;
    target_addr &= 0x00FF;
    return 0;
}

/**
 * @brief Zero Page Y, 8 bit address + contents of Y
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::ZPY()
{
    target_addr = read(pc + y);
    pc++;
    target_addr &= 0x00FF;
    return 0;
}

/**
 * @brief Absolute address
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::ABS()
{
    // Use uint16_t's to make the shifting easier
    uint16_t low_byte = read(pc);
    pc++;
    uint16_t high_byte = read(pc);
    pc++;

    // Piece 'em together
    target_addr = (high_byte << 8 | low_byte);

    return 0;
}

/**
 * @brief Absolute address + X
 * 
 * @return uint8_t 0 or 1 if overflowed to next page
 */
uint8_t CPU6502::ABX()
{
    // Use uint16_t's to make the shifting easier
    uint16_t low_byte = read(pc);
    pc++;
    uint16_t high_byte = read(pc);
    pc++;

    // Piece 'em together
    target_addr = (high_byte << 8 | low_byte) + x;

    if ((target_addr & 0xFF00 >> 8) != high_byte)
    {
        // We've overflowed, may need an extra cycle
        return 1;
    }
    
    return 0;
}

/**
 * @brief Absolute address + Y
 * 
 * @return uint8_t 0 or 1 if overflowed to next page
 */
uint8_t CPU6502::ABY()
{
    // Use uint16_t's to make the shifting easier
    uint16_t low_byte = read(pc);
    pc++;
    uint16_t high_byte = read(pc);
    pc++;

    // Piece 'em together
    target_addr = (high_byte << 8 | low_byte) + y;

    if ((target_addr & 0xFF00 >> 8) != high_byte)
    {
        // We've overflowed, may need an extra cycle
        return 1;
    }
    
    return 0;
}

/**
 * @brief Indirect addressing (address of address)
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::IND()
{
    uint16_t loc_low = read(pc);
    pc++;
    uint16_t loc_high = read(pc);
    pc++;

    uint16_t read_loc = (loc_high << 8) | loc_low;

    // Now grab the actual address
    uint16_t low_byte = read(read_loc);
    uint16_t high_byte = read(read_loc + 1);

    // Hardware bug emulation (lol)
    if (loc_low == 0x00FF)
    {
        target_addr = (read(read_loc & 0xFF00) << 8) | read(read_loc + 0);
    }
    else
    {
        target_addr = read((high_byte << 8) | low_byte);
    }

    return 0;
}

/**
 * @brief Indirect addressing zero page (address of address) + X
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::IZX()
{
    uint16_t loc = read(pc);
    pc++;

    uint16_t low_byte = read((uint16_t)(loc + (uint16_t)x) & 0x00FF);
    uint16_t high_byte = read((uint16_t)(loc + (uint16_t)x + 1) & 0x00FF);

    target_addr = (high_byte << 8) | low_byte;

    return 0;
}

/**
 * @brief Indirect addressing zero page (address of address) + Y (post)
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::IZY()
{
    uint16_t loc = read(pc);
    pc++;

    uint16_t low_byte = read(loc & 0x00FF);
    uint16_t high_byte = read((loc + 1) & 0x00FF);

    target_addr = (high_byte << 8) | low_byte;
    target_addr += y;

    if ((target_addr & 0xFF00) != (high_byte << 8))
    {
        // Crossed page boundary
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief Relative addressing
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::REL()
{
    rel_addr = read(pc);
    pc++;

    // Signed, negative relative jump
    if (rel_addr & 0x80)
    {
        rel_addr |= 0xFF00;
    }

    return 0;
}

//----------------------
// Opcodes (and helper functions!)
//----------------------

/**
 * @brief Fetch data from fetched_data if addressing mode is !IMP
 * 
 * @return uint8_t fetched_data
 * @note also sets fetched_data
 */
uint8_t CPU6502::fetch()
{
    if (!(instructions[current_opcode].addr_mode == &CPU6502::IMP))
    {
        fetched_data = read(target_addr);
    }

    return fetched_data;
}

uint8_t CPU6502::ADC()
{
    return 0;
}

/**
 * @brief AND opcode
 * 
 * @return uint8_t extra cycle
 */
uint8_t CPU6502::AND()
{
    fetch();
    a = fetched_data & a;

    setFlag(STATUS_FLAG::Z, (a == 0));
    setFlag(STATUS_FLAG::N, (a & 0x80));
    
    return 0;
}

/**
 * @brief ASL opcode (arithmetic shift left)
 * 
 * @return uint8_t extra cycle
 */
uint8_t CPU6502::ASL()
{
    fetch();
    uint8_t shifted = (uint16_t)a << 1;

    setFlag(STATUS_FLAG::C, (shifted & 0xFF00));
    setFlag(STATUS_FLAG::Z, ((shifted & 0xFF00) == 0));
    setFlag(STATUS_FLAG::N, (shifted & 0x80));

    if (instructions[current_opcode].addr_mode == &CPU6502::IMP)
    {
        // Use accumulator A
        a = shifted & 0x00FF;
    }
    else
    {
        // Write to address
        write(target_addr, shifted & 0x00FF);
    }

    return 0;
}

/**
 * @brief BCC (branch carry clear)
 * 
 * @return uint8_t extra cycle
 */
uint8_t CPU6502::BCC()
{
    if (getFlag(STATUS_FLAG::C) == 0)
    {
        cycles++;
        target_addr = pc + rel_addr;

        if (target_addr & 0xFF00 != pc & 0xFF00)
        {
            // Page turn
            cycles++;
        }

        pc = target_addr;
    }

    return 0;
}
uint8_t CPU6502::BCS()
{
    return 0;
}
uint8_t CPU6502::BEQ()
{
    return 0;
}
uint8_t CPU6502::BIT()
{
    return 0;
}
uint8_t CPU6502::BMI()
{
    return 0;
}
uint8_t CPU6502::BNE()
{
    return 0;
}
uint8_t CPU6502::BPL()
{
    return 0;
}
uint8_t CPU6502::BRK()
{
    return 0;
}
uint8_t CPU6502::BVC()
{
    return 0;
}
uint8_t CPU6502::BVS()
{
    return 0;
}
uint8_t CPU6502::CLC()
{
    return 0;
}
uint8_t CPU6502::CLD()
{
    return 0;
}
uint8_t CPU6502::CLI()
{
    return 0;
}
uint8_t CPU6502::CLV()
{
    return 0;
}
uint8_t CPU6502::CMP()
{
    return 0;
}
uint8_t CPU6502::CPX()
{
    return 0;
}
uint8_t CPU6502::CPY()
{
    return 0;
}
uint8_t CPU6502::DEC()
{
    return 0;
}
uint8_t CPU6502::DEX()
{
    return 0;
}
uint8_t CPU6502::DEY()
{
    return 0;
}
uint8_t CPU6502::EOR()
{
    return 0;
}
uint8_t CPU6502::INC()
{
    return 0;
}
uint8_t CPU6502::INX()
{
    return 0;
}
uint8_t CPU6502::INY()
{
    return 0;
}
uint8_t CPU6502::JMP()
{
    return 0;
}
uint8_t CPU6502::JSR()
{
    return 0;
}
uint8_t CPU6502::LDA()
{
    return 0;
}
uint8_t CPU6502::LDX()
{
    return 0;
}
uint8_t CPU6502::LDY()
{
    return 0;
}
uint8_t CPU6502::LSR()
{
    return 0;
}
uint8_t CPU6502::NOP()
{
    return 0;
}
uint8_t CPU6502::ORA()
{
    return 0;
}
uint8_t CPU6502::PHA()
{
    return 0;
}
uint8_t CPU6502::PHP()
{
    return 0;
}
uint8_t CPU6502::PLA()
{
    return 0;
}
uint8_t CPU6502::PLP()
{
    return 0;
}
uint8_t CPU6502::ROL()
{
    return 0;
}
uint8_t CPU6502::ROR()
{
    return 0;
}
uint8_t CPU6502::RTI()
{
    return 0;
}
uint8_t CPU6502::RTS()
{
    return 0;
}
uint8_t CPU6502::SBC()
{
    return 0;
}
uint8_t CPU6502::SEC()
{
    return 0;
}
uint8_t CPU6502::SED()
{
    return 0;
}
uint8_t CPU6502::SEI()
{
    return 0;
}
uint8_t CPU6502::STA()
{
    return 0;
}
uint8_t CPU6502::STX()
{
    return 0;
}
uint8_t CPU6502::STY()
{
    return 0;
}
uint8_t CPU6502::TAX()
{
    return 0;
}
uint8_t CPU6502::TAY()
{
    return 0;
}
uint8_t CPU6502::TSX()
{
    return 0;
}
uint8_t CPU6502::TXA()
{
    return 0;
}
uint8_t CPU6502::TXS()
{
    return 0;
}
uint8_t CPU6502::TYA()
{
    return 0;
}

uint8_t CPU6502::ILL()
{
    printf("ILL instruction");
    raise(SIGSEGV);
    return 0;
}