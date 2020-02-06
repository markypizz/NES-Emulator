#include <stdio.h>
#include <signal.h>
#include "6502.hpp"
#include "Bus.hpp"
#include <map>

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
        cycles += ((this->*instructions[current_opcode].addr_mode)() &
                   (this->*instructions[current_opcode].op)());
    }

    // Decrement
    cycles--;
}

/**
 * @brief Reset chip to known state
 */
void CPU6502::reset()
{
    // Reset vars
    rel_addr = 0;
    fetched_data = 0;

    // Reset registers
    x = 0;
    y = 0;
    a = 0;

    s = STACK_DEFAULT;
    p = P_DEFAULT;
    target_addr = PC_DEFAULT;

    // Grab address from PC_DEFAULT reg
    uint16_t high_byte = read(target_addr);
    uint16_t low_byte = read(target_addr + 1);

    pc = (high_byte << 8 | low_byte);

    // Time to clear these cycles
    cycles = 8;
}

/**
 * @brief Interrupt request
 */
void CPU6502::irq()
{
    if (getFlag(STATUS_FLAG::I) == 0)
    {
        nmi();

        // Though this one takes 7 cycles
        cycles = 7;
    }
}


/**
 * @brief Non-maskable interrupt request
 */
void CPU6502::nmi()
{
    // Start writing data to stack, first: need PC
    write(STACK_BASE + s, ((pc >> 8) & 0x00FF));
    s--;

    write(STACK_BASE + s, pc & 0x00FF);
    s--;

    // Set flags accordingly and then write status register
    setFlag(STATUS_FLAG::U, 1);
    setFlag(STATUS_FLAG::I, 1);
    setFlag(STATUS_FLAG::B, 0);
    write(STACK_BASE + s, p);
    s--;

    target_addr = INT_TARGET_ADDR;
    uint16_t high_byte = read(target_addr);
    uint16_t low_byte = read(target_addr + 1);

    pc = (high_byte << 8 | low_byte);

    // Runs for 8 cycles
    cycles = 8;
}

/**
 * @brief Dissasemble function
 * @note Code from https://github.com/OneLoneCoder/olcNES/blob/master/Part%232%20-%20CPU/CPU6502.cpp
 */
std::map<uint16_t, std::string> CPU6502::disassemble(uint16_t nStart, uint16_t nStop)
{
	uint32_t addr = nStart;
	uint8_t value = 0x00, lo = 0x00, hi = 0x00;
	std::map<uint16_t, std::string> mapLines;
	uint16_t line_addr = 0;

	// A convenient utility to convert variables into
	// hex strings because "modern C++"'s method with 
	// streams is atrocious
	auto hex = [](uint32_t n, uint8_t d)
	{
		std::string s(d, '0');
		for (int i = d - 1; i >= 0; i--, n >>= 4)
			s[i] = "0123456789ABCDEF"[n & 0xF];
		return s;
	};

	// Starting at the specified address we read an instruction
	// byte, which in turn yields information from the lookup table
	// as to how many additional bytes we need to read and what the
	// addressing mode is. I need this info to assemble human readable
	// syntax, which is different depending upon the addressing mode

	// As the instruction is decoded, a std::string is assembled
	// with the readable output
	while (addr <= (uint32_t)nStop)
	{
		line_addr = addr;

		// Prefix line with instruction address
		std::string sInst = "$" + hex(addr, 4) + ": ";

		// Read instruction, and get its readable name
		uint8_t opcode = bus->read(addr, true); addr++;
		sInst += instructions[opcode].name + " ";

		// Get oprands from desired locations, and form the
		// instruction based upon its addressing mode. These
		// routines mimmick the actual fetch routine of the
		// 6502 in order to get accurate data as part of the
		// instruction
		if (instructions[opcode].addr_mode == &CPU6502::IMP)
		{
			sInst += " {IMP}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::IMM)
		{
			value = bus->read(addr, true); addr++;
			sInst += "#$" + hex(value, 2) + " {IMM}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::ZP0)
		{
			lo = bus->read(addr, true); addr++;
			hi = 0x00;												
			sInst += "$" + hex(lo, 2) + " {ZP0}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::ZPX)
		{
			lo = bus->read(addr, true); addr++;
			hi = 0x00;														
			sInst += "$" + hex(lo, 2) + ", X {ZPX}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::ZPY)
		{
			lo = bus->read(addr, true); addr++;
			hi = 0x00;														
			sInst += "$" + hex(lo, 2) + ", Y {ZPY}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::IZX)
		{
			lo = bus->read(addr, true); addr++;
			hi = 0x00;								
			sInst += "($" + hex(lo, 2) + ", X) {IZX}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::IZY)
		{
			lo = bus->read(addr, true); addr++;
			hi = 0x00;								
			sInst += "($" + hex(lo, 2) + "), Y {IZY}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::ABS)
		{
			lo = bus->read(addr, true); addr++;
			hi = bus->read(addr, true); addr++;
			sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + " {ABS}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::ABX)
		{
			lo = bus->read(addr, true); addr++;
			hi = bus->read(addr, true); addr++;
			sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + ", X {ABX}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::ABY)
		{
			lo = bus->read(addr, true); addr++;
			hi = bus->read(addr, true); addr++;
			sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + ", Y {ABY}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::IND)
		{
			lo = bus->read(addr, true); addr++;
			hi = bus->read(addr, true); addr++;
			sInst += "($" + hex((uint16_t)(hi << 8) | lo, 4) + ") {IND}";
		}
		else if (instructions[opcode].addr_mode == &CPU6502::REL)
		{
			value = bus->read(addr, true); addr++;
			sInst += "$" + hex(value, 2) + " [$" + hex(addr + value, 4) + "] {REL}";
		}

		// Add the formed string to a std::map, using the instruction's
		// address as the key. This makes it convenient to look for later
		// as the instructions are variable in length, so a straight up
		// incremental index is not sufficient.
		mapLines[line_addr] = sInst;
	}

	return mapLines;
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

/**
 * @brief Add with carry
 * 
 * @return uint8_t extra cycle
 */
uint8_t CPU6502::ADC()
{
    fetch();

    uint16_t addition = 
            (uint16_t)a + 
            (uint16_t)fetched_data + 
            (uint16_t)getFlag(STATUS_FLAG::C);

    // Set flags as needed
    setFlag(STATUS_FLAG::C, addition > 0x00FF);
    setFlag(STATUS_FLAG::Z, ((addition & 0x00FF) == 0));
    setFlag(STATUS_FLAG::N, addition & 0x0080);

    // Overflow: if (A,M negative while result positive) || (A,M positive while result negative)
    setFlag(STATUS_FLAG::O, 
            ((is8BitNeg(a) && is8BitNeg(fetched_data) && !is8BitNeg(addition)) ||
            (!is8BitNeg(a) && !is8BitNeg(fetched_data) && is8BitNeg(addition))));

    a = addition & 0x00FF;
    return 1;
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
    
    // Could potentially cross page boundary
    return 1;
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

/**
 * @brief BCS (branch carry set)
 * 
 * @return uint8_t extra cycle
 */
uint8_t CPU6502::BCS()
{
    if (getFlag(STATUS_FLAG::C) != 0)
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

/**
 * @brief BEQ (branch if equal)
 * 
 * @return uint8_t extra cycle
 */
uint8_t CPU6502::BEQ()
{
    // EQ is true if Z is 0
    if (getFlag(STATUS_FLAG::Z) == 0)
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

/**
 * @brief BIT 
 * 
 * @return uint8_t extra cycle
 */
uint8_t CPU6502::BIT()
{
    return 0;
}

/**
 * @brief BMI (branch on minus)
 * 
 * @return uint8_t 
 */
uint8_t CPU6502::BMI()
{
    // BMI is true if N (negative) flag is 1
    if (getFlag(STATUS_FLAG::N) == 1)
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

/**
 * @brief BNE (branch not equal)
 * 
 * @return uint8_t extra cycle
 */
uint8_t CPU6502::BNE()
{
    // BNE is true if Z flag is not set
    if (getFlag(STATUS_FLAG::Z) == 0)
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

/**
 * @brief BPL (branch plus)
 * 
 * @return uint8_t 
 */
uint8_t CPU6502::BPL()
{
    // BMI is true if N (negative) flag is 1
    if (getFlag(STATUS_FLAG::N) == 0)
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

/**
 * @brief BRK
 * 
 * @return uint8_t 
 */
uint8_t CPU6502::BRK()
{
    return 0;
}

/**
 * @brief BVC (branch overflow clear)
 * 
 * @return uint8_t 
 */
uint8_t CPU6502::BVC()
{
    if (getFlag(STATUS_FLAG::O) == 0)
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

/**
 * @brief BVS (branch overflow set)
 * 
 * @return uint8_t 
 */
uint8_t CPU6502::BVS()
{
    if (getFlag(STATUS_FLAG::O) == 1)
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

/**
 * @brief CLC (clear carry)
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::CLC()
{
    setFlag(STATUS_FLAG::C, false);
    return 0;
}

/**
 * @brief CLD (clear decimal)
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::CLD()
{
    setFlag(STATUS_FLAG::D, false);

    return 0;
}

/**
 * @brief CLI (clear interrupt flag)
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::CLI()
{
    setFlag(STATUS_FLAG::I, false);
    return 0;
}

/**
 * @brief CLV (clear overflow flag)
 * 
 * @return uint8_t 
 */
uint8_t CPU6502::CLV()
{
    setFlag(STATUS_FLAG::O, false);
    return 0;
}

/**
 * @brief CMP (compare)
 * 
 * @return uint8_t 0
 */
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

/**
 * @brief Push A to stack
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::PHA()
{
    write(STACK_BASE + s, a);
    s--;
    return 0;
}
uint8_t CPU6502::PHP()
{
    return 0;
}

/**
 * @brief Pop A from stack
 * 
 * @return uint8_t 0
 */
uint8_t CPU6502::PLA()
{
    s++;
    a = read(STACK_BASE + s);
    setFlag(STATUS_FLAG::N, a & 0x80);
    setFlag(STATUS_FLAG::Z, a == 0x00);

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

/**
 * @brief Return from interrupt
 * 
 * @return uint8_t extra cycles
 */
uint8_t CPU6502::RTI()
{
    s++;
    p = read(STACK_BASE + s);
    p &= ~STATUS_FLAG::B;
    p &= ~STATUS_FLAG::U;

    s++;
    pc = (uint16_t)read(STACK_BASE + s);
    s++;
    pc |= (uint16_t)read(STACK_BASE + s) << 8;
    return 0;
}
uint8_t CPU6502::RTS()
{
    return 0;
}

/**
 * @brief Subtract with carry
 * 
 * @return uint8_t extra cycle
 */
uint8_t CPU6502::SBC()
{
    fetch();
    uint8_t sub_val = ((uint16_t)fetched_data) ^ 0x00FF;
    uint8_t subtraction = (uint16_t)a + sub_val + getFlag(STATUS_FLAG::C);

    // Set flags appropriately
    setFlag(STATUS_FLAG::C, subtraction & 0xFF00);
    setFlag(STATUS_FLAG::N, subtraction & 0x0080);
    setFlag(STATUS_FLAG::Z, (subtraction & 0x00FF) == 0);
    setFlag(STATUS_FLAG::O, (subtraction ^ (uint16_t)a) & (subtraction ^ sub_val) & 0x0080);

    a = subtraction & 0x00FF;

    return 1;
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