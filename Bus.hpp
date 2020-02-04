#pragma once

#include <cstdint>
#include "6502.hpp"

/**
 * @brief Bus class
 */
class Bus
{
public:

    /**
     * @brief Construct a new Bus object
     */
    Bus();

    /**
     * @brief Destroy the Bus object
     */
    ~Bus();

    CPU6502 cpu;
    uint8_t ram[64 * 1024];

    /**
     * @brief Bus read function
     * 
     * @param addr 16 bit address to read
     * @param readOnly flag readOnly 
     * @return uint8_t data at addr on bus
     */
    uint8_t read(uint16_t addr, bool readOnly = false);

    /**
     * @brief Bus write function
     * 
     * @param addr 16 bit address to write
     * @param data data to write to addr
     */
    void write(uint16_t addr, uint8_t data);

private:
    uint16_t RAM_MIN = 0x0000;
    uint16_t RAM_MAX = 0xFFFF;

};