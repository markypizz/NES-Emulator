#include "Bus.hpp"

/**
 * @brief Construct a new Bus object
 */
Bus::Bus()
{
    // Clear RAM
    for (auto& i : ram)
    {
        i = 0x00;
    }

    // Connect bus to CPU
    cpu.connectBus(this);
}

/**
 * @brief Destroy the Bus object
 */
Bus::~Bus()
{

}


/**
 * @brief Bus read function
 * 
 * @param addr 16 bit address to read
 * @param readOnly flag readOnly 
 * @return uint8_t data at addr on bus
 */
uint8_t Bus::read(uint16_t addr, bool readOnly)
{
    if (addr >= RAM_MIN && addr <= RAM_MAX)
    {
        return ram[addr];
    }

    // Failsafe
    return 0x00;
}

/**
 * @brief Bus write function
 * 
 * @param addr 16 bit address to write
 * @param data data to write to addr
 */
void Bus::write(uint16_t addr, uint8_t data)
{
    if (addr >= RAM_MIN && addr <= RAM_MAX)
    {
        ram[addr] = data;
    }
}

