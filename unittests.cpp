#define CATCH_CONFIG_MAIN

#include "6502.hpp"
#include "catch.hpp"

TEST_CASE( "Test is8BitNeg", "[is8BitNeg]" ) {

    SECTION("uint8_t")
    {
        uint8_t positive = 0b00010101;
        uint8_t negative = 0b10010101;

        REQUIRE(is8BitNeg(negative));
        REQUIRE_FALSE(is8BitNeg(positive));
    }
    SECTION("uint16_t")
    {
        uint16_t positive = 0b0000010000010101;
        uint16_t negative = 0b0100010010010101;

        REQUIRE(is8BitNeg(negative));
        REQUIRE_FALSE(is8BitNeg(positive));
    }
    SECTION("uint32_t")
    {
        uint32_t positive = 0x12345678;
        uint32_t negative = 0x12345688;

        REQUIRE(is8BitNeg(negative));
        REQUIRE_FALSE(is8BitNeg(positive));
    }
}