#include <stdint.h>
#include <stdio.h>

// function declaration
uint16_t generatorCRC15(uint64_t data, uint8_t dataBits);

/*
    generates the CRC using CRC-15 CAN using polynomial is x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
*/
uint16_t generatorCRC15(uint64_t data, uint8_t dataBits)
{
    // need to first append 15 zeros onto the right of the data message
    uint16_t remainder = 0;
    uint16_t polynomial = 0x4599;

    // go through and continue to XOR the new data message with the polynomial to form the remainder
    for (int i = dataBits - 1; i >= 0; i--)
    {
        uint8_t dataBit = (data >> i) & 1;
        uint8_t remainderBit = (remainder >> 14) & 1;
        remainder = (remainder << 1) & 0x7FFF;
        if (dataBit ^ remainderBit)
        {
            remainder ^= polynomial;
        }
    }

    // append the 15 zeros
    for (int i = 0; i < 15; i++)
    {
        uint8_t remainderBit = (remainder >> 14) & 1;
        remainder = (remainder << 1) & 0x7FFF;
        if (remainderBit)
        {
            remainder ^= polynomial;
        }
    }

    // return the remainder
    return remainder;
}

int main()
{
    uint64_t data = 0b101011010111010111010111100011;
    uint16_t remainder = generatorCRC15(data, 30);
    printf("%u\n", remainder);
    
    return 0;
}