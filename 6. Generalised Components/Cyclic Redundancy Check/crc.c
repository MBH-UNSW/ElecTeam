#include <stdint.h>
#include <stdio.h>

// function declaration
uint16_t generatorCRC15(uint16_t data);

/*
    generates the CRC using CRC-15 CAN using polynomial is x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
*/ 
uint16_t generatorCRC15(uint16_t data)
{
    // need to first append 15 zeros onto the right of the data message
    uint32_t message = ((uint32_t)data) << 15;

    // go through and continue to XOR the new data message with the polynomial to form the remainder
    uint32_t polynomial = 0xC599;

    for (int i = 30; i >= 15; i--)
    {
        if (message & ((uint32_t)1 << i))
        {
            message ^= polynomial << (i - 15);
        }
    }

    // return the remainder
    return (uint16_t)(message & 0x7FFF);
}

int main()
{
    uint16_t remainder = generatorCRC15(0b101011010111010);
    printf("%u\n", remainder);

    return 0;
}