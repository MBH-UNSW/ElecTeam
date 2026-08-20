#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

// function declaration
uint16_t generator_CRC15(uint64_t data, uint8_t data_bits);
bool checker_CRC15(uint64_t data, uint32_t crc);
int bit_length(uint64_t data);

/*
    generates the CRC using CRC-15 CAN using polynomial is x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
*/
uint16_t generator_CRC15(uint64_t data, uint8_t data_bits)
{
    // need to first append 15 zeros onto the right of the data message
    uint16_t remainder = 0;
    uint16_t polynomial = 0x4599;

    // go through and continue to XOR the new data message with the polynomial to form the remainder
    for (int i = data_bits - 1; i >= 0; i--)
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

/*
    checks if data is correct by validating CRC equivalency
*/
bool checker_CRC15(uint64_t data, uint32_t crc)
{
    int data_bits = bit_length(data);
    uint16_t calculated_CRC = generator_CRC15(data, data_bits);

    // check if calculated crc is equivalent to actual crc
    if (calculated_CRC == crc)
    {
        return true;
    }
    
    return false;
}

/*
    calculate data bit length
*/
int bit_length(uint64_t data)
{
    int length = 0;

    while (data > 0)
    {
        length++;
        data >>= 1;
    }

    return length;
}

int main()
{
    uint64_t data = 0b101011010111010111010111100011;
    uint16_t crc = generator_CRC15(data, 30);

    printf("CRC: %u\n", crc);

    if (checker_CRC15(data, crc))
    {
        printf("CRC valid\n");
    }
    else
    {
        printf("CRC invalid\n");
    }

    return 0;
}