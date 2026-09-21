#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

// // disable compiler padding to guarantee exact byte alignment
// #pragma pack(push, 1)

typedef struct {
    uint8_t sof;              // 1 bit
    uint16_t id;              // 11 bits
    uint8_t rtr;              // 1 bit
    uint8_t ide;              // 1 bit
    uint8_t r0;               // 1 bit
    uint8_t dlc;              // 4 bits
    uint64_t data;            // 0-64 bits
    uint16_t crc;             // 15 bits
    uint8_t crc_delimiter;    // 1 bit
    uint8_t ack_slot;         // 1 bit
    uint8_t ack_delimiter;    // 1 bit
    uint8_t eof;              // 7 bits
    uint8_t ifs;              // 3 bits
} baseCanFrame;

// #pragma pack(pop)

// function declarations
uint8_t start_of_frame();
uint16_t identifier(uint16_t id);
uint8_t remote_transmission_request();
uint8_t identifier_extension();
uint8_t reserved_bit();
uint8_t data_length_code(uint8_t dlc);
uint64_t data_field(uint64_t data);
uint16_t crc(uint64_t data);
uint8_t crc_delimiter();
uint8_t ack_slot();
uint8_t ack_delimiter();
uint8_t end_of_frame();
uint8_t inter_frame_spacing();

uint16_t generator_CRC15(uint64_t data, uint8_t data_bits);
int bit_length(uint64_t data);

int main(void)
{
    baseCanFrame frame;

    id = 0x123;
    data = 0x123456789ABCDEF0;

    frame.sof = start_of_frame();
    frame.id = identifier(id);
    frame.rtr = remote_transmission_request();
    frame.ide = identifier_extension();
    frame.r0 = reserved_bit();
    frame.dlc = data_length_code(8);
    frame.data = data_field(data);
    frame.crc = crc(frame.data);
    frame.crc_delimiter = crc_delimiter();
    frame.ack_slot = ack_slot();
    frame.ack_delimiter = ack_delimiter();
    frame.eof = end_of_frame();
    frame.ifs = inter_frame_spacing();

    printf("SOF: %u\n", frame.sof);
    printf("ID: %u\n", frame.id);
    printf("RTR: %u\n", frame.rtr);
    printf("IDE: %u\n", frame.ide);
    printf("DLC: %u\n", frame.dlc);

    return 0;
}


/*
    start of frame | 1 bit/s | denotes the start of frame transmission
*/
uint8_t start_of_frame() { return 0b0; }

/*
    identifier | 11 bit/s | a (unique) identifier which also represents the message priority
*/
uint16_t identifier(uint16_t id) { return id & 0x7FF; }

/*
    remote transmission request (RTR) | 1 bit/s | must be dominant (0) for data frames and recessive (1) for remote request frames
*/
uint8_t remote_transmission_request() { return 0b0; }

/*
    identifier extension bit | 1 bit/s | must be dominant (0) for base frame format with 11-bit identifiers
*/
uint8_t identifier_extension() { return 0b0; }

/*
    reserved bit (r0) | 1 bit/s | must be dominant (0), but accepted as either dominant or recessive
*/
uint8_t reserved_bit() { return 0b0; }

/*
    data length code (DLC) | 4 bit/s | number of bytes of data (0-8 bytes)
*/
uint8_t data_length_code(uint8_t dlc)
{
    if (dlc > 8) { dlc = 8; }
    return dlc;
}

/*
    data field | 0-64 bit/s | data to be transmitted (length in bytes dictated by DLC field)
*/
uint64_t data_field(uint64_t data) { return data; }

/*
    CRC | 15 bit/s | cylic redundancy check
*/
uint16_t crc(uint64_t data)
{
    return generator_CRC15(data, bit_length(data));
}

/*
    CRC delimiter | 1 bit/s | must be recessive (1)
*/
uint8_t crc_delimiter() { return 0b1; }

/*
    ACK slot | 1 bit/s | transmitter sends recessive (1) and any receiver can assert a dominant (0)
*/
uint8_t ack_slot() { return 0b1; }

/*
    ACK delimiter | 1 bit/s | must be recessive (1)
*/
uint8_t ack_delimiter() { return 0b1; }

/*
    end of frame (EOF) | 7 bit/s | must be recessive (1)
*/
uint8_t end_of_frame() { return 0b1111111; }

/*
    inter frame spacing (IFS) | 3 bit/s | must be recessive (1)
*/
uint8_t inter_frame_spacing() { return 0b111; }

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

