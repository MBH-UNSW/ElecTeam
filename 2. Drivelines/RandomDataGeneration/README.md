# Random Data Generation

## Purpose
This feature uses the STM32G431KB hardware RNG peripheral to generate a 32-bit random number, convert it into individual bits, and output the bit sequence through:
* PB4 GPIO output (HIGH/LOW signal)
* USART2 serial terminal output
A new random sequence is generated continuously in the main loop.

## Hardware
**Board:** STM32 NUCLEO-G431KB

**Connections:**
* PB4 → LED (with current limiting resistor) or logic analyser/oscilloscope
* USB connection for programming and UART communication

**UART Settings:**
* Baud Rate: 115200
* Data Bits: 8
* Stop Bits: 1
* Parity: None

## CubeMX Configuration
### Clock
* HSI (16 MHz) with PLL enabled
* System Clock: 96 MHz

### Peripherals
* RNG enabled
* USART2 enabled (Asynchronous, 115200 baud)
* GPIO PB4 configured as Output Push-Pull
* TIM1 enabled (not used in current implementation)

| Pin | Function |
|------|----------|
| PA2 | USART2_TX |
| PA3 | USART2_RX |
| PB4 | Random bit output |

## Code Functionality
* `Generate_Random_Bit_Array()` generates a random 32-bit number and stores each bit in an array.
* `USART_Send_Bit_Sequence()` sends each bit value over UART.
* `Sequence_Output()` outputs the bit sequence on PB4 and sends UART messages.

## Build & Test
1. Build and flash the project to the NUCLEO-G431KB.
2. Open a serial terminal at 115200 baud.
3. Observe random bit values in the terminal and corresponding HIGH/LOW output on PB4.

## Pinout View + Clock Configruation Screenshots
<img width="774" height="554" alt="image" src="https://github.com/user-attachments/assets/a6fa512a-84d4-4643-a519-b9ab38a52218" />
<img width="1382" height="719" alt="image" src="https://github.com/user-attachments/assets/5de8fbd7-bcea-4adf-b279-d198f98ca6f9" />
