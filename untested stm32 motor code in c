/* =========================================================
 * BLDC Motor Control with Hall Sensors
 * STM32 Version (HAL Library - C)
 * Converted from ESP32 Arduino C++
 *
 * Target: STM32 (e.g. STM32F4/F1/G4)
 * Uses:
 *  - TIM PWM outputs
 *  - GPIO outputs
 *  - ADC for Hall sensors
 *  - UART for serial commands
 * ========================================================= */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* -------------------- EXTERNAL HANDLES -------------------- */
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;

/* -------------------- MOTOR CONFIG -------------------- */
#define POLE_PAIRS        2
#define RPM_TOLERANCE     20.0f

/* -------------------- PWM CONFIG -------------------- */
#define PWM_MAX           255

/* -------------------- HALL THRESHOLD -------------------- */
#define HALL_THRESHOLD    2048   // midpoint for 12-bit ADC

/* -------------------- VARIABLES -------------------- */
volatile uint8_t hallState = 0;
uint8_t lastState = 0;

uint8_t dutyCycle = 150;
uint8_t stopMotor = 0;
uint8_t first_time = 0;

const uint8_t hallSequence[6] = {1, 5, 4, 6, 2, 3};
uint8_t currentIndex = 0;

/* RPM measurement */
volatile uint32_t commutationCount = 0;
uint32_t lastRPMUpdate = 0;

float rpm = 0;
float lastPrintedRPM = 0;

/* UART RX buffer */
char rxBuffer[50];

/* -------------------- FUNCTION DECLARATIONS -------------------- */
void updateHallState(void);
void commutate(uint8_t state);
void stopAll(void);
void updateRPM(void);
void processCommand(char *cmd);
uint16_t readADC(uint32_t channel);
void uartPrint(char *msg);

/* =========================================================
 * MAIN
 * ========================================================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_USART2_UART_Init();

    /* Start PWM channels */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // UH
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // VH
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // WH

    updateHallState();
    commutate(hallState);

    lastState = hallState;

    uartPrint("BLDC Controller Started\r\n");

    while (1)
    {
        /* ---------------- UART COMMANDS ---------------- */

        memset(rxBuffer, 0, sizeof(rxBuffer));

        if (HAL_UART_Receive(&huart2,
                             (uint8_t*)rxBuffer,
                             sizeof(rxBuffer),
                             10) == HAL_OK)
        {
            processCommand(rxBuffer);
        }

        /* ---------------- COMMUTATION ---------------- */

        if (!stopMotor && dutyCycle > 0)
        {
            updateHallState();

            if (hallState == 0 || hallState == 7)
            {
                uartPrint("[WARN] Invalid Hall State\r\n");
            }

            if (first_time)
            {
                for (int i = 0; i < 6; i++)
                {
                    if (hallSequence[i] == hallState)
                    {
                        currentIndex = i;
                        break;
                    }
                }

                currentIndex = (currentIndex + 1) % 6;

                hallState = hallSequence[currentIndex];

                commutate(hallState);

                commutationCount++;
                lastState = hallState;

                first_time = 0;
            }
            else if (hallState != lastState)
            {
                commutate(hallState);

                commutationCount++;
                lastState = hallState;
            }
        }

        /* ---------------- RPM UPDATE ---------------- */

        if ((HAL_GetTick() - lastRPMUpdate) >= 1000)
        {
            updateRPM();
            lastRPMUpdate = HAL_GetTick();
        }
    }
}

/* =========================================================
 * UPDATE HALL STATE
 * ========================================================= */
void updateHallState(void)
{
    uint16_t u = readADC(ADC_CHANNEL_0);
    uint16_t v = readADC(ADC_CHANNEL_1);
    uint16_t w = readADC(ADC_CHANNEL_2);

    uint8_t digU = (u > HALL_THRESHOLD) ? 1 : 0;
    uint8_t digV = (v > HALL_THRESHOLD) ? 1 : 0;
    uint8_t digW = (w > HALL_THRESHOLD) ? 1 : 0;

    hallState = (digU << 2) | (digV << 1) | digW;
}

/* =========================================================
 * COMMUTATION TABLE
 * ========================================================= */
void commutate(uint8_t state)
{
    stopAll();

    switch(state)
    {
        case 1: // 001
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyCycle);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // VL
            break;

        case 5: // 101
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyCycle);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // WL
            break;

        case 4: // 100
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyCycle);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // WL
            break;

        case 6: // 110
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyCycle);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // UL
            break;

        case 2: // 010
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyCycle);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // UL
            break;

        case 3: // 011
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyCycle);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // VL
            break;

        default:
            break;
    }
}

/* =========================================================
 * STOP ALL OUTPUTS
 * ========================================================= */
void stopAll(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

/* =========================================================
 * RPM CALCULATION
 * ========================================================= */
void updateRPM(void)
{
    float newRPM =
        (commutationCount * 60.0f) /
        (6.0f * POLE_PAIRS);

    commutationCount = 0;

    if (fabs(newRPM - lastPrintedRPM) >= RPM_TOLERANCE)
    {
        rpm = newRPM;
        lastPrintedRPM = rpm;

        char msg[64];

        sprintf(msg, "[RPM] %.2f RPM\r\n", rpm);

        uartPrint(msg);
    }
}

/* =========================================================
 * UART COMMAND PROCESSING
 * ========================================================= */
void processCommand(char *cmd)
{
    if (strncmp(cmd, "STOP", 4) == 0)
    {
        stopMotor = 1;
        stopAll();

        uartPrint("[CMD] Motor Stopped\r\n");
    }

    else if (strncmp(cmd, "START", 5) == 0)
    {
        stopMotor = 0;
        first_time = 1;

        uartPrint("[CMD] Motor Started\r\n");
    }

    else if (strncmp(cmd, "SPEED", 5) == 0)
    {
        int val = atoi(&cmd[6]);

        if (val < 0) val = 0;
        if (val > 255) val = 255;

        dutyCycle = val;

        uartPrint("[CMD] Speed Updated\r\n");
    }
}

/* =========================================================
 * ADC READ
 * ========================================================= */
uint16_t readADC(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

    return HAL_ADC_GetValue(&hadc1);
}

/* =========================================================
 * UART PRINT
 * ========================================================= */
void uartPrint(char *msg)
{
    HAL_UART_Transmit(&huart2,
                      (uint8_t*)msg,
                      strlen(msg),
                      HAL_MAX_DELAY);
}
