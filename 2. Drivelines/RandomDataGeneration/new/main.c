/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LSK data transmitter - drives Q1 (DATA_SCT) with a random
  *                   32-bit sequence so each bit loads/unloads the inductor tank
  ******************************************************************************
  * Target board : custom STM32G431CBT6 (LQFP48) data-transfer PCB
  *
  * Pin map (from schematic):
  *   PA8  (pin 30) DATA_SCT    -> Q1 gate (IRFL4310), R15 10k pull-down on board
  *   PB3  (pin 40) STATUS_LED  -> D6 via R11 220R  (mirrors the bit being sent)
  *   PB8  (pin 45) BOOT0       -> R12 10k pull-down, left untouched
  *   PA13/PA14     SWDIO/SWCLK -> J3 programming header
  *   PF0/PF1       OSC_IN/OUT  -> Y1 24 MHz HSE crystal
  *   PA10/11/12    CAN_STBY / FDCAN_RX / FDCAN_TX (not used here)
  *
  * Bit encoding (NRZ, LSB first):
  *   bit 1 -> DATA_SCT HIGH -> Q1 ON  -> tank loaded   (LSK "1")
  *   bit 0 -> DATA_SCT LOW  -> Q1 OFF -> tank unloaded (LSK "0")
  *
  * Clocks: HSE 24 MHz -> PLL -> SYSCLK 96 MHz, PLLQ 48 MHz for the RNG.
  * TIM1 runs at 1 MHz (1 us tick) and its update flag sets the bit period,
  * so bit timing does not depend on loop/HAL overhead.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* ---------------- User settings ---------------- */
#define RANDOM_ARRAY_SIZE   32
#define DATA_RATE_BPS       10000U   /* 10 kb/s -> 100 us per bit (30 cycles of a 300 kHz carrier) */
#define FRAME_GAP_BITS      16U      /* idle (Q1 off) bit-times between frames */

/* Set to 1 to send 1 bit per second so you can watch it on the status LED,
 * set to 0 for real-rate transmission timed by TIM1. */
#define DEBUG_SLOW_MODE     0
#define DEBUG_BIT_DELAY_MS  1000U

/* ---------------- Pin map ---------------- */
#define LSK_DATA_PORT       GPIOA
#define LSK_DATA_PIN        GPIO_PIN_8   /* DATA_SCT -> Q1 gate */
#define STATUS_LED_PORT     GPIOB
#define STATUS_LED_PIN      GPIO_PIN_3   /* D6 */

#define BIT_HIGH 1
#define BIT_LOW  0

/* TIM1 tick = 1 us, so the auto-reload value is the bit period in us */
#define TIM1_TICK_HZ        1000000U
#define BIT_PERIOD_TICKS    (TIM1_TICK_HZ / DATA_RATE_BPS)

volatile uint32_t random_number = 0;
volatile uint8_t  random_number_bits[RANDOM_ARRAY_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM1_Init(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/** Generates a 32-bit random number with the RNG peripheral, then splits it
  * into an array of individual bits (LSB first) for transmission.
  */
void Generate_Random_Bit_Array(void)
{
  if (HAL_RNG_GenerateRandomNumber(&hrng, (uint32_t *)&random_number) != HAL_OK)
  {
    Error_Handler();
  }

  for (int i = 0; i < RANDOM_ARRAY_SIZE; i++)
  {
    /* shift right by i, mask off everything but the lowest bit */
    random_number_bits[i] = (random_number >> i) & 0x01;
  }
}

/** Drives the MOSFET gate (and mirrors it on the status LED). */
static inline void LSK_Write_Bit(uint8_t bit)
{
  GPIO_PinState state = (bit == BIT_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(LSK_DATA_PORT, LSK_DATA_PIN, state);       /* Q1 on/off */
  HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, state);   /* visual copy */
}

/** Blocks until the next TIM1 update event, i.e. the end of the current bit. */
static inline void LSK_Wait_Bit_Period(void)
{
#if DEBUG_SLOW_MODE
  HAL_Delay(DEBUG_BIT_DELAY_MS);
#else
  while (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) == RESET)
  {
    /* wait */
  }
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
#endif
}

/** Restarts the bit clock so the first bit of a frame gets a full period. */
static void LSK_Bit_Clock_Restart(void)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
}

/** Sends the random bit sequence by switching Q1: HIGH = load tank, LOW = release.
  * Q1 is always left OFF at the end so the tank is unloaded while idle.
  */
void Sequence_Output(void)
{
  LSK_Bit_Clock_Restart();

  for (int i = 0; i < RANDOM_ARRAY_SIZE; i++)
  {
    LSK_Write_Bit(random_number_bits[i]);
    LSK_Wait_Bit_Period();
  }

  /* idle: Q1 off (R15 also holds the gate low if the pin ever floats) */
  LSK_Write_Bit(BIT_LOW);
  for (uint32_t g = 0; g < FRAME_GAP_BITS; g++)
  {
    LSK_Wait_Bit_Period();
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_RNG_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  /* start the free-running bit clock (polled, no interrupt) */
  if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Generate_Random_Bit_Array();
    Sequence_Output();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *        HSE 24 MHz / M2 = 12 MHz -> x16 = 192 MHz VCO
  *        PLLR /2 = 96 MHz SYSCLK, PLLQ /4 = 48 MHz (RNG)
  *        Make sure HSE_VALUE = 24000000 in stm32g4xx_hal_conf.h
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function (clocked from PLLQ = 48 MHz)
  */
static void MX_RNG_Init(void)
{
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  *        96 MHz / (95 + 1) = 1 MHz tick; ARR = bit period in us - 1
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = (96000000U / TIM1_TICK_HZ) - 1U;   /* 95 */
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = BIT_PERIOD_TICKS - 1U;                /* 99 for 10 kb/s */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  *        PA8 = DATA_SCT (Q1 gate), PB3 = STATUS_LED. Both start LOW.
  *        PB8/BOOT0 is deliberately not configured (R12 holds it low).
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();   /* OSC_IN / OSC_OUT */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* outputs low before enabling so Q1 never glitches on at boot */
  HAL_GPIO_WritePin(LSK_DATA_PORT, LSK_DATA_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);

  /* PA8 - DATA_SCT: push-pull, medium speed for clean gate edges */
  GPIO_InitStruct.Pin = LSK_DATA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;            /* R15 10k pull-down on board */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LSK_DATA_PORT, &GPIO_InitStruct);

  /* PB3 - STATUS_LED (PB3 defaults to SWO; SWD on PA13/PA14 is unaffected) */
  GPIO_InitStruct.Pin = STATUS_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_PORT, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  *         Forces Q1 off and leaves the status LED solid on.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  LSK_DATA_PORT->BRR    = LSK_DATA_PIN;    /* Q1 off */
  STATUS_LED_PORT->BSRR = STATUS_LED_PIN;  /* LED on = fault */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
