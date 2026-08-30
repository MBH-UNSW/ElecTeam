/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stddef.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
} CAN_Message_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_FDCAN1_Init(void);

/* USER CODE BEGIN PFP */

HAL_StatusTypeDef CAN_Init(void);

HAL_StatusTypeDef CAN_Send(
    uint32_t id,
    const uint8_t *data,
    uint8_t length
);

HAL_StatusTypeDef CAN_Receive(
    CAN_Message_t *message
);

uint8_t CAN_MessageAvailable(void);

static uint32_t CAN_LengthToDLC(
    uint8_t length
);

static uint8_t CAN_DLCToLength(
    uint32_t dlc
);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void)
{
    /* USER CODE BEGIN 1 */

    CAN_Message_t rxMessage;

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */

    MX_GPIO_Init();

    MX_FDCAN1_Init();

    /* USER CODE BEGIN 2 */

    if (CAN_Init() != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE END 2 */

    /* Infinite loop */

    /* USER CODE BEGIN WHILE */

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        if (CAN_MessageAvailable())
        {
            if (CAN_Receive(&rxMessage) == HAL_OK)
            {
                /*
                 * Received CAN message is now stored in:
                 *
                 * rxMessage.id
                 * rxMessage.length
                 * rxMessage.data[0]
                 * rxMessage.data[1]
                 * ...
                 * rxMessage.data[7]
                 */

                switch (rxMessage.id)
                {
                    case 0x100:
                    {
                        /*
                         * Handle CAN ID 0x100 here
                         */

                        break;
                    }

                    case 0x101:
                    {
                        /*
                         * Handle CAN ID 0x101 here
                         */

                        break;
                    }

                    case 0x102:
                    {
                        /*
                         * Handle CAN ID 0x102 here
                         */

                        break;
                    }

                    default:
                    {
                        /*
                         * Unknown CAN ID
                         */

                        break;
                    }
                }
            }
        }
    }

    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(
        PWR_REGULATOR_VOLTAGE_SCALE1_BOOST
    );

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;

    RCC_OscInitStruct.PLL.PLLN = 85;

    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;

    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;

    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */

static void MX_FDCAN1_Init(void)
{
    /* USER CODE BEGIN FDCAN1_Init 0 */

    /* USER CODE END FDCAN1_Init 0 */

    /* USER CODE BEGIN FDCAN1_Init 1 */

    /* USER CODE END FDCAN1_Init 1 */

    hfdcan1.Instance = FDCAN1;

    hfdcan1.Init.ClockDivider =
        FDCAN_CLOCK_DIV1;

    hfdcan1.Init.FrameFormat =
        FDCAN_FRAME_CLASSIC;

    hfdcan1.Init.Mode =
        FDCAN_MODE_NORMAL;

    hfdcan1.Init.AutoRetransmission =
        DISABLE;

    hfdcan1.Init.TransmitPause =
        DISABLE;

    hfdcan1.Init.ProtocolException =
        DISABLE;

    hfdcan1.Init.NominalPrescaler =
        1;

    hfdcan1.Init.NominalSyncJumpWidth =
        1;

    hfdcan1.Init.NominalTimeSeg1 =
        148;

    hfdcan1.Init.NominalTimeSeg2 =
        21;

    hfdcan1.Init.DataPrescaler =
        1;

    hfdcan1.Init.DataSyncJumpWidth =
        1;

    hfdcan1.Init.DataTimeSeg1 =
        1;

    hfdcan1.Init.DataTimeSeg2 =
        1;

    hfdcan1.Init.StdFiltersNbr =
        0;

    hfdcan1.Init.ExtFiltersNbr =
        0;

    hfdcan1.Init.TxFifoQueueMode =
        FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(
            &hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN FDCAN1_Init 2 */

    /* USER CODE END FDCAN1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */

static void MX_GPIO_Init(void)
{
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */

    __HAL_RCC_GPIOF_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * Convert number of data bytes to STM32 FDCAN DLC value.
 *
 * Classic CAN supports 0-8 bytes.
 */

static uint32_t CAN_LengthToDLC(
    uint8_t length)
{
    switch (length)
    {
        case 0:
            return FDCAN_DLC_BYTES_0;

        case 1:
            return FDCAN_DLC_BYTES_1;

        case 2:
            return FDCAN_DLC_BYTES_2;

        case 3:
            return FDCAN_DLC_BYTES_3;

        case 4:
            return FDCAN_DLC_BYTES_4;

        case 5:
            return FDCAN_DLC_BYTES_5;

        case 6:
            return FDCAN_DLC_BYTES_6;

        case 7:
            return FDCAN_DLC_BYTES_7;

        case 8:
            return FDCAN_DLC_BYTES_8;

        default:
            return FDCAN_DLC_BYTES_0;
    }
}


/*
 * Convert received STM32 FDCAN DLC value back into
 * number of data bytes.
 */

static uint8_t CAN_DLCToLength(
    uint32_t dlc)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0:
            return 0;

        case FDCAN_DLC_BYTES_1:
            return 1;

        case FDCAN_DLC_BYTES_2:
            return 2;

        case FDCAN_DLC_BYTES_3:
            return 3;

        case FDCAN_DLC_BYTES_4:
            return 4;

        case FDCAN_DLC_BYTES_5:
            return 5;

        case FDCAN_DLC_BYTES_6:
            return 6;

        case FDCAN_DLC_BYTES_7:
            return 7;

        case FDCAN_DLC_BYTES_8:
            return 8;

        default:
            return 0;
    }
}


/*
 * Initialise CAN operation.
 *
 * The CubeMX peripheral configuration has already been
 * performed by MX_FDCAN1_Init().
 *
 * Since StdFiltersNbr = 0 and ExtFiltersNbr = 0,
 * the global filter is used to accept messages.
 */

HAL_StatusTypeDef CAN_Init(void)
{
    HAL_StatusTypeDef status;

    /*
     * Accept non-matching standard IDs into FIFO0.
     *
     * Accept non-matching extended IDs into FIFO0.
     *
     * Reject standard remote frames.
     *
     * Reject extended remote frames.
     */

    status = HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Start the FDCAN peripheral.
     */

    status = HAL_FDCAN_Start(
        &hfdcan1
    );

    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}


/*
 * Send a Classic CAN frame using an 11-bit standard ID.
 *
 * id:
 *      0x000 - 0x7FF
 *
 * data:
 *      Pointer to data to transmit
 *
 * length:
 *      0 - 8 bytes
 */

HAL_StatusTypeDef CAN_Send(
    uint32_t id,
    const uint8_t *data,
    uint8_t length)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};

    /*
     * Validate standard CAN identifier.
     */

    if (id > 0x7FF)
    {
        return HAL_ERROR;
    }

    /*
     * Classic CAN can contain at most 8 bytes.
     */

    if (length > 8)
    {
        return HAL_ERROR;
    }

    /*
     * Data pointer must exist if data length
     * is greater than zero.
     */

    if ((data == NULL) && (length > 0))
    {
        return HAL_ERROR;
    }

    /*
     * Configure CAN frame header.
     */

    txHeader.Identifier =
        id;

    txHeader.IdType =
        FDCAN_STANDARD_ID;

    txHeader.TxFrameType =
        FDCAN_DATA_FRAME;

    txHeader.DataLength =
        CAN_LengthToDLC(length);

    txHeader.ErrorStateIndicator =
        FDCAN_ESI_ACTIVE;

    txHeader.BitRateSwitch =
        FDCAN_BRS_OFF;

    txHeader.FDFormat =
        FDCAN_CLASSIC_CAN;

    txHeader.TxEventFifoControl =
        FDCAN_NO_TX_EVENTS;

    txHeader.MessageMarker =
        0;

    /*
     * Add message to the FDCAN transmit FIFO.
     */

    return HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1,
        &txHeader,
        (uint8_t *)data
    );
}


/*
 * Check whether at least one CAN message is waiting
 * inside RX FIFO0.
 *
 * Returns:
 *
 * 0 = no message
 * 1 = message available
 */

uint8_t CAN_MessageAvailable(void)
{
    if (HAL_FDCAN_GetRxFifoFillLevel(
            &hfdcan1,
            FDCAN_RX_FIFO0) > 0)
    {
        return 1;
    }

    return 0;
}


/*
 * Read one message from RX FIFO0.
 *
 * The received information is placed inside
 * CAN_Message_t.
 */

HAL_StatusTypeDef CAN_Receive(
    CAN_Message_t *message)
{
    FDCAN_RxHeaderTypeDef rxHeader = {0};

    HAL_StatusTypeDef status;

    /*
     * Check pointer.
     */

    if (message == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * Check whether anything is actually waiting.
     */

    if (HAL_FDCAN_GetRxFifoFillLevel(
            &hfdcan1,
            FDCAN_RX_FIFO0) == 0)
    {
        return HAL_ERROR;
    }

    /*
     * Retrieve CAN message from FIFO0.
     */

    status = HAL_FDCAN_GetRxMessage(
        &hfdcan1,
        FDCAN_RX_FIFO0,
        &rxHeader,
        message->data
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Save identifier.
     */

    message->id =
        rxHeader.Identifier;

    /*
     * Convert DLC into actual byte count.
     */

    message->length =
        CAN_DLCToLength(
            rxHeader.DataLength
        );

    return HAL_OK;
}

/* USER CODE END 4 */


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */

    /* User can add his own implementation to report the HAL error return state */

    __disable_irq();

    while (1)
    {
    }

    /* USER CODE END Error_Handler_Debug */
}


#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */

void assert_failed(
    uint8_t *file,
    uint32_t line)
{
    /* USER CODE BEGIN 6 */

    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */