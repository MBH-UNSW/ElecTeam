/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    MOTOR_STOPPED = 0,          /* No voltage is applied to the motor. */
    MOTOR_ALIGNING,             /* Hold rotor at a known starting position. */
    MOTOR_STARTING,             /* Force rotation until handover RPM is reached. */
    MOTOR_RUNNING,              /* Normal Hall-sensor PI speed control. */
    MOTOR_FAULT                 /* Startup or Hall feedback failed. */
} MotorControlState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI_RADIANS                      3.14159265358979323846f
#define SQRT_3_OVER_2                   0.8660254037844386f

/* TIM6 executes the motor-control interrupt at 20 kHz. */
#define MOTOR_CONTROL_FREQUENCY_HZ      20000.0f

/* The speed PI controller executes every 20 motor-control interrupts (1 kHz). */
#define SPEED_CONTROLLER_DIVIDER        20u
#define SPEED_CONTROLLER_FREQUENCY_HZ   \
    (MOTOR_CONTROL_FREQUENCY_HZ / SPEED_CONTROLLER_DIVIDER)

/* Motor settings -----------------------------------------------------------*/
#define MOTOR_POLE_PAIRS                2.0f   /* Number of rotor poles / 2 */
#define MOTOR_TARGET_SPEED_RPM          400.0f /* Required mechanical speed */
#define MOTOR_ROTATION_DIRECTION        1.0f   /* Use +1.0f or -1.0f */
#define HALL_ANGLE_DIRECTION            1.0f   /* Use -1 if measured RPM is reversed */

/* PI speed-controller settings. These values must be tuned on the motor. */
#define SPEED_PROPORTIONAL_GAIN         0.00035f
#define SPEED_INTEGRAL_GAIN             0.00150f
#define MINIMUM_DRIVE_MODULATION        0.08f
#define MAXIMUM_DRIVE_MODULATION        0.80f
#define SPEED_FILTER_COEFFICIENT        0.12f

/* Automatic startup settings ----------------------------------------------*/
/* Step 1: hold a fixed magnetic field so the rotor starts from a known angle. */
#define STARTUP_ALIGNMENT_TIME_MS       500u
#define STARTUP_ALIGNMENT_MODULATION    0.18f

/* Step 2: ramp a forced rotating field to accelerate the stationary rotor. */
#define STARTUP_INITIAL_ELECTRICAL_HZ   1.0f
#define STARTUP_MAXIMUM_ELECTRICAL_HZ   14.0f
#define STARTUP_FREQUENCY_RAMP_TIME_MS  1800u
#define STARTUP_DRIVE_MODULATION        0.22f

/* Step 3: switch to normal PI control after measured speed remains above this
 * value for STARTUP_CONFIRMATION_TIME_MS. Keep this below target speed. */
#define STARTUP_HANDOVER_SPEED_RPM      200.0f
#define STARTUP_CONFIRMATION_TIME_MS    100u

/* Stop and report a fault if the handover speed is not reached in this time. */
#define STARTUP_TIMEOUT_MS              4000u

/* Hall-sensor settings -----------------------------------------------------*/
#define MINIMUM_HALL_VECTOR_MAGNITUDE   40.0f

/* DRV5053 nominal zero-field output is ~1 V: ~1241 counts with 3.3 V ADC.
 * Replace each offset with (maximum + minimum)/2 from one electrical turn.
 * Gains correct sensor amplitude mismatch: use common_amplitude/amplitude_i.
 */
#define HALL_A_ZERO_FIELD_ADC            1241.0f
#define HALL_B_ZERO_FIELD_ADC            1241.0f
#define HALL_C_ZERO_FIELD_ADC            1241.0f
#define HALL_A_AMPLITUDE_CORRECTION      1.0f
#define HALL_B_AMPLITUDE_CORRECTION      1.0f
#define HALL_C_AMPLITUDE_CORRECTION      1.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
/* DMA continuously stores the latest Hall A, B and C ADC readings here. */
volatile uint16_t hallSensorAdcReading[3];

/* Desired mechanical speed after startup. This can be changed while running. */
volatile float requestedMotorSpeedRpm = MOTOR_TARGET_SPEED_RPM;

/* Filtered shaft speed calculated from the Hall electrical angle. */
volatile float measuredMotorSpeedRpm = 0.0f;

/* Present rotor position in electrical radians from -pi to +pi. */
volatile float measuredRotorElectricalAngleRad = 0.0f;

/* Output of the PI controller: 0.0 means no voltage and 1.0 means maximum. */
volatile float commandedPwmModulation = 0.0f;

/* Becomes 1 if Hall feedback is invalid or startup times out. */
volatile uint8_t hallSensorFaultActive = 0u;

/* Shows whether the motor is stopped, aligning, starting, running or faulted. */
volatile MotorControlState motorControlState = MOTOR_STOPPED;

static uint32_t pwmTimerPeriodCounts;            /* PWM timer period in counts */
static uint32_t speedControllerInterruptCounter; /* Divides 20 kHz to PI rate */
static uint32_t statusLedInterruptCounter;       /* Heartbeat LED timing */
static uint32_t startupInterruptCounter;         /* Alignment/startup timer */
static uint32_t handoverConfirmationCounter;     /* Confirms threshold speed */
static float previousRotorElectricalAngleRad;    /* Used to calculate speed */
static float speedControllerIntegral;            /* Stored PI integral term */
static float forcedStartupElectricalAngleRad;    /* Open-loop field angle */
static float hallToPhaseElectricalOffsetRad;     /* Learned sensor alignment */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC2_Init(void);

/* USER CODE BEGIN PFP */
void MotorControl_Initialise(void);
void MotorControl_Start(void);
void MotorControl_Stop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Restricts a value to the supplied minimum and maximum limits. */
static float LimitFloat(float value, float minimumValue, float maximumValue)
{
    if (value < minimumValue) return minimumValue;
    if (value > maximumValue) return maximumValue;
    return value;
}

/* Wraps an electrical angle to the range -pi to +pi. */
static float WrapElectricalAngle(float angleRad)
{
    while (angleRad > PI_RADIANS) angleRad -= 2.0f * PI_RADIANS;
    while (angleRad < -PI_RADIANS) angleRad += 2.0f * PI_RADIANS;
    return angleRad;
}

/* Converts a duty cycle between 0 and 1 into a timer compare value. */
static uint32_t DutyCycleToCompareValue(float dutyCycle)
{
    dutyCycle = LimitFloat(dutyCycle, 0.02f, 0.98f);
    return (uint32_t)(dutyCycle * (float)pwmTimerPeriodCounts + 0.5f);
}

/* Writes the three phase duty cycles to the existing PWM channels. */
static void SetThreePhaseDutyCycles(float phaseADutyCycle,
                                    float phaseBDutyCycle,
                                    float phaseCDutyCycle)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,
                          DutyCycleToCompareValue(phaseADutyCycle)); /* A: PB7 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,
                          DutyCycleToCompareValue(phaseBDutyCycle)); /* B: PB6 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
                          DutyCycleToCompareValue(phaseCDutyCycle)); /* C: PB5 */
}

/* Applies equal phase duties, producing zero phase-to-phase voltage. */
static void SetMotorNeutral(void)
{
    SetThreePhaseDutyCycles(0.5f, 0.5f, 0.5f);
}

/* Applies a sinusoidal three-phase voltage vector at the requested angle. */
static void ApplyVoltageVector(float voltageVectorAngleRad, float modulation)
{
    SetThreePhaseDutyCycles(
        0.5f + 0.5f * modulation * sinf(voltageVectorAngleRad),
        0.5f + 0.5f * modulation *
            sinf(voltageVectorAngleRad - 2.0f * PI_RADIANS / 3.0f),
        0.5f + 0.5f * modulation *
            sinf(voltageVectorAngleRad + 2.0f * PI_RADIANS / 3.0f));
}

/* Converts the three analogue Hall readings into one electrical rotor angle. */
static uint8_t ReadRotorElectricalAngle(float *rotorAngleRad)
{
    float hallA = ((float)hallSensorAdcReading[0] - HALL_A_ZERO_FIELD_ADC) *
                  HALL_A_AMPLITUDE_CORRECTION;
    float hallB = ((float)hallSensorAdcReading[1] - HALL_B_ZERO_FIELD_ADC) *
                  HALL_B_AMPLITUDE_CORRECTION;
    float hallC = ((float)hallSensorAdcReading[2] - HALL_C_ZERO_FIELD_ADC) *
                  HALL_C_AMPLITUDE_CORRECTION;
    float stationaryAxis = (2.0f / 3.0f) *
                           (hallA - 0.5f * hallB - 0.5f * hallC);
    float quadratureAxis = (2.0f / 3.0f) * SQRT_3_OVER_2 * (hallB - hallC);
    float hallVectorMagnitudeSquared = stationaryAxis * stationaryAxis +
                                       quadratureAxis * quadratureAxis;

    if (hallVectorMagnitudeSquared < MINIMUM_HALL_VECTOR_MAGNITUDE *
                                     MINIMUM_HALL_VECTOR_MAGNITUDE)
    {
        return 0u;
    }

    /* HALL_ANGLE_DIRECTION corrects the direction created by the physical
     * Hall A/B/C order without requiring rewiring. */
    *rotorAngleRad = HALL_ANGLE_DIRECTION *
                     atan2f(quadratureAxis, stationaryAxis);
    return 1u;
}

/* Updates filtered mechanical speed from successive Hall electrical angles. */
static uint8_t UpdateMeasuredMotorSpeed(void)
{
    float rotorAngleRad;
    float angleChangeRad;
    float instantaneousSpeedRpm;

    if (!ReadRotorElectricalAngle(&rotorAngleRad)) return 0u;

    angleChangeRad = WrapElectricalAngle(
        rotorAngleRad - previousRotorElectricalAngleRad);
    previousRotorElectricalAngleRad = rotorAngleRad;
    measuredRotorElectricalAngleRad = rotorAngleRad;

    instantaneousSpeedRpm = angleChangeRad * MOTOR_CONTROL_FREQUENCY_HZ * 60.0f /
                            (2.0f * PI_RADIANS * MOTOR_POLE_PAIRS);
    measuredMotorSpeedRpm += SPEED_FILTER_COEFFICIENT *
                             (instantaneousSpeedRpm - measuredMotorSpeedRpm);
    return 1u;
}

/* PI speed controller with conditional-integration anti-windup. */
static void RunSpeedController(void)
{
    float speedErrorRpm = requestedMotorSpeedRpm -
                          MOTOR_ROTATION_DIRECTION * measuredMotorSpeedRpm;
    float proposedIntegral = speedControllerIntegral +
        SPEED_INTEGRAL_GAIN * speedErrorRpm / SPEED_CONTROLLER_FREQUENCY_HZ;
    float unlimitedModulation = SPEED_PROPORTIONAL_GAIN * speedErrorRpm +
                                proposedIntegral;
    float limitedModulation = LimitFloat(unlimitedModulation,
                                         MINIMUM_DRIVE_MODULATION,
                                         MAXIMUM_DRIVE_MODULATION);

    if ((unlimitedModulation == limitedModulation) ||
        ((unlimitedModulation > MAXIMUM_DRIVE_MODULATION) &&
         (speedErrorRpm < 0.0f)) ||
        ((unlimitedModulation < MINIMUM_DRIVE_MODULATION) &&
         (speedErrorRpm > 0.0f)))
    {
        speedControllerIntegral = proposedIntegral;
    }

    commandedPwmModulation = limitedModulation;
}

/* Resets the variables needed for one startup attempt. */
static void BeginStartupProcedure(void)
{
    startupInterruptCounter = 0u;
    handoverConfirmationCounter = 0u;
    speedControllerInterruptCounter = 0u;
    measuredMotorSpeedRpm = 0.0f;
    commandedPwmModulation = STARTUP_ALIGNMENT_MODULATION;
    forcedStartupElectricalAngleRad = 0.0f;
    speedControllerIntegral = STARTUP_DRIVE_MODULATION;
    hallSensorFaultActive = 0u;
    motorControlState = MOTOR_ALIGNING;
}

/* Public start command. Calling this always begins with rotor alignment. */
void MotorControl_Start(void)
{
    BeginStartupProcedure();
}

/* Stops the motor and removes phase-to-phase voltage. */
void MotorControl_Stop(void)
{
    motorControlState = MOTOR_STOPPED;
    commandedPwmModulation = 0.0f;
    SetMotorNeutral();
}

/* Initialises the PWM, Hall ADC DMA and 20 kHz motor-control interrupt. */
void MotorControl_Initialise(void)
{
    pwmTimerPeriodCounts = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1u;
    SetMotorNeutral();

    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
        Error_Handler();
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)hallSensorAdcReading, 3u) != HAL_OK)
        Error_Handler();

    HAL_Delay(20);
    ReadRotorElectricalAngle(&previousRotorElectricalAngleRad);

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
    MotorControl_Start();
}

/* Aligns the stationary rotor to a known electrical position. */
static void RunRotorAlignment(void)
{
    ApplyVoltageVector(0.0f, STARTUP_ALIGNMENT_MODULATION);

    if (++startupInterruptCounter >=
        (uint32_t)(STARTUP_ALIGNMENT_TIME_MS *
                   MOTOR_CONTROL_FREQUENCY_HZ / 1000.0f))
    {
        startupInterruptCounter = 0u;
        forcedStartupElectricalAngleRad = 0.0f;
        motorControlState = MOTOR_STARTING;
    }
}

/* Startup stage: rotate the commanded magnetic field without using Hall angle
 * for commutation. This removes the need to spin the rotor by hand. As soon as
 * measured speed reaches STARTUP_HANDOVER_SPEED_RPM, control transfers to the
 * Hall-based PI controller. */
static void RunAutomaticOpenLoopStartup(void)
{
    const float frequencyRampInterrupts = STARTUP_FREQUENCY_RAMP_TIME_MS *
                                          MOTOR_CONTROL_FREQUENCY_HZ / 1000.0f;
    const uint32_t confirmationInterrupts =
        (uint32_t)(STARTUP_CONFIRMATION_TIME_MS *
                   MOTOR_CONTROL_FREQUENCY_HZ / 1000.0f);
    const uint32_t timeoutInterrupts =
        (uint32_t)(STARTUP_TIMEOUT_MS *
                   MOTOR_CONTROL_FREQUENCY_HZ / 1000.0f);
    float startupProgress = LimitFloat(
        (float)startupInterruptCounter / frequencyRampInterrupts, 0.0f, 1.0f);
    float forcedElectricalFrequencyHz = STARTUP_INITIAL_ELECTRICAL_HZ +
        startupProgress * (STARTUP_MAXIMUM_ELECTRICAL_HZ -
                           STARTUP_INITIAL_ELECTRICAL_HZ);
    float forcedAngleStepRad = MOTOR_ROTATION_DIRECTION * 2.0f * PI_RADIANS *
                               forcedElectricalFrequencyHz /
                               MOTOR_CONTROL_FREQUENCY_HZ;

    forcedStartupElectricalAngleRad = WrapElectricalAngle(
        forcedStartupElectricalAngleRad + forcedAngleStepRad);
    ApplyVoltageVector(forcedStartupElectricalAngleRad,
                       STARTUP_DRIVE_MODULATION);
    /* Hall feedback is measured during startup only to determine speed. The
     * forced angle above still controls commutation until handover. */
    if (UpdateMeasuredMotorSpeed())
    {
        float directionAdjustedSpeedRpm = MOTOR_ROTATION_DIRECTION *
                                          measuredMotorSpeedRpm;

        if (directionAdjustedSpeedRpm >= STARTUP_HANDOVER_SPEED_RPM)
        {
            /* Require the threshold to be maintained so one noisy sample does
             * not cause an early handover. */
            handoverConfirmationCounter++;
        }
        else
        {
            handoverConfirmationCounter = 0u;
        }

        if (handoverConfirmationCounter >= confirmationInterrupts)
        {
            /* Learn the physical Hall mounting offset. This makes the first
             * Hall-based voltage angle equal to the present forced angle and
             * reduces the torque step during the change of control mode. */
            hallToPhaseElectricalOffsetRad = WrapElectricalAngle(
                forcedStartupElectricalAngleRad -
                measuredRotorElectricalAngleRad -
                MOTOR_ROTATION_DIRECTION * 0.5f * PI_RADIANS);

            speedControllerIntegral = STARTUP_DRIVE_MODULATION;
            commandedPwmModulation = STARTUP_DRIVE_MODULATION;
            speedControllerInterruptCounter = 0u;
            motorControlState = MOTOR_RUNNING;
        }
    }

    startupInterruptCounter++;

    /* If the motor never reaches the handover speed, remove the drive rather
     * than remaining indefinitely in forced commutation. */
    if (startupInterruptCounter >= timeoutInterrupts &&
        motorControlState == MOTOR_STARTING)
    {
        hallSensorFaultActive = 1u;
        motorControlState = MOTOR_FAULT;
        SetMotorNeutral();
    }
}

/* Runs Hall-position commutation and PI speed regulation. */
static void RunClosedLoopControl(void)
{
    float driveElectricalAngleRad;

    if (!UpdateMeasuredMotorSpeed())
    {
        hallSensorFaultActive = 1u;
        motorControlState = MOTOR_FAULT;
        SetMotorNeutral();
        return;
    }

    hallSensorFaultActive = 0u;
    if (++speedControllerInterruptCounter >= SPEED_CONTROLLER_DIVIDER)
    {
        speedControllerInterruptCounter = 0u;
        RunSpeedController();
    }

    driveElectricalAngleRad = measuredRotorElectricalAngleRad +
        hallToPhaseElectricalOffsetRad +
        MOTOR_ROTATION_DIRECTION * 0.5f * PI_RADIANS;
    ApplyVoltageVector(driveElectricalAngleRad, commandedPwmModulation);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;

    switch (motorControlState)
    {
        case MOTOR_ALIGNING:
            RunRotorAlignment();
            break;

        case MOTOR_STARTING:
            RunAutomaticOpenLoopStartup();
            break;

        case MOTOR_RUNNING:
            RunClosedLoopControl();
            break;

        case MOTOR_STOPPED:
        case MOTOR_FAULT:
        default:
            SetMotorNeutral();
            break;
    }

    if (++statusLedInterruptCounter >=
        (uint32_t)(MOTOR_CONTROL_FREQUENCY_HZ / 2.0f))
    {
        statusLedInterruptCounter = 0u;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    }
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  MotorControl_Initialise();
  /* USER CODE END 2 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_ADC2_Init(void)
{
  /* USER CODE BEGIN ADC2_Init 0 */
  /* USER CODE END ADC2_Init 0 */
  ADC_ChannelConfTypeDef sConfig = {0};
  /* USER CODE BEGIN ADC2_Init 1 */
  /* USER CODE END ADC2_Init 1 */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
  /* USER CODE BEGIN ADC2_Init 2 */
  /* USER CODE END ADC2_Init 2 */
}

static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void)
{
  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

static void MX_TIM6_Init(void)
{
  /* USER CODE BEGIN TIM6_Init 0 */
  /* USER CODE END TIM6_Init 0 */
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  /* USER CODE BEGIN TIM6_Init 1 */
  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) Error_Handler();
  /* USER CODE BEGIN TIM6_Init 2 */
  /* USER CODE END TIM6_Init 2 */
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  SetMotorNeutral();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
