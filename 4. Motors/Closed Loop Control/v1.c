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

/* The motor passes through these states in order:
 * STOPPED -> ALIGNING -> STARTING -> RUNNING.
 * A detected sensor/startup problem moves the motor into MOTOR_FAULT.
 */
typedef enum
{
    MOTOR_STOPPED = 0,
    MOTOR_ALIGNING,
    MOTOR_STARTING,
    MOTOR_RUNNING,
    MOTOR_FAULT
} MotorState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ======================= ADJUSTABLE MOTOR SETTINGS ======================== */

#define MOTOR_POLE_PAIRS             2.0f    /* Number of rotor pole pairs; for example, a four-pole motor has two pole pairs. */
#define DEFAULT_TARGET_RPM        1000.0f    /* Mechanical running speed in RPM that the PI controller will maintain after startup. */
#define MOTOR_DIRECTION              1.0f    /* Use +1.0f for one direction or -1.0f to reverse the commanded rotation direction. */
#define TORQUE_ANGLE_RAD             1.5708f /* Electrical angle placed between rotor and stator fields in closed loop; pi/2 gives high forward torque. */

#define ALIGNMENT_MODULATION         0.15f   /* PWM waveform strength used to pull the stationary rotor into a known starting orientation. */
#define ALIGNMENT_TIME_MS             200U   /* Time the stationary alignment field is held before the open-loop rotation begins. */
#define ALIGNMENT_ADVANCE_RAD        1.5708f /* Angle ahead of the initially measured rotor position where the stationary alignment field is placed. */

#define STARTUP_INITIAL_RPM          20.0f   /* Mechanical RPM represented by the rotating field at the beginning of the startup ramp. */
#define STARTUP_RPM_PERCENTAGE        0.30f  /* Fraction of target RPM that must be measured before changing from startup to closed-loop running. */
#define STARTUP_RAMP_TIME_MS          1500U  /* Time taken for the timed rotating field to ramp from its initial RPM to the handover RPM. */
#define STARTUP_MODULATION            0.22f  /* PWM waveform strength used during startup; increase carefully if the rotor cannot overcome inertia. */
#define STARTUP_CONFIRMATION_MS        150U  /* Time measured RPM must continuously exceed the handover threshold before closed-loop control begins. */
#define STARTUP_TIMEOUT_MS            3500U  /* Maximum allowed startup duration before the controller stops the motor and reports a startup fault. */

#define SPEED_KP                   0.00035f  /* Proportional gain: controls the immediate PWM-strength change produced by the present RPM error. */
#define SPEED_KI                   0.00150f  /* Integral gain per second: removes lasting RPM error by accumulating the error over time. */
#define MIN_RUNNING_MODULATION        0.08f  /* Lowest PWM waveform strength allowed while running so the motor does not lose commutation. */
#define MAX_RUNNING_MODULATION        0.80f  /* Highest PWM waveform strength allowed; reduce this if current, heating, or voltage is excessive. */
#define SPEED_FILTER_ALPHA            0.12f  /* RPM low-pass filter weight from 0 to 1; smaller is smoother but responds more slowly. */


/* 
    Offset ADC ≈ (Hall offset voltage / ADC reference voltage) × 4095
    Offset ADC ≈ (1.0 V / 3.3 V) × 4095 ≈ 1241
    Offset ADC ≈ (0.7 V / 3.3 V) × 4095 ≈ 870
*/
#define HALL_A_OFFSET               870.0f   /* Hall A zero-field ADC value, measured as (maximum ADC + minimum ADC) / 2 over one electrical turn. */
#define HALL_B_OFFSET               870.0f   /* Hall B zero-field ADC value, measured as (maximum ADC + minimum ADC) / 2 over one electrical turn. */
#define HALL_C_OFFSET               870.0f   /* Hall C zero-field ADC value, measured as (maximum ADC + minimum ADC) / 2 over one electrical turn. */
#define HALL_A_GAIN                   1.0f   /* Hall A amplitude correction, calculated as chosen common amplitude divided by Hall A amplitude. */
#define HALL_B_GAIN                   1.0f   /* Hall B amplitude correction, calculated as chosen common amplitude divided by Hall B amplitude. */
#define HALL_C_GAIN                   1.0f   /* Hall C amplitude correction, calculated as chosen common amplitude divided by Hall C amplitude. */
#define HALL_MINIMUM_MAGNITUDE       40.0f   /* Minimum valid Hall-vector magnitude in ADC counts; readings below this are treated as a sensor fault. */
#define HALL_FAULT_TIME_MS             20U   /* Time Hall readings may remain invalid before the motor enters MOTOR_FAULT. */








/*
 * ============================================================================
 *                    CUBEMX CONFIGURATION REQUIREMENTS
 * ============================================================================
 *
 * This project uses a 24 MHz external oscillator (HSE) and runs the
 * STM32G431CBT6 at 170 MHz.
 *
 *
 * 1. SYSTEM CLOCK CONFIGURATION
 * --------------------------------
 * In CubeMX, open "Clock Configuration" and configure:
 *
 *     Clock source  = HSE
 *     HSE frequency = 24 MHz
 *     PLLM          = /6
 *     PLLN          = x85
 *     PLLR          = /2
 *
 * Clock calculation:
 *
 *     SYSCLK = 24 MHz / 6 x 85 / 2
 *            = 170 MHz
 *
 * Confirm that CubeMX shows:
 *
 *     SYSCLK = 170 MHz
 *     HCLK   = 170 MHz
 *     PCLK1  = 170 MHz
 *     PCLK2  = 170 MHz
 *
 *
 * 2. TIM6 MOTOR-CONTROL INTERRUPT
 * --------------------------------
 * TIM6 runs the complete motor controller at 20 kHz.
 *
 * In CubeMX, configure TIM6 as:
 *
 *     Clock source      = Internal Clock
 *     Prescaler         = 169
 *     Counter period    = 49
 *     Auto-reload       = Disabled
 *
 * Timer calculation:
 *
 *     TIM6 frequency = 170 MHz / (169 + 1) / (49 + 1)
 *                    = 20,000 Hz
 *
 * Under TIM6 "NVIC Settings", enable:
 *
 *     TIM6 global interrupt / TIM6_DAC interrupt
 *
 * If the TIM6 frequency is changed in CubeMX, CONTROL_ISR_HZ below must be
 * changed to the new frequency.
 *
 *
 * 3. THREE MOTOR PWM OUTPUTS
 * --------------------------------
 * Configure these PWM pins:
 *
 *     PB7 = TIM4 Channel 2 = Motor phase A
 *     PB6 = TIM4 Channel 1 = Motor phase B
 *     PB5 = TIM3 Channel 2 = Motor phase C
 *
 * Configure all three channels as:
 *
 *     Mode               = PWM Generation
 *     PWM mode           = PWM Mode 1
 *     Prescaler          = 0
 *     Counter period     = 8499
 *     Pulse              = 0
 *     Polarity           = High
 *     Fast mode          = Disabled
 *
 * PWM calculation:
 *
 *     PWM frequency = 170 MHz / (0 + 1) / (8499 + 1)
 *                   = 20,000 Hz
 *
 * TIM3 and TIM4 must always have the same prescaler and counter period.
 *
 *
 * 4. ANALOG HALL SENSOR ADC
 * --------------------------------
 * Configure ADC2 with three regular conversion ranks:
 *
 *     Rank 1 = Hall sensor A ADC channel
 *     Rank 2 = Hall sensor B ADC channel
 *     Rank 3 = Hall sensor C ADC channel
 *
 * The current code uses:
 *
 *     Rank 1 = ADC2 Channel 3
 *     Rank 2 = ADC2 Channel 4
 *     Rank 3 = ADC2 Channel 5
 *
 * These channels must match the actual Hall sensor pins on the PCB.
 *
 * Configure ADC2 as:
 *
 *     Resolution               = 12 bits
 *     Scan conversion mode     = Enabled
 *     Number of conversions    = 3
 *     Continuous conversion    = Enabled
 *     External trigger         = Software Start
 *     Data alignment           = Right
 *     DMA continuous requests  = Enabled
 *     Overrun                  = Data Overwritten
 *     Sampling time            = 47.5 cycles
 *     Input mode               = Single-ended
 *     Clock prescaler          = Synchronous PCLK divided by 4
 *
 *
 * 5. ADC DMA
 * --------------------------------
 * Add a DMA request to ADC2 and configure:
 *
 *     Direction                  = Peripheral to Memory
 *     Mode                       = Circular
 *     Peripheral increment       = Disabled
 *     Memory increment           = Enabled
 *     Peripheral data alignment  = Half Word
 *     Memory data alignment      = Half Word
 *     Priority                   = High
 *
 * Circular mode must be enabled so DMA continuously updates:
 *
 *     hall_adc[0] = Hall A
 *     hall_adc[1] = Hall B
 *     hall_adc[2] = Hall C
 *
 * The DMA interrupt is not required by this motor-control code.
 *
 *
 * 6. VALUES TO CHANGE WHEN CONFIGURATION CHANGES
 * --------------------------------
 *
 * If the TIM6 frequency changes:
 *
 *     Change CONTROL_ISR_HZ to the real TIM6 frequency.
 *
 * If the motor pole count changes:
 *
 *     Change MOTOR_POLE_PAIRS.
 *
 * If the desired speed changes:
 *
 *     Change DEFAULT_TARGET_RPM or call Motor_SetTargetRPM().
 *
 * If the motor turns in the wrong direction:
 *
 *     Change MOTOR_DIRECTION from +1.0f to -1.0f.
 *
 * If the Hall sensor ADC channels or rank order change:
 *
 *     Ensure hall_adc[0], hall_adc[1] and hall_adc[2] still correspond to
 *     Hall A, Hall B and Hall C in that exact order.
 *
 * Do not manually change SPEED_CONTROL_HZ or TICKS_PER_MS. They are calculated
 * automatically from CONTROL_ISR_HZ and SPEED_UPDATE_DIVIDER.
 *
 * ============================================================================
 */










/* ======================= FIXED CONTROL CONSTANTS ========================== */

#define PI_F                 3.14159265358979323846f
#define TWO_PI_F             (2.0f * PI_F)
#define SQRT3_OVER_2_F       0.8660254037844386f
#define CONTROL_ISR_HZ       20000.0f
#define SPEED_UPDATE_DIVIDER 20U
#define SPEED_CONTROL_HZ     (CONTROL_ISR_HZ / (float)SPEED_UPDATE_DIVIDER)
#define TICKS_PER_MS         ((uint32_t)(CONTROL_ISR_HZ / 1000.0f))

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

/* Latest Hall A, B and C ADC samples; DMA refreshes these continuously. */
volatile uint16_t hall_adc[3] = {0U, 0U, 0U};

/* Current requested mechanical speed; may be changed with Motor_SetTargetRPM(). */
volatile float target_rpm = DEFAULT_TARGET_RPM;

/* Filtered mechanical rotor speed calculated from Hall-angle movement. */
volatile float measured_rpm = 0.0f;

/* Latest electrical rotor angle calculated from the three analog Hall signals. */
volatile float hall_electrical_angle_rad = 0.0f;

/* Present three-phase PWM waveform strength, ranging approximately from 0.0 to 1.0. */
volatile float pwm_modulation = 0.0f;

/* Set when the Hall vector remains invalid for longer than the allowed fault time. */
volatile uint8_t hall_sensor_fault = 0U;

/* Current stage of the motor startup/running state machine. */
volatile MotorState_t motor_state = MOTOR_STOPPED;

/* Internal motor-control variables. */
static uint32_t pwm_period_counts = 0U;
static uint32_t state_tick_count = 0U;
static uint32_t startup_confirm_tick_count = 0U;
static uint32_t invalid_hall_tick_count = 0U;
static uint32_t speed_update_count = 0U;
static uint32_t heartbeat_count = 0U;

static float previous_hall_angle_rad = 0.0f;
static float accumulated_hall_angle_rad = 0.0f;
static float alignment_field_angle_rad = 0.0f;
static float startup_field_angle_rad = 0.0f;
static float speed_integral = 0.0f;

static uint8_t hall_angle_initialized = 0U;

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

void Motor_Control_Init(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetTargetRPM(float new_target_rpm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Limit a value between a lower and upper boundary.
  */
static float Limit_Value(float value, float lower_limit, float upper_limit)
{
    if (value < lower_limit)
    {
        return lower_limit;
    }

    if (value > upper_limit)
    {
        return upper_limit;
    }

    return value;
}

/**
  * @brief  Wrap an angle into the range -pi to +pi.
  *
  * This prevents the Hall angle changing from +pi to -pi from appearing as
  * one very large movement and producing an incorrect RPM measurement.
  */
static float Wrap_Angle(float angle_rad)
{
    while (angle_rad > PI_F)
    {
        angle_rad -= TWO_PI_F;
    }

    while (angle_rad < -PI_F)
    {
        angle_rad += TWO_PI_F;
    }

    return angle_rad;
}

/**
  * @brief  Convert a duty-cycle fraction into a timer compare value.
  *
  * The duty cycle is limited to 2%-98% to avoid commanding exactly 0% or 100%.
  */
static uint32_t Duty_To_Compare(float duty_cycle)
{
    duty_cycle = Limit_Value(duty_cycle, 0.02f, 0.98f);

    return (uint32_t)(duty_cycle * (float)pwm_period_counts + 0.5f);
}

/**
  * @brief  Write the three requested duty cycles to the PWM outputs.
  *
  * Phase A = PB7 = TIM4 Channel 2
  * Phase B = PB6 = TIM4 Channel 1
  * Phase C = PB5 = TIM3 Channel 2
  */
static void Set_Phase_Duties(float phase_a_duty,
                             float phase_b_duty,
                             float phase_c_duty)
{
    __HAL_TIM_SET_COMPARE(&htim4,
                          TIM_CHANNEL_2,
                          Duty_To_Compare(phase_a_duty));

    __HAL_TIM_SET_COMPARE(&htim4,
                          TIM_CHANNEL_1,
                          Duty_To_Compare(phase_b_duty));

    __HAL_TIM_SET_COMPARE(&htim3,
                          TIM_CHANNEL_2,
                          Duty_To_Compare(phase_c_duty));
}

/**
  * @brief  Set every motor phase to the same 50% duty cycle.
  *
  * Identical phase duties produce zero ideal line-to-line motor voltage.
  * The Hall sensors and control interrupt remain active while stopped.
  */
static void Set_Neutral_Output(void)
{
    Set_Phase_Duties(0.5f, 0.5f, 0.5f);
}

/**
  * @brief  Produce the three sinusoidal phase PWM signals.
  *
  * The three phase commands are separated by 120 electrical degrees.
  * The angle controls the orientation of the stator magnetic field.
  * The modulation controls the strength of that magnetic field.
  */
static void Apply_Rotating_Field(float field_angle_rad, float modulation)
{
    float phase_a_duty;
    float phase_b_duty;
    float phase_c_duty;

    phase_a_duty = 0.5f
                 + 0.5f * modulation * sinf(field_angle_rad);

    phase_b_duty = 0.5f
                 + 0.5f * modulation
                 * sinf(field_angle_rad - (TWO_PI_F / 3.0f));

    phase_c_duty = 0.5f
                 + 0.5f * modulation
                 * sinf(field_angle_rad + (TWO_PI_F / 3.0f));

    Set_Phase_Duties(phase_a_duty,
                     phase_b_duty,
                     phase_c_duty);
}

/**
  * @brief  Calculate rotor electrical angle from the analog Hall sensors.
  *
  * The Hall offsets are removed first. The three Hall values are then converted
  * into a two-axis vector using a Clarke transform. atan2f() gives the angle of
  * this vector, which represents the electrical rotor angle.
  *
  * @param  calculated_angle_rad Location where the calculated angle is stored.
  * @retval 1 if the Hall signals are valid, otherwise 0.
  */
static uint8_t Calculate_Hall_Angle(float *calculated_angle_rad)
{
    float hall_a;
    float hall_b;
    float hall_c;
    float hall_alpha;
    float hall_beta;
    float hall_magnitude_squared;

    hall_a = ((float)hall_adc[0] - HALL_A_OFFSET) * HALL_A_GAIN;
    hall_b = ((float)hall_adc[1] - HALL_B_OFFSET) * HALL_B_GAIN;
    hall_c = ((float)hall_adc[2] - HALL_C_OFFSET) * HALL_C_GAIN;

    hall_alpha = (2.0f / 3.0f)
               * (hall_a - 0.5f * hall_b - 0.5f * hall_c);

    hall_beta = (2.0f / 3.0f)
              * SQRT3_OVER_2_F
              * (hall_b - hall_c);

    hall_magnitude_squared = hall_alpha * hall_alpha
                           + hall_beta * hall_beta;

    /*
     * A very small Hall vector normally means that the sensors are disconnected,
     * badly calibrated, or returning invalid values.
     */
    if (hall_magnitude_squared
        < HALL_MINIMUM_MAGNITUDE * HALL_MINIMUM_MAGNITUDE)
    {
        return 0U;
    }

    *calculated_angle_rad = atan2f(hall_beta, hall_alpha);

    return 1U;
}

/**
  * @brief  Update Hall position and measured motor RPM.
  *
  * This function runs on every control interrupt in every motor state.
  * Hall angle movements are accumulated for several samples before RPM is
  * calculated, reducing the effect of ADC noise.
  *
  * @retval 1 if the current Hall signals are valid, otherwise 0.
  */
static uint8_t Update_Hall_Position_And_Speed(void)
{
    float new_hall_angle_rad;
    float angle_change_rad;

    /*
     * The DMA updates hall_adc[] automatically, so this function only processes
     * the most recent samples.
     */
    if (!Calculate_Hall_Angle(&new_hall_angle_rad))
    {
        if (invalid_hall_tick_count
            < (HALL_FAULT_TIME_MS * TICKS_PER_MS))
        {
            invalid_hall_tick_count++;
        }

        return 0U;
    }

    invalid_hall_tick_count = 0U;
    hall_sensor_fault = 0U;
    hall_electrical_angle_rad = new_hall_angle_rad;

    /*
     * The first valid sample establishes the previous angle. Two angle samples
     * are needed before movement and speed can be calculated.
     */
    if (!hall_angle_initialized)
    {
        previous_hall_angle_rad = new_hall_angle_rad;
        hall_angle_initialized = 1U;

        return 1U;
    }

    /*
     * Calculate the movement since the previous Hall reading and add it to the
     * angle accumulated during the current RPM measurement window.
     */
    angle_change_rad = Wrap_Angle(new_hall_angle_rad
                                - previous_hall_angle_rad);

    previous_hall_angle_rad = new_hall_angle_rad;
    accumulated_hall_angle_rad += angle_change_rad;

    /*
     * Calculate RPM at SPEED_CONTROL_HZ rather than on every 20 kHz interrupt.
     */
    if (++speed_update_count >= SPEED_UPDATE_DIVIDER)
    {
        float raw_rpm;

        speed_update_count = 0U;

        /*
         * Hall angle is electrical angle. Dividing by MOTOR_POLE_PAIRS converts
         * electrical rotation into mechanical rotation.
         */
        raw_rpm = accumulated_hall_angle_rad
                * SPEED_CONTROL_HZ
                * 60.0f
                / (TWO_PI_F * MOTOR_POLE_PAIRS);

        accumulated_hall_angle_rad = 0.0f;

        /*
         * Filter the RPM measurement so Hall and ADC noise do not cause rapid
         * changes to the PI output.
         */
        measured_rpm += SPEED_FILTER_ALPHA
                      * (raw_rpm - measured_rpm);
    }

    return 1U;
}

/**
  * @brief  Run one iteration of the speed PI controller.
  *
  * Hall position determines the phase waveform angle. The PI controller only
  * adjusts waveform strength so that measured RPM follows target RPM.
  */
static void Run_Speed_PI_Controller(void)
{
    float directed_measured_rpm;
    float rpm_error;
    float proposed_integral;
    float unlimited_output;
    float limited_output;

    /*
     * Multiplying by MOTOR_DIRECTION makes correct-direction RPM positive,
     * regardless of which commanded direction is selected.
     */
    directed_measured_rpm = MOTOR_DIRECTION * measured_rpm;

    rpm_error = target_rpm - directed_measured_rpm;

    /*
     * Integrate error using the fixed PI update period.
     * Dividing by SPEED_CONTROL_HZ is equivalent to multiplying by time step.
     */
    proposed_integral = speed_integral
                      + SPEED_KI * rpm_error / SPEED_CONTROL_HZ;

    unlimited_output = SPEED_KP * rpm_error
                     + proposed_integral;

    limited_output = Limit_Value(unlimited_output,
                                 MIN_RUNNING_MODULATION,
                                 MAX_RUNNING_MODULATION);

    /*
     * Anti-windup:
     * Update the integral normally when the controller is not saturated.
     * If saturated, only integrate when the error moves the output away from
     * the active limit.
     */
    if ((unlimited_output == limited_output)
        || ((unlimited_output > MAX_RUNNING_MODULATION)
            && (rpm_error < 0.0f))
        || ((unlimited_output < MIN_RUNNING_MODULATION)
            && (rpm_error > 0.0f)))
    {
        speed_integral = proposed_integral;
    }

    pwm_modulation = limited_output;
}

/**
  * @brief  Initialise PWM generation, Hall ADC DMA and motor-control timing.
  *
  * Call this once after all CubeMX MX_*_Init() functions.
  */
void Motor_Control_Init(void)
{
    pwm_period_counts = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;

    /*
     * TIM3 and TIM4 must have the same PWM period because the same duty-to-count
     * conversion is used for all three motor phases.
     */
    if ((__HAL_TIM_GET_AUTORELOAD(&htim3) + 1U)
        != pwm_period_counts)
    {
        Error_Handler();
    }

    Set_Neutral_Output();

    /* Start the three PWM outputs. */
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Calibrate ADC2 before beginning Hall sensor conversions. */
    if (HAL_ADCEx_Calibration_Start(&hadc2,
                                    ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * Start continuous circular DMA.
     * The DMA continually places Hall A, B and C into hall_adc[0..2].
     */
    if (HAL_ADC_Start_DMA(&hadc2,
                          (uint32_t *)hall_adc,
                          3U) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * Allow the ADC and analog Hall sensor readings to settle before using them.
     */
    HAL_Delay(20U);

    /*
     * Capture the first rotor angle before Motor_Start() begins alignment.
     */
    if (!Calculate_Hall_Angle(&previous_hall_angle_rad))
    {
        hall_sensor_fault = 1U;
        motor_state = MOTOR_FAULT;
        Error_Handler();
    }

    hall_electrical_angle_rad = previous_hall_angle_rad;
    hall_angle_initialized = 1U;
    motor_state = MOTOR_STOPPED;

    /*
     * Start the 20 kHz motor-control interrupt. Hall sensors will now be
     * processed continuously, including while the motor is stopped.
     */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  Begin automatic alignment and startup.
  *
  * Once the startup RPM threshold has been reached consistently, the motor
  * automatically enters Hall-based closed-loop PI control.
  */
void Motor_Start(void)
{
    /*
     * Ignore another start request if the motor is already aligning,
     * starting, or running.
     */
    if ((motor_state != MOTOR_STOPPED)
        && (motor_state != MOTOR_FAULT))
    {
        return;
    }

    /*
     * A valid Hall angle is required so alignment begins relative to the
     * actual rotor position.
     */
    if (!hall_angle_initialized || hall_sensor_fault)
    {
        motor_state = MOTOR_FAULT;
        Set_Neutral_Output();

        return;
    }

    /* Reset state timing and PI values before every startup. */
    state_tick_count = 0U;
    startup_confirm_tick_count = 0U;
    speed_integral = 0.0f;
    measured_rpm = 0.0f;
    accumulated_hall_angle_rad = 0.0f;
    speed_update_count = 0U;
    pwm_modulation = ALIGNMENT_MODULATION;

    /*
     * Place a stationary stator field ahead of the current rotor position.
     * The rotor may make a small movement and settle into this orientation.
     */
    alignment_field_angle_rad = hall_electrical_angle_rad
                              + MOTOR_DIRECTION
                              * ALIGNMENT_ADVANCE_RAD;

    alignment_field_angle_rad = Wrap_Angle(alignment_field_angle_rad);
    startup_field_angle_rad = alignment_field_angle_rad;

    motor_state = MOTOR_ALIGNING;
}

/**
  * @brief  Stop the motor and reset the controller.
  *
  * Hall sensor acquisition continues after the motor is stopped.
  */
void Motor_Stop(void)
{
    motor_state = MOTOR_STOPPED;
    state_tick_count = 0U;
    startup_confirm_tick_count = 0U;
    speed_integral = 0.0f;
    pwm_modulation = 0.0f;

    Set_Neutral_Output();
}

/**
  * @brief  Change the target running speed.
  *
  * @param  new_target_rpm New positive mechanical speed in RPM.
  */
void Motor_SetTargetRPM(float new_target_rpm)
{
    if (new_target_rpm > 0.0f)
    {
        target_rpm = new_target_rpm;
    }
}

/**
  * @brief  Timer interrupt containing the motor state machine.
  *
  * This callback executes at CONTROL_ISR_HZ using TIM6.
  * Hall sensors are processed before the motor-state logic on every interrupt.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t hall_is_valid;

    if (htim->Instance != TIM6)
    {
        return;
    }

    /*
     * Always process the latest Hall sensor values first, regardless of whether
     * the motor is stopped, aligning, starting, running, or faulted.
     */
    hall_is_valid = Update_Hall_Position_And_Speed();

    /*
     * One invalid reading does not immediately stop the motor. A fault is only
     * declared if the signal remains invalid for HALL_FAULT_TIME_MS.
     */
    if (!hall_is_valid
        && (invalid_hall_tick_count
            >= (HALL_FAULT_TIME_MS * TICKS_PER_MS)))
    {
        hall_sensor_fault = 1U;
        motor_state = MOTOR_FAULT;
    }

    switch (motor_state)
    {
        case MOTOR_STOPPED:
        {
            /*
             * Hall sensing remains active, but equal phase duties produce no
             * ideal line-to-line motor voltage.
             */
            Set_Neutral_Output();
            break;
        }

        case MOTOR_ALIGNING:
        {
            /*
             * Hold a stationary stator field. The rotor moves only enough to
             * settle into a known starting orientation.
             */
            Apply_Rotating_Field(alignment_field_angle_rad,
                                 ALIGNMENT_MODULATION);

            state_tick_count++;

            /*
             * After the alignment time, begin the timed rotating startup field.
             */
            if (state_tick_count
                >= (ALIGNMENT_TIME_MS * TICKS_PER_MS))
            {
                state_tick_count = 0U;
                startup_confirm_tick_count = 0U;
                startup_field_angle_rad = alignment_field_angle_rad;
                pwm_modulation = STARTUP_MODULATION;

                motor_state = MOTOR_STARTING;
            }

            break;
        }

        case MOTOR_STARTING:
        {
            float startup_progress;
            float startup_handover_rpm;
            float commanded_startup_rpm;
            float electrical_speed_rad_per_second;
            float directed_measured_rpm;

            state_tick_count++;

            /*
             * Calculate how far the controller is through the startup ramp.
             * The result remains at 1.0 after the ramp time has elapsed.
             */
            startup_progress = (float)state_tick_count
                             / (float)(STARTUP_RAMP_TIME_MS
                             * TICKS_PER_MS);

            startup_progress = Limit_Value(startup_progress,
                                           0.0f,
                                           1.0f);

            /*
             * The handover RPM is a percentage of the selected running RPM.
             * Example: 30% of a 1000 RPM target gives a 300 RPM handover.
             */
            startup_handover_rpm = target_rpm
                                 * STARTUP_RPM_PERCENTAGE;

            /*
             * Gradually increase the commanded startup speed.
             */
            commanded_startup_rpm = STARTUP_INITIAL_RPM
                                  + startup_progress
                                  * (startup_handover_rpm
                                  - STARTUP_INITIAL_RPM);

            /*
             * Convert mechanical RPM into electrical radians per second.
             */
            electrical_speed_rad_per_second =
                    commanded_startup_rpm
                  * MOTOR_POLE_PAIRS
                  * TWO_PI_F
                  / 60.0f;

            /*
             * Move the stator field forward using timing. This produces
             * rotation even before Hall feedback is used for commutation.
             */
            startup_field_angle_rad +=
                    MOTOR_DIRECTION
                  * electrical_speed_rad_per_second
                  / CONTROL_ISR_HZ;

            startup_field_angle_rad =
                    Wrap_Angle(startup_field_angle_rad);

            Apply_Rotating_Field(startup_field_angle_rad,
                                 STARTUP_MODULATION);

            /*
             * Convert measured speed into a positive number when the motor
             * is rotating in the selected direction.
             */
            directed_measured_rpm = MOTOR_DIRECTION * measured_rpm;

            /*
             * Require the measured motor speed to stay above the handover
             * threshold continuously before enabling closed-loop control.
             */
            if (directed_measured_rpm >= startup_handover_rpm)
            {
                startup_confirm_tick_count++;
            }
            else
            {
                startup_confirm_tick_count = 0U;
            }

            /*
             * Transfer permanently into Hall-based closed-loop PI control
             * once the startup RPM has been confirmed.
             */
            if (startup_confirm_tick_count
                >= (STARTUP_CONFIRMATION_MS * TICKS_PER_MS))
            {
                /*
                 * Begin the PI integral near the startup modulation so the
                 * PWM strength does not suddenly jump during handover.
                 */
                speed_integral =
                        Limit_Value(STARTUP_MODULATION,
                                    MIN_RUNNING_MODULATION,
                                    MAX_RUNNING_MODULATION);

                pwm_modulation = speed_integral;
                speed_update_count = 0U;

                motor_state = MOTOR_RUNNING;
            }
            /*
             * If the rotor cannot reach the handover RPM before timeout,
             * stop driving it and enter the fault state.
             */
            else if (state_tick_count
                     >= (STARTUP_TIMEOUT_MS * TICKS_PER_MS))
            {
                motor_state = MOTOR_FAULT;
            }

            break;
        }

        case MOTOR_RUNNING:
        {
            /*
             * Run the speed PI controller at SPEED_CONTROL_HZ.
             * Hall rotor angle is still processed at the full interrupt rate.
             */
            if (speed_update_count == 0U)
            {
                Run_Speed_PI_Controller();
            }

            /*
             * Hall position determines the stator field angle.
             * PI output determines how strongly the motor is driven.
             */
            Apply_Rotating_Field(
                    hall_electrical_angle_rad
                  + MOTOR_DIRECTION * TORQUE_ANGLE_RAD,
                    pwm_modulation);

            break;
        }

        case MOTOR_FAULT:
        default:
        {
            /*
             * Remain de-energised after a Hall or startup fault.
             * Hall sensor acquisition continues for debugging.
             */
            pwm_modulation = 0.0f;
            Set_Neutral_Output();

            break;
        }
    }

    /*
     * Toggle the status LED every 0.5 seconds to indicate that the control
     * interrupt is still executing.
     */
    if (++heartbeat_count
        >= (uint32_t)(CONTROL_ISR_HZ / 2.0f))
    {
        heartbeat_count = 0U;

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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

  /*
   * Initialise continuous Hall sensing and the three PWM outputs.
   */
  Motor_Control_Init();

  /*
   * Select the required running speed.
   * You can replace DEFAULT_TARGET_RPM with a direct value such as 1000.0f.
   */
  Motor_SetTargetRPM(DEFAULT_TARGET_RPM);

  /*
   * Automatically begin alignment and startup when the STM32 powers on.
   */
  Motor_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /*
     * Motor control happens inside the TIM6 interrupt.
     * Other non-time-critical application code can be placed here.
     */

    /* USER CODE BEGIN 3 */
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

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType =
          RCC_CLOCKTYPE_HCLK
        | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1
        | RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                          FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
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

  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  /* Hall sensor A. */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;

  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Hall sensor B. */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;

  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Hall sensor C. */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;

  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
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

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim3,
                                            &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3,
                                &sConfigOC,
                                TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
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

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim4,
                                            &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim4,
                                &sConfigOC,
                                TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4,
                                &sConfigOC,
                                TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */

  /*
   * TIM6 runs at:
   *
   * 170 MHz / (169 + 1) / (49 + 1) = 20 kHz
   */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim6,
                                            &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA,
                    GPIO_PIN_9,
                    GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_3,
                    GPIO_PIN_RESET);

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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  __disable_irq();

  /*
   * Only access the PWM registers if timer initialisation was completed.
   */
  if ((htim3.Instance == TIM3)
      && (htim4.Instance == TIM4)
      && (pwm_period_counts > 0U))
  {
    Set_Neutral_Output();
  }

  while (1)
  {
  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and source line number.
  * @param  file Pointer to the source file name.
  * @param  line Error line number.
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */