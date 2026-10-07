/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Hall Start / Maximum Torque Pump Motor Controller
  ******************************************************************************
  *
  * STM32G431CBT6 + DRV8316CT
  *
  * NEW PCB PINOUT
  * --------------------------------------------------------------------------
  *
  * PWM / DRV8316 3xPWM:
  *   Phase A / INHA = PB7 = TIM4_CH2
  *   Phase B / INHB = PB6 = TIM4_CH1
  *   Phase C / INHC = PB5 = TIM3_CH2
  *
  * Analog Hall:
  *   Hall A = Hall Sensor 1A = PA4 = ADC2_IN17
  *   Hall B = Hall Sensor 2A = PA3 = ADC1_IN4
  *   Hall C = Hall Sensor 3A = PA5 = ADC2_IN13
  *
  * Driver:
  *   DRVOFF = PA9
  *   nFAULT = PA10
  *
  * Other:
  *   CAN STBY   = PB0
  *   Status LED = PB3
  *
  * Rotor:
  *   4 poles = 2 pole pairs
  *
  * STARTUP:
  *
  *   1. Driver disabled while peripherals start.
  *   2. Start neutral PWM.
  *   3. Enable DRV8316 by pulling DRVOFF LOW.
  *   4. Hold stator field at 0 electrical degrees.
  *   5. Rotor aligns to this field.
  *   6. Read analog Hall electrical angle.
  *   7. Calculate Hall-to-stator electrical offset.
  *   8. Enter Hall closed-loop directly.
  *   9. Apply +90 degree torque-producing field.
  *  10. Ramp modulation to 100%.
  *  11. Maximum torque until target RPM.
  *  12. Run at full modulation for MAX_RUN_MS (5 s) and record
  *      the highest filtered speed reached (max_run_rpm).
  *  13. Set the sinusoid from that:
  *        high = 1.00 * max_run_rpm
  *        low  = 0.50 * max_run_rpm
  *  14. Speed reference oscillates sinusoidally between low and
  *      high at PULSE_BPM, starting at the peak.
  *
  * IMPORTANT DRV8316 HARDWARE REQUIREMENTS:
  *
  *   - DRV8316 must be configured for 3xPWM mode.
  *   - INLA, INLB and INLC must be pulled HIGH.
  *   - nSLEEP must be HIGH.
  *   - nFAULT must have an external pull-up.
  *   - DRVOFF LOW enables the MOSFET outputs.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdint.h>

/* USER CODE END Includes */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ========================================================================== */
/* MATH                                                                       */
/* ========================================================================== */

#define PI_F                        3.14159265358979323846f
#define TWO_PI_F                    6.28318530717958647692f
#define SQRT3_2                     0.86602540378443864676f

#define SVPWM_GAIN                  1.154700538f


/* ========================================================================== */
/* PCB / DRIVER                                                               */
/* ========================================================================== */

#define DRVOFF_PORT                 GPIOA
#define DRVOFF_PIN                  GPIO_PIN_9

#define NFAULT_PORT                 GPIOA
#define NFAULT_PIN                  GPIO_PIN_10

#define CAN_STBY_PORT               GPIOB
#define CAN_STBY_PIN                GPIO_PIN_0

#define STATUS_LED_PORT             GPIOB
#define STATUS_LED_PIN              GPIO_PIN_3


/*
 * Debounce the DRV8316 nFAULT signal.
 *
 * Control loop = 5 kHz.
 * 25 samples ~= 5 ms.
 */
#define DRIVER_FAULT_LIMIT          25u


/* ========================================================================== */
/* MOTOR                                                                      */
/* ========================================================================== */

#define POLE_PAIRS                  2.0f

/*
 * +1.0f = forward
 * -1.0f = reverse
 */
#define DIRECTION                   1.0f


/*
 * Change to -1.0f ONLY if the Hall electrical angle itself
 * runs backwards relative to mechanical rotation.
 */
#define HALL_DIRECTION              1.0f


/* ========================================================================== */
/* SPEED                                                                      */
/* ========================================================================== */

#define TARGET_RPM                  3000.0f

#define FULL_TORQUE_BAND_RPM        30.0f


/* ========================================================================== */
/* PULSATILITY                                                                */
/* ========================================================================== */

/*
 * Sinusoidal speed reference, one "heartbeat" per period:
 *
 *   rpm(t) = mean + amp * sin(2*pi * BPM/60 * t)
 *
 *   mean = (LOW + HIGH) / 2
 *   amp  = (HIGH - LOW) / 2
 *
 *        HIGH    .-.           .-.
 *               /   \         /   \
 *        MEAN -/-----\-------/-----\---
 *                     \     /       \
 *        LOW           '-'           '-
 *              |<--- 1 beat --->|
 *
 * LOW and HIGH below are only placeholders. They are overwritten
 * after the 5 s max-speed run with:
 *
 *   HIGH = PULSE_HIGH_FRACTION * max_run_rpm   (1.00)
 *   LOW  = PULSE_LOW_FRACTION  * max_run_rpm   (0.50)
 *
 * The sine starts at its peak (phase 0.25) right after the max run.
 *
 * All of these are copied into volatile variables below so
 * they can be changed live in the debugger.
 */
#define PULSE_ENABLE_DEFAULT        1u

#define PULSE_BPM                   12.0f   /* 0.2 Hz = 5 s per cycle */

#define PULSE_LOW_RPM               2500.0f

#define PULSE_HIGH_RPM              3500.0f

#define PULSE_START_DELAY_MS        3000u


/*
 * Safety clamps on live-tunable values.
 */
#define PULSE_BPM_MIN               1.0f    /* 60 s per cycle */
#define PULSE_BPM_MAX               150.0f

#define PULSE_RPM_MIN               0.0f
#define PULSE_RPM_MAX               20000.0f


/* ========================================================================== */
/* MAX SPEED RUN (sets the sinusoid limits)                                   */
/* ========================================================================== */

/*
 * After the startup torque ramp, hold full modulation for this long
 * and record the highest measured speed.
 */
#define MAX_RUN_MS                  5000u


/*
 * Sinusoid limits as a fraction of the measured max speed.
 */
#define PULSE_HIGH_FRACTION         1.00f

#define PULSE_LOW_FRACTION          0.50f


/*
 * If the motor never gets above this during the max run,
 * something is wrong. Fault out instead of pulsing.
 */
#define MAX_RUN_MIN_VALID_RPM       300.0f



/* ========================================================================== */
/* CONTROL TIMING                                                             */
/* ========================================================================== */

/*
 * TIM6 interrupt:
 *
 * 170 MHz / (170 * 50)
 * = 20 kHz
 */
#define ISR_HZ                      20000.0f


/*
 * Hall/commutation calculation:
 *
 * 20 kHz / 4
 * = 5 kHz
 */
#define CONTROL_DIV                 4u

#define CONTROL_HZ                  (ISR_HZ / (float)CONTROL_DIV)


/*
 * Speed update:
 *
 * 5 kHz / 5
 * = 1 kHz
 */
#define SPEED_DIV                   5u

#define SPEED_HZ                    (CONTROL_HZ / (float)SPEED_DIV)

#define PI_HZ                       SPEED_HZ


#define MAX_RUN_TICKS               \
    ((uint32_t)((SPEED_HZ * (float)MAX_RUN_MS) / 1000.0f))


#define PULSE_START_DELAY_TICKS     \
    ((uint32_t)((SPEED_HZ * (float)PULSE_START_DELAY_MS) / 1000.0f))


/* ========================================================================== */
/* ALIGNMENT                                                                  */
/* ========================================================================== */

/*
 * Known stator field position.
 */
#define ALIGN_ANGLE_RAD             0.0f


/*
 * KEEPING SAME VALUE AS YOUR WORKING CODE.
 *
 * Reduce to 0.45f if alignment is too aggressive.
 */
#define ALIGN_MODULATION            0.60f


#define ALIGN_TIME_MS               1500u


/*
 * Average multiple Hall measurements after the rotor has aligned.
 */
#define ALIGN_HALL_SAMPLES          100u

#define ALIGN_HALL_SAMPLE_DELAY_MS  2u


/* ========================================================================== */
/* STARTUP TORQUE                                                             */
/* ========================================================================== */

/*
 * Same startup modulation as your working code.
 */
#define START_TORQUE_MODULATION     0.55f


/*
 * Ramp toward full modulation.
 */
#define FULL_TORQUE_RAMP_MS         1200u


#define FULL_TORQUE_RAMP_STEPS      \
    ((uint32_t)((CONTROL_HZ * (float)FULL_TORQUE_RAMP_MS) / 1000.0f))


/* ========================================================================== */
/* TORQUE ANGLE                                                               */
/* ========================================================================== */

#define DEFAULT_TORQUE_ADVANCE_DEG  90.0f


/* ========================================================================== */
/* MODULATION                                                                 */
/* ========================================================================== */

#define M_MIN                       0.00f   /* allow zero drive so speed can fall to 0 */

#define M_MAX                       1.00f


/* ========================================================================== */
/* SPEED PI                                                                   */
/* ========================================================================== */

#define KP                          0.0020f

#define KI                          0.0150f

#define SPEED_ALPHA                 0.18f


/* ========================================================================== */
/* HALL CALIBRATION                                                           */
/* ========================================================================== */

/*
 * DRV5053 zero-field nominal output ~1 V.
 *
 * With 3.3-V ADC reference:
 *
 * 4095 * 1 / 3.3 ~= 1241
 *
 * Replace these later with actual measured values.
 */
#define HA_OFFSET                   1241.0f
#define HB_OFFSET                   1241.0f
#define HC_OFFSET                   1241.0f

#define HA_GAIN                     1.0f
#define HB_GAIN                     1.0f
#define HC_GAIN                     1.0f

#define HALL_MIN_MAG                40.0f


/*
 * Do not stop because of one invalid Hall sample.
 */
#define HALL_FAULT_LIMIT            20u


/* USER CODE END PD */


/* Private variables ---------------------------------------------------------*/

/*
 * NEW PCB:
 *
 * ADC1:
 * PA3 = Hall Sensor 2A = Hall B
 */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;


/*
 * ADC2:
 * PA4 = Hall Sensor 1A = Hall A
 * PA5 = Hall Sensor 3A = Hall C
 */
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;


TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;


/* USER CODE BEGIN PV */


/* ========================================================================== */
/* HALL ADC                                                                   */
/* ========================================================================== */

/*
 * ADC2 DMA:
 *
 * [0] = PA4 = Hall A
 * [1] = PA5 = Hall C
 */
volatile uint16_t hall_ac_adc[2] =
{
    0u,
    0u
};


/*
 * ADC1 DMA:
 *
 * PA3 = Hall B
 */
volatile uint16_t hall_b_adc =
    0u;


/* ========================================================================== */
/* USER VARIABLES                                                             */
/* ========================================================================== */

volatile float speed_reference_rpm =
    TARGET_RPM;


volatile float torque_advance_deg =
    DEFAULT_TORQUE_ADVANCE_DEG;


/* ========================================================================== */
/* PULSATILITY (live tunable)                                                 */
/* ========================================================================== */

/*
 * Set to 0 in the debugger to go back to constant TARGET_RPM.
 */
volatile uint8_t pulse_enabled =
    PULSE_ENABLE_DEFAULT;


volatile float pulse_bpm =
    PULSE_BPM;


volatile float pulse_low_rpm =
    PULSE_LOW_RPM;


volatile float pulse_high_rpm =
    PULSE_HIGH_RPM;


/*
 * Debug: position within current beat, 0.0 to 1.0.
 */
volatile float pulse_phase =
    0.0f;


/*
 * Debug: 1 once the start delay has elapsed and pulsing is live.
 */
volatile uint8_t pulse_active =
    0u;


/* ========================================================================== */
/* SPEED DEBUG                                                                */
/* ========================================================================== */

volatile float measured_speed_rpm =
    0.0f;


volatile float raw_speed_rpm =
    0.0f;


volatile float speed_error_rpm =
    0.0f;


/* ========================================================================== */
/* ANGLE DEBUG                                                                */
/* ========================================================================== */

volatile float electrical_angle_rad =
    0.0f;


volatile float drive_angle_rad =
    0.0f;


volatile float aligned_hall_angle_rad =
    0.0f;


volatile float aligned_hall_angle_deg =
    0.0f;


/*
 * Learned Hall-to-motor electrical calibration.
 */
volatile float hall_zero_offset_rad =
    0.0f;


volatile float hall_zero_offset_deg =
    0.0f;


/* ========================================================================== */
/* POWER DEBUG                                                                */
/* ========================================================================== */

volatile float modulation_command =
    START_TORQUE_MODULATION;


volatile float hall_magnitude =
    0.0f;


volatile uint8_t full_torque_mode =
    0u;


volatile uint8_t hall_fault =
    0u;


volatile uint32_t hall_fault_count =
    0u;


/*
 * DRV8316 hardware fault.
 */
volatile uint8_t driver_fault =
    0u;


volatile uint32_t driver_fault_count =
    0u;


/* ========================================================================== */
/* STATE                                                                      */
/* ========================================================================== */

volatile uint8_t motor_running =
    0u;


volatile uint8_t motor_state =
    0u;


#define MOTOR_STATE_IDLE            0u
#define MOTOR_STATE_ALIGN           1u
#define MOTOR_STATE_HALL_CAL        2u
#define MOTOR_STATE_TORQUE_RAMP     3u
#define MOTOR_STATE_CLOSED          4u
#define MOTOR_STATE_FAULT           5u
#define MOTOR_STATE_MAX_RUN         6u


/* ========================================================================== */
/* MAX SPEED RUN DEBUG                                                        */
/* ========================================================================== */

/*
 * Highest filtered speed seen during the 5 s full-modulation run.
 */
volatile float max_run_rpm =
    0.0f;


/*
 * 1 once the max run has finished and the sinusoid limits are set.
 */
volatile uint8_t max_run_done =
    0u;




/* ========================================================================== */
/* INTERNAL                                                                   */
/* ========================================================================== */

/*
 * IMPORTANT:
 *
 * Keeping 65536-count PWM period from the code that you
 * confirmed actually spins the motor.
 */
static uint32_t pwm_period =
    65536u;


static uint32_t control_div_count =
    0u;


static uint32_t speed_div_count =
    0u;


static uint32_t torque_ramp_count =
    0u;


static uint32_t heartbeat =
    0u;


static uint32_t pulse_start_count =
    0u;


static uint32_t max_run_count =
    0u;


static float previous_hall_angle =
    0.0f;


static float speed_angle_accumulator =
    0.0f;


static float integrator =
    START_TORQUE_MODULATION;


static uint8_t closed_loop_enabled =
    0u;


static uint8_t previous_full_torque_mode =
    1u;


/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);

static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);

static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);


/* USER CODE BEGIN PFP */

static float limit(
    float x,
    float lo,
    float hi
);

static float wrap_pi(
    float x
);

static float wrap_2pi(
    float x
);

static uint32_t ccr(
    float duty
);

static void phases(
    float a,
    float b,
    float c
);

static void neutral(void);

static void drive_svpwm(
    float theta,
    float modulation
);

static uint8_t hall_angle(
    float *theta
);

static void update_pulse_reference(void);

static void max_run_step(void);

static void speed_controller(void);

static void motor_align_and_calibrate(void);

static void motor_hall_start(void);

static void driver_enable(void);

static void driver_disable(void);

static uint8_t driver_fault_active(void);

/* USER CODE END PFP */


/* USER CODE BEGIN 0 */


/* ========================================================================== */
/* DRIVER                                                                     */
/* ========================================================================== */

static void driver_enable(void)
{
    /*
     * DRV8316:
     *
     * DRVOFF LOW = outputs enabled.
     */
    HAL_GPIO_WritePin(
        DRVOFF_PORT,
        DRVOFF_PIN,
        GPIO_PIN_RESET
    );
}


static void driver_disable(void)
{
    /*
     * DRVOFF HIGH disables all six MOSFETs.
     */
    HAL_GPIO_WritePin(
        DRVOFF_PORT,
        DRVOFF_PIN,
        GPIO_PIN_SET
    );
}


static uint8_t driver_fault_active(void)
{
    /*
     * nFAULT is active LOW.
     */
    return
        HAL_GPIO_ReadPin(
            NFAULT_PORT,
            NFAULT_PIN
        )
        ==
        GPIO_PIN_RESET;
}


/* ========================================================================== */
/* LIMIT                                                                      */
/* ========================================================================== */

static float limit(
    float x,
    float lo,
    float hi
)
{
    if (x < lo)
    {
        return lo;
    }

    if (x > hi)
    {
        return hi;
    }

    return x;
}


/* ========================================================================== */
/* ANGLE WRAPPING                                                             */
/* ========================================================================== */

static float wrap_pi(
    float x
)
{
    while (x > PI_F)
    {
        x -= TWO_PI_F;
    }

    while (x < -PI_F)
    {
        x += TWO_PI_F;
    }

    return x;
}


static float wrap_2pi(
    float x
)
{
    while (x >= TWO_PI_F)
    {
        x -= TWO_PI_F;
    }

    while (x < 0.0f)
    {
        x += TWO_PI_F;
    }

    return x;
}


/* ========================================================================== */
/* PWM                                                                        */
/* ========================================================================== */

static uint32_t ccr(
    float duty
)
{
    duty =
        limit(
            duty,
            0.01f,
            0.99f
        );

    return
        (uint32_t)
        (
            duty *
            (float)pwm_period +
            0.5f
        );
}


/* ========================================================================== */
/* PHASE OUTPUT                                                               */
/* ========================================================================== */

static void phases(
    float a,
    float b,
    float c
)
{
    /*
     * DRV8316 3xPWM:
     *
     * Phase A / INHA
     * PB7
     * TIM4 CH2
     */
    __HAL_TIM_SET_COMPARE(
        &htim4,
        TIM_CHANNEL_2,
        ccr(c)
    );


    /*
     * Phase B / INHB
     * PB6
     * TIM4 CH1
     */
    __HAL_TIM_SET_COMPARE(
        &htim4,
        TIM_CHANNEL_1,
        ccr(b)
    );


    /*
     * Phase C / INHC
     * PB5
     * TIM3 CH2
     */
    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_2,
        ccr(a)
    );
}


/* ========================================================================== */
/* NEUTRAL                                                                    */
/* ========================================================================== */

static void neutral(void)
{
    /*
     * Equal duty on all phases gives approximately
     * zero phase-to-phase voltage.
     */
    phases(
        0.5f,
        0.5f,
        0.5f
    );
}


/* ========================================================================== */
/* SVPWM                                                                      */
/* ========================================================================== */

static void drive_svpwm(
    float theta,
    float modulation
)
{
    float va;
    float vb;
    float vc;

    float vmax;
    float vmin;

    float common;

    float da;
    float db;
    float dc;


    modulation =
        limit(
            modulation,
            0.0f,
            M_MAX
        );


    va =
        sinf(theta);


    vb =
        sinf(
            theta -
            2.0f * PI_F / 3.0f
        );


    vc =
        sinf(
            theta +
            2.0f * PI_F / 3.0f
        );


    vmax =
        va;

    if (vb > vmax)
    {
        vmax = vb;
    }

    if (vc > vmax)
    {
        vmax = vc;
    }


    vmin =
        va;

    if (vb < vmin)
    {
        vmin = vb;
    }

    if (vc < vmin)
    {
        vmin = vc;
    }


    /*
     * Common-mode injection.
     */
    common =
        -0.5f *
        (
            vmax +
            vmin
        );


    va += common;
    vb += common;
    vc += common;


    va *=
        modulation *
        SVPWM_GAIN;


    vb *=
        modulation *
        SVPWM_GAIN;


    vc *=
        modulation *
        SVPWM_GAIN;


    da =
        0.5f +
        0.5f * va;


    db =
        0.5f +
        0.5f * vb;


    dc =
        0.5f +
        0.5f * vc;


    phases(
        da,
        db,
        dc
    );
}


/* ========================================================================== */
/* HALL ANGLE                                                                 */
/* ========================================================================== */

static uint8_t hall_angle(
    float *theta
)
{
    float a;
    float b;
    float c;

    float alpha;
    float beta;

    float mag_sq;

    float raw_theta;


    /*
     * NEW PCB HALL MAPPING
     *
     * Hall A = Hall Sensor 1A
     * PA4
     * ADC2_IN17
     * ADC2 DMA rank 1
     */
    c =
        (
            (float)hall_ac_adc[0] -
            HA_OFFSET
        ) *
        HA_GAIN;


    /*
     * Hall B = Hall Sensor 2A
     * PA3
     * ADC1_IN4
     */
    b =
        (
            (float)hall_b_adc -
            HB_OFFSET
        ) *
        HB_GAIN;


    /*
     * Hall C = Hall Sensor 3A
     * PA5
     * ADC2_IN13
     * ADC2 DMA rank 2
     */
    a =
        (
            (float)hall_ac_adc[1] -
            HC_OFFSET
        ) *
        HC_GAIN;


    /*
     * Clarke transform.
     */
    alpha =
        (2.0f / 3.0f) *
        (
            a -
            0.5f * b -
            0.5f * c
        );


    beta =
        (2.0f / 3.0f) *
        SQRT3_2 *
        (
            b -
            c
        );


    mag_sq =
        alpha * alpha +
        beta * beta;


    hall_magnitude =
        sqrtf(
            mag_sq
        );


    if
    (
        mag_sq <
        HALL_MIN_MAG *
        HALL_MIN_MAG
    )
    {
        return 0u;
    }


    raw_theta =
        atan2f(
            beta,
            alpha
        );


    raw_theta *=
        HALL_DIRECTION;


    *theta =
        wrap_pi(
            raw_theta
        );


    return 1u;
}


/* ========================================================================== */
/* PULSATILE SPEED REFERENCE                                                  */
/* ========================================================================== */

/*
 * Called at SPEED_HZ (1 kHz) from the ISR, only in MOTOR_STATE_CLOSED,
 * immediately before speed_controller().
 *
 * Writes speed_reference_rpm.
 */
static void update_pulse_reference(void)
{
    float bpm;
    float low;
    float high;
    float mean;
    float amplitude;


    /* ====================================================================== */
    /* DISABLED: constant speed                                               */
    /* ====================================================================== */

    if (!pulse_enabled)
    {
        /*
         * Hold the middle of the sinusoid band.
         */
        speed_reference_rpm =
            0.5f *
            (
                pulse_low_rpm +
                pulse_high_rpm
            );


        pulse_phase =
            0.0f;


        pulse_start_count =
            0u;


        pulse_active =
            0u;


        return;
    }


    /* ====================================================================== */
    /* SETTLE AT TARGET BEFORE PULSING                                        */
    /* ====================================================================== */

    if
    (
        pulse_start_count <
        PULSE_START_DELAY_TICKS
    )
    {
        pulse_start_count++;


        /*
         * Only reached if pulsing was toggled off and back on.
         * Hold the band mean, then the sine starts from it.
         */
        speed_reference_rpm =
            0.5f *
            (
                pulse_low_rpm +
                pulse_high_rpm
            );


        pulse_phase =
            0.0f;


        pulse_active =
            0u;


        return;
    }


    pulse_active =
        1u;


    /* ====================================================================== */
    /* CLAMP LIVE-TUNABLE VALUES                                              */
    /* ====================================================================== */

    bpm =
        limit(
            pulse_bpm,
            PULSE_BPM_MIN,
            PULSE_BPM_MAX
        );


    low =
        limit(
            pulse_low_rpm,
            PULSE_RPM_MIN,
            PULSE_RPM_MAX
        );


    high =
        limit(
            pulse_high_rpm,
            low,
            PULSE_RPM_MAX
        );


    /* ====================================================================== */
    /* ADVANCE BEAT PHASE                                                     */
    /* ====================================================================== */

    pulse_phase +=
        bpm /
        (
            60.0f *
            SPEED_HZ
        );


    while (pulse_phase >= 1.0f)
    {
        pulse_phase -=
            1.0f;
    }


    /* ====================================================================== */
    /* WAVEFORM                                                               */
    /* ====================================================================== */

    /*
     * Pure sinusoid about the mean:
     *
     *   rpm = mean + amplitude * sin(2*pi*phase)
     *
     * Phase 0 = mean, rising, so pulsing starts from the
     * steady-state speed with no step.
     */
    mean =
        0.5f *
        (
            low +
            high
        );


    amplitude =
        0.5f *
        (
            high -
            low
        );


    speed_reference_rpm =
        mean +
        amplitude *
        sinf(
            TWO_PI_F *
            pulse_phase
        );
}


/* ========================================================================== */
/* SPEED CONTROLLER                                                           */
/* ========================================================================== */

static void speed_controller(void)
{
    float error;

    float proposed_integrator;

    float raw_output;

    float output;


    error =
        speed_reference_rpm -
        measured_speed_rpm;


    speed_error_rpm =
        error;


    /* ====================================================================== */
    /* FULL TORQUE BELOW TARGET                                               */
    /* ====================================================================== */

    if
    (
        measured_speed_rpm <
        (
            speed_reference_rpm -
            FULL_TORQUE_BAND_RPM
        )
    )
    {
        full_torque_mode =
            1u;


        modulation_command =
            M_MAX;


        previous_full_torque_mode =
            1u;


        return;
    }


    /* ====================================================================== */
    /* TRANSITION TO PI                                                       */
    /* ====================================================================== */

    if (previous_full_torque_mode)
    {
        integrator =
            modulation_command -
            KP *
            error;


        integrator =
            limit(
                integrator,
                M_MIN,
                M_MAX
            );


        previous_full_torque_mode =
            0u;
    }


    full_torque_mode =
        0u;


    /* ====================================================================== */
    /* PI                                                                     */
    /* ====================================================================== */

    proposed_integrator =
        integrator +
        KI *
        error /
        PI_HZ;


    raw_output =
        KP *
        error +
        proposed_integrator;


    output =
        limit(
            raw_output,
            M_MIN,
            M_MAX
        );


    if
    (
        raw_output ==
        output
    )
    {
        integrator =
            proposed_integrator;
    }

    else if
    (
        raw_output >
        M_MAX &&
        error < 0.0f
    )
    {
        integrator =
            proposed_integrator;
    }

    else if
    (
        raw_output <
        M_MIN &&
        error > 0.0f
    )
    {
        integrator =
            proposed_integrator;
    }


    integrator =
        limit(
            integrator,
            M_MIN,
            M_MAX
        );


    modulation_command =
        output;
}


/* ========================================================================== */
/* MAX SPEED RUN                                                              */
/* ========================================================================== */

/*
 * Called at SPEED_HZ (1 kHz) from the ISR while in MOTOR_STATE_MAX_RUN.
 *
 * Holds full modulation for MAX_RUN_MS, tracks the peak filtered speed,
 * then sets the sinusoid limits and hands over to closed-loop pulsing.
 */
static void max_run_step(void)
{
    float high;
    float low;


    /*
     * Flat out.
     */
    modulation_command =
        M_MAX;


    full_torque_mode =
        1u;


    /*
     * Track peak of the FILTERED speed so one noisy raw
     * sample can't set the limits.
     */
    if
    (
        measured_speed_rpm >
        max_run_rpm
    )
    {
        max_run_rpm =
            measured_speed_rpm;
    }


    max_run_count++;


    if
    (
        max_run_count <
        MAX_RUN_TICKS
    )
    {
        return;
    }


    /* ====================================================================== */
    /* RUN FINISHED                                                           */
    /* ====================================================================== */

    if
    (
        max_run_rpm <
        MAX_RUN_MIN_VALID_RPM
    )
    {
        /*
         * Motor never really got going. Don't pulse around garbage.
         */
        motor_state =
            MOTOR_STATE_FAULT;


        closed_loop_enabled =
            0u;


        motor_running =
            0u;


        neutral();

        driver_disable();

        return;
    }


    high =
        PULSE_HIGH_FRACTION *
        max_run_rpm;


    low =
        PULSE_LOW_FRACTION *
        max_run_rpm;


    pulse_high_rpm =
        high;


    pulse_low_rpm =
        low;


    /*
     * Motor is at max right now. Start the sine at its peak
     * (phase 0.25, sin = +1) so the first move is a smooth
     * 10% drop to `high`, not a 15% jump to the mean.
     */
    pulse_phase =
        0.25f;


    /*
     * Skip the settle delay, the max run already settled it.
     */
    pulse_start_count =
        PULSE_START_DELAY_TICKS;


    speed_reference_rpm =
        high;


    /*
     * Hand over to the speed controller from full torque.
     */
    previous_full_torque_mode =
        1u;


    max_run_done =
        1u;


    motor_state =
        MOTOR_STATE_CLOSED;
}


/* ========================================================================== */
/* ALIGN + CALIBRATE HALL ZERO                                                */
/* ========================================================================== */

static void motor_align_and_calibrate(void)
{
    uint32_t start;

    uint32_t sample_count;

    float theta;

    float sin_sum =
        0.0f;

    float cos_sum =
        0.0f;

    float average_theta;


    motor_state =
        MOTOR_STATE_ALIGN;


    modulation_command =
        ALIGN_MODULATION;


    /*
     * Hold fixed magnetic field.
     */
    start =
        HAL_GetTick();


    while
    (
        (
            HAL_GetTick() -
            start
        )
        <
        ALIGN_TIME_MS
    )
    {
        drive_svpwm(
            ALIGN_ANGLE_RAD,
            ALIGN_MODULATION
        );
    }


    /*
     * Keep holding rotor while measuring Hall position.
     */
    drive_svpwm(
        ALIGN_ANGLE_RAD,
        ALIGN_MODULATION
    );


    motor_state =
        MOTOR_STATE_HALL_CAL;


    /*
     * Average Hall angle circularly.
     */
    for
    (
        sample_count = 0u;
        sample_count < ALIGN_HALL_SAMPLES;
        sample_count++
    )
    {
        if
        (
            hall_angle(
                &theta
            )
        )
        {
            sin_sum +=
                sinf(theta);


            cos_sum +=
                cosf(theta);
        }


        HAL_Delay(
            ALIGN_HALL_SAMPLE_DELAY_MS
        );
    }


    /*
     * No valid Hall field.
     */
    if
    (
        (
            sin_sum * sin_sum +
            cos_sum * cos_sum
        )
        <
        1.0f
    )
    {
        hall_fault =
            1u;


        motor_state =
            MOTOR_STATE_FAULT;


        neutral();

        driver_disable();

        Error_Handler();
    }


    average_theta =
        atan2f(
            sin_sum,
            cos_sum
        );


    aligned_hall_angle_rad =
        average_theta;


    aligned_hall_angle_deg =
        average_theta *
        180.0f /
        PI_F;


    /*
     * The stator field is currently ALIGN_ANGLE_RAD.
     *
     * Therefore the actual rotor electrical angle should
     * correspond to ALIGN_ANGLE_RAD.
     *
     * This learns the offset between the physical Hall
     * sensor arrangement and the motor phase coordinate.
     */
    hall_zero_offset_rad =
        wrap_pi(
            ALIGN_ANGLE_RAD -
            average_theta
        );


    hall_zero_offset_deg =
        hall_zero_offset_rad *
        180.0f /
        PI_F;


    /*
     * Initialise speed calculation.
     */
    previous_hall_angle =
        average_theta;


    speed_angle_accumulator =
        0.0f;


    measured_speed_rpm =
        0.0f;


    raw_speed_rpm =
        0.0f;
}


/* ========================================================================== */
/* START DIRECTLY IN HALL CLOSED LOOP                                         */
/* ========================================================================== */

static void motor_hall_start(void)
{
    float theta;

    float torque_advance_rad;

    float command_angle;


    if
    (
        !hall_angle(
            &theta
        )
    )
    {
        hall_fault =
            1u;


        motor_state =
            MOTOR_STATE_FAULT;


        neutral();

        driver_disable();

        Error_Handler();
    }


    previous_hall_angle =
        theta;


    electrical_angle_rad =
        theta;


    torque_advance_rad =
        torque_advance_deg *
        PI_F /
        180.0f;


    /*
     * IMPORTANT:
     *
     * This is the behaviour from the working firmware.
     *
     * There is NO open-loop rotating-field startup.
     *
     * Instead:
     *
     * measured rotor angle
     * + learned Hall offset
     * + torque advance
     */
    command_angle =
        theta +
        hall_zero_offset_rad +
        DIRECTION *
        torque_advance_rad;


    command_angle =
        wrap_2pi(
            command_angle
        );


    drive_angle_rad =
        command_angle;


    modulation_command =
        START_TORQUE_MODULATION;


    drive_svpwm(
        command_angle,
        modulation_command
    );


    torque_ramp_count =
        0u;


    control_div_count =
        0u;


    speed_div_count =
        0u;


    hall_fault_count =
        0u;


    driver_fault_count =
        0u;


    pulse_start_count =
        0u;


    pulse_phase =
        0.0f;


    pulse_active =
        0u;


    max_run_count =
        0u;


    max_run_rpm =
        0.0f;


    max_run_done =
        0u;


    speed_reference_rpm =
        TARGET_RPM;


    integrator =
        START_TORQUE_MODULATION;


    previous_full_torque_mode =
        1u;


    full_torque_mode =
        1u;


    motor_state =
        MOTOR_STATE_TORQUE_RAMP;


    closed_loop_enabled =
        1u;


    motor_running =
        1u;


    if
    (
        HAL_TIM_Base_Start_IT(
            &htim6
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* TIM6 CONTROL ISR                                                           */
/* ========================================================================== */

void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef *htim
)
{
    float theta;

    float delta;

    float calculated_rpm;

    float torque_advance_rad;

    float command_angle;

    float ramp;


    if
    (
        htim->Instance !=
        TIM6
    )
    {
        return;
    }


    if (!closed_loop_enabled)
    {
        return;
    }


    /* ====================================================================== */
    /* 20 kHz -> 5 kHz CONTROL                                                */
    /* ====================================================================== */

    control_div_count++;


    if
    (
        control_div_count <
        CONTROL_DIV
    )
    {
        return;
    }


    control_div_count =
        0u;


    /* ====================================================================== */
    /* DRV8316 nFAULT                                                         */
    /* ====================================================================== */

    if
    (
        driver_fault_active()
    )
    {
        driver_fault_count++;


        if
        (
            driver_fault_count >=
            DRIVER_FAULT_LIMIT
        )
        {
            driver_fault =
                1u;


            motor_state =
                MOTOR_STATE_FAULT;


            closed_loop_enabled =
                0u;


            motor_running =
                0u;


            /*
             * Remove PWM torque command first.
             */
            neutral();


            /*
             * Then disable the power stage.
             */
            driver_disable();


            return;
        }
    }
    else
    {
        driver_fault_count =
            0u;


        driver_fault =
            0u;
    }


    /* ====================================================================== */
    /* HALL                                                                   */
    /* ====================================================================== */

    if
    (
        !hall_angle(
            &theta
        )
    )
    {
        hall_fault_count++;


        if
        (
            hall_fault_count >=
            HALL_FAULT_LIMIT
        )
        {
            hall_fault =
                1u;


            motor_state =
                MOTOR_STATE_FAULT;


            closed_loop_enabled =
                0u;


            motor_running =
                0u;


            neutral();


            driver_disable();
        }


        return;
    }


    hall_fault_count =
        0u;


    hall_fault =
        0u;


    electrical_angle_rad =
        theta;


    /* ====================================================================== */
    /* SPEED                                                                  */
    /* ====================================================================== */

    delta =
        wrap_pi(
            theta -
            previous_hall_angle
        );


    previous_hall_angle =
        theta;


    speed_angle_accumulator +=
        delta;


    speed_div_count++;


    if
    (
        speed_div_count >=
        SPEED_DIV
    )
    {
        speed_div_count =
            0u;


        calculated_rpm =
            speed_angle_accumulator *
            SPEED_HZ *
            60.0f /
            (
                TWO_PI_F *
                POLE_PAIRS
            );


        speed_angle_accumulator =
            0.0f;


        /*
         * Convert selected motor direction into positive RPM.
         */
        calculated_rpm *=
            DIRECTION;


        raw_speed_rpm =
            calculated_rpm;


        measured_speed_rpm +=
            SPEED_ALPHA *
            (
                calculated_rpm -
                measured_speed_rpm
            );


        if
        (
            motor_state ==
            MOTOR_STATE_CLOSED
        )
        {
            /*
             * Pulsatile reference first, then regulate to it.
             */
            update_pulse_reference();

            speed_controller();
        }
        else if
        (
            motor_state ==
            MOTOR_STATE_MAX_RUN
        )
        {
            max_run_step();


            /*
             * max_run_step() may have faulted the drive.
             */
            if (!closed_loop_enabled)
            {
                return;
            }
        }
    }


    /* ====================================================================== */
    /* TORQUE ANGLE                                                           */
    /* ====================================================================== */

    torque_advance_deg =
        limit(
            torque_advance_deg,
            30.0f,
            150.0f
        );


    torque_advance_rad =
        torque_advance_deg *
        PI_F /
        180.0f;


    /*
     * Rotor-locked electrical field:
     *
     * measured Hall rotor angle
     * + learned Hall offset
     * + torque-producing advance
     */
    command_angle =
        theta +
        hall_zero_offset_rad +
        DIRECTION *
        torque_advance_rad;


    command_angle =
        wrap_2pi(
            command_angle
        );


    drive_angle_rad =
        command_angle;


    /* ====================================================================== */
    /* TORQUE RAMP                                                            */
    /* ====================================================================== */

    if
    (
        motor_state ==
        MOTOR_STATE_TORQUE_RAMP
    )
    {
        torque_ramp_count++;


        ramp =
            (float)torque_ramp_count /
            (float)FULL_TORQUE_RAMP_STEPS;


        ramp =
            limit(
                ramp,
                0.0f,
                1.0f
            );


        modulation_command =
            START_TORQUE_MODULATION +
            (
                M_MAX -
                START_TORQUE_MODULATION
            ) *
            ramp;


        full_torque_mode =
            1u;


        if
        (
            torque_ramp_count >=
            FULL_TORQUE_RAMP_STEPS
        )
        {
            modulation_command =
                M_MAX;


            full_torque_mode =
                1u;


            previous_full_torque_mode =
                1u;


            /*
             * Ramp done: go flat out for MAX_RUN_MS to find max speed.
             */
            max_run_count =
                0u;


            max_run_rpm =
                0.0f;


            motor_state =
                MOTOR_STATE_MAX_RUN;
        }
    }


    /* ====================================================================== */
    /* DRIVE                                                                  */
    /* ====================================================================== */

    drive_svpwm(
        command_angle,
        modulation_command
    );


    /* ====================================================================== */
    /* HEARTBEAT                                                              */
    /* ====================================================================== */

    heartbeat++;


    if
    (
        heartbeat >=
        2500u
    )
    {
        heartbeat =
            0u;


        HAL_GPIO_TogglePin(
            STATUS_LED_PORT,
            STATUS_LED_PIN
        );
    }
}


/* USER CODE END 0 */


/* ========================================================================== */
/* MAIN                                                                       */
/* ========================================================================== */

int main(void)
{
    HAL_Init();


    SystemClock_Config();


    MX_GPIO_Init();

    MX_DMA_Init();

    MX_TIM3_Init();

    MX_TIM4_Init();

    MX_TIM6_Init();

    MX_ADC1_Init();

    MX_ADC2_Init();


    /* USER CODE BEGIN 2 */


    /*
     * Keep the CAN transceiver inactive.
     */
    HAL_GPIO_WritePin(
        CAN_STBY_PORT,
        CAN_STBY_PIN,
        GPIO_PIN_SET
    );


    /*
     * DRV8316 OFF while timers and ADC are starting.
     */
    driver_disable();


    /* ====================================================================== */
    /* PWM                                                                    */
    /* ====================================================================== */

    pwm_period =
        __HAL_TIM_GET_AUTORELOAD(
            &htim4
        )
        +
        1u;


    neutral();


    /*
     * PB6
     * Phase B
     */
    if
    (
        HAL_TIM_PWM_Start(
            &htim4,
            TIM_CHANNEL_1
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    /*
     * PB7
     * Phase A
     */
    if
    (
        HAL_TIM_PWM_Start(
            &htim4,
            TIM_CHANNEL_2
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    /*
     * PB5
     * Phase C
     */
    if
    (
        HAL_TIM_PWM_Start(
            &htim3,
            TIM_CHANNEL_2
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    /* ====================================================================== */
    /* HALL ADC                                                               */
    /* ====================================================================== */

    /*
     * ADC1
     * PA3
     * Hall B
     */
    if
    (
        HAL_ADCEx_Calibration_Start(
            &hadc1,
            ADC_SINGLE_ENDED
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    /*
     * ADC2
     * PA4 Hall A
     * PA5 Hall C
     */
    if
    (
        HAL_ADCEx_Calibration_Start(
            &hadc2,
            ADC_SINGLE_ENDED
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    /*
     * ADC1 DMA:
     *
     * Hall B
     */
    if
    (
        HAL_ADC_Start_DMA(
            &hadc1,
            (uint32_t *)&hall_b_adc,
            1u
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    /*
     * ADC2 DMA:
     *
     * hall_ac_adc[0] = Hall A
     * hall_ac_adc[1] = Hall C
     */
    if
    (
        HAL_ADC_Start_DMA(
            &hadc2,
            (uint32_t *)hall_ac_adc,
            2u
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    /*
     * Let ADC values settle.
     */
    HAL_Delay(
        100
    );


    motor_state =
        MOTOR_STATE_IDLE;


    neutral();


    /*
     * Enable the DRV8316.
     *
     * DRVOFF LOW.
     */
    driver_enable();


    /*
     * Give driver time to enable.
     */
    HAL_Delay(
        10
    );


    /*
     * Do not immediately kill startup because nFAULT
     * can have a very short transient during power-up.
     *
     * The ISR performs debounced nFAULT monitoring.
     */


    /*
     * Same startup pause as working firmware.
     */
    HAL_Delay(
        2000
    );


    /* ====================================================================== */
    /* ALIGN ROTOR AND CALIBRATE HALL                                         */
    /* ====================================================================== */

    motor_align_and_calibrate();


    /*
     * Short hold before starting torque.
     */
    HAL_Delay(
        50
    );


    /* ====================================================================== */
    /* DIRECT HALL CLOSED-LOOP START                                          */
    /* ====================================================================== */

    motor_hall_start();


    /* USER CODE END 2 */


    while (1)
    {
        HAL_Delay(
            100
        );
    }
}


/* ========================================================================== */
/* SYSTEM CLOCK                                                               */
/* ========================================================================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct =
    {
        0
    };


    RCC_ClkInitTypeDef RCC_ClkInitStruct =
    {
        0
    };


    HAL_PWREx_ControlVoltageScaling(
        PWR_REGULATOR_VOLTAGE_SCALE1_BOOST
    );


    /*
     * 24-MHz HSE crystal.
     */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;


    RCC_OscInitStruct.HSEState =
        RCC_HSE_ON;


    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;


    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;


    /*
     * 24 / 6 = 4 MHz
     * 4 * 85 = 340 MHz
     * 340 / 2 = 170 MHz
     */
    RCC_OscInitStruct.PLL.PLLM =
        RCC_PLLM_DIV6;


    RCC_OscInitStruct.PLL.PLLN =
        85;


    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;


    RCC_OscInitStruct.PLL.PLLQ =
        RCC_PLLQ_DIV2;


    RCC_OscInitStruct.PLL.PLLR =
        RCC_PLLR_DIV2;


    if
    (
        HAL_RCC_OscConfig(
            &RCC_OscInitStruct
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


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


    if
    (
        HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_4
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* ADC1                                                                       */
/* PA3 = Hall B = ADC1_IN4                                                    */
/* ========================================================================== */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig =
    {
        0
    };


    hadc1.Instance =
        ADC1;


    hadc1.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV4;


    hadc1.Init.Resolution =
        ADC_RESOLUTION_12B;


    hadc1.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;


    hadc1.Init.GainCompensation =
        0;


    hadc1.Init.ScanConvMode =
        ADC_SCAN_DISABLE;


    hadc1.Init.EOCSelection =
        ADC_EOC_SEQ_CONV;


    hadc1.Init.LowPowerAutoWait =
        DISABLE;


    hadc1.Init.ContinuousConvMode =
        ENABLE;


    hadc1.Init.NbrOfConversion =
        1;


    hadc1.Init.DiscontinuousConvMode =
        DISABLE;


    hadc1.Init.ExternalTrigConv =
        ADC_SOFTWARE_START;


    hadc1.Init.ExternalTrigConvEdge =
        ADC_EXTERNALTRIGCONVEDGE_NONE;


    hadc1.Init.DMAContinuousRequests =
        ENABLE;


    hadc1.Init.Overrun =
        ADC_OVR_DATA_OVERWRITTEN;


    hadc1.Init.OversamplingMode =
        DISABLE;


    if
    (
        HAL_ADC_Init(
            &hadc1
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    /*
     * PA3
     * ADC1_IN4
     * Hall B
     */
    sConfig.Channel =
        ADC_CHANNEL_4;


    sConfig.Rank =
        ADC_REGULAR_RANK_1;


    sConfig.SamplingTime =
        ADC_SAMPLETIME_47CYCLES_5;


    sConfig.SingleDiff =
        ADC_SINGLE_ENDED;


    sConfig.OffsetNumber =
        ADC_OFFSET_NONE;


    sConfig.Offset =
        0;


    if
    (
        HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* ADC2                                                                       */
/*                                                                            */
/* Rank 1 = PA4 = Hall A = ADC2_IN17                                         */
/* Rank 2 = PA5 = Hall C = ADC2_IN13                                         */
/* ========================================================================== */

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig =
    {
        0
    };


    hadc2.Instance =
        ADC2;


    hadc2.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV4;


    hadc2.Init.Resolution =
        ADC_RESOLUTION_12B;


    hadc2.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;


    hadc2.Init.GainCompensation =
        0;


    hadc2.Init.ScanConvMode =
        ADC_SCAN_ENABLE;


    hadc2.Init.EOCSelection =
        ADC_EOC_SEQ_CONV;


    hadc2.Init.LowPowerAutoWait =
        DISABLE;


    hadc2.Init.ContinuousConvMode =
        ENABLE;


    hadc2.Init.NbrOfConversion =
        2;


    hadc2.Init.DiscontinuousConvMode =
        DISABLE;


    hadc2.Init.ExternalTrigConv =
        ADC_SOFTWARE_START;


    hadc2.Init.ExternalTrigConvEdge =
        ADC_EXTERNALTRIGCONVEDGE_NONE;


    hadc2.Init.DMAContinuousRequests =
        ENABLE;


    hadc2.Init.Overrun =
        ADC_OVR_DATA_OVERWRITTEN;


    hadc2.Init.OversamplingMode =
        DISABLE;


    if
    (
        HAL_ADC_Init(
            &hadc2
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    sConfig.SamplingTime =
        ADC_SAMPLETIME_47CYCLES_5;


    sConfig.SingleDiff =
        ADC_SINGLE_ENDED;


    sConfig.OffsetNumber =
        ADC_OFFSET_NONE;


    sConfig.Offset =
        0;


    /*
     * Rank 1
     *
     * Hall A
     * PA4
     * ADC2_IN17
     */
    sConfig.Channel =
        ADC_CHANNEL_17;


    sConfig.Rank =
        ADC_REGULAR_RANK_1;


    if
    (
        HAL_ADC_ConfigChannel(
            &hadc2,
            &sConfig
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    /*
     * Rank 2
     *
     * Hall C
     * PA5
     * ADC2_IN13
     */
    sConfig.Channel =
        ADC_CHANNEL_13;


    sConfig.Rank =
        ADC_REGULAR_RANK_2;


    if
    (
        HAL_ADC_ConfigChannel(
            &hadc2,
            &sConfig
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* TIM3                                                                       */
/* PB5 = Phase C = TIM3_CH2                                                   */
/*                                                                            */
/* IMPORTANT: Keeping timer period from known-good firmware.                  */
/* ========================================================================== */

static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig =
    {
        0
    };


    TIM_OC_InitTypeDef sConfigOC =
    {
        0
    };


    htim3.Instance =
        TIM3;


    htim3.Init.Prescaler =
        0;


    htim3.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    /*
     * KEEP SAME AS WORKING CODE.
     */
    htim3.Init.Period =
        65535;


    htim3.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;


    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if
    (
        HAL_TIM_PWM_Init(
            &htim3
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;


    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if
    (
        HAL_TIMEx_MasterConfigSynchronization(
            &htim3,
            &sMasterConfig
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    sConfigOC.OCMode =
        TIM_OCMODE_PWM1;


    sConfigOC.Pulse =
        0;


    sConfigOC.OCPolarity =
        TIM_OCPOLARITY_HIGH;


    sConfigOC.OCFastMode =
        TIM_OCFAST_DISABLE;


    if
    (
        HAL_TIM_PWM_ConfigChannel(
            &htim3,
            &sConfigOC,
            TIM_CHANNEL_2
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    HAL_TIM_MspPostInit(
        &htim3
    );
}


/* ========================================================================== */
/* TIM4                                                                       */
/* PB6 = Phase B = TIM4_CH1                                                   */
/* PB7 = Phase A = TIM4_CH2                                                   */
/* ========================================================================== */

static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig =
    {
        0
    };


    TIM_OC_InitTypeDef sConfigOC =
    {
        0
    };


    htim4.Instance =
        TIM4;


    htim4.Init.Prescaler =
        0;


    htim4.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    /*
     * KEEP SAME AS WORKING CODE.
     */
    htim4.Init.Period =
        65535;


    htim4.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;


    htim4.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if
    (
        HAL_TIM_PWM_Init(
            &htim4
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;


    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if
    (
        HAL_TIMEx_MasterConfigSynchronization(
            &htim4,
            &sMasterConfig
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    sConfigOC.OCMode =
        TIM_OCMODE_PWM1;


    sConfigOC.Pulse =
        0;


    sConfigOC.OCPolarity =
        TIM_OCPOLARITY_HIGH;


    sConfigOC.OCFastMode =
        TIM_OCFAST_DISABLE;


    /*
     * PB6
     * Phase B
     */
    if
    (
        HAL_TIM_PWM_ConfigChannel(
            &htim4,
            &sConfigOC,
            TIM_CHANNEL_1
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    /*
     * PB7
     * Phase A
     */
    if
    (
        HAL_TIM_PWM_ConfigChannel(
            &htim4,
            &sConfigOC,
            TIM_CHANNEL_2
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    HAL_TIM_MspPostInit(
        &htim4
    );
}


/* ========================================================================== */
/* TIM6                                                                       */
/* 20-kHz control interrupt                                                   */
/* ========================================================================== */

static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig =
    {
        0
    };


    htim6.Instance =
        TIM6;


    /*
     * 170 MHz / 170 = 1 MHz
     */
    htim6.Init.Prescaler =
        169;


    htim6.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    /*
     * 1 MHz / 50 = 20 kHz
     */
    htim6.Init.Period =
        49;


    htim6.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if
    (
        HAL_TIM_Base_Init(
            &htim6
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;


    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if
    (
        HAL_TIMEx_MasterConfigSynchronization(
            &htim6,
            &sMasterConfig
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* DMA                                                                        */
/* ========================================================================== */

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    __HAL_RCC_DMA1_CLK_ENABLE();


    /*
     * DMA interrupts are not required by the motor algorithm.
     *
     * ADC DMA simply runs continuously in circular mode.
     *
     * TIM6 reads the latest values.
     */
}


/* ========================================================================== */
/* GPIO                                                                       */
/* ========================================================================== */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct =
    {
        0
    };


    __HAL_RCC_GPIOF_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();


    /* ====================================================================== */
    /* SAFE STARTUP                                                           */
    /* ====================================================================== */

    /*
     * DRVOFF HIGH initially.
     *
     * Driver power stage disabled while MCU starts.
     */
    HAL_GPIO_WritePin(
        DRVOFF_PORT,
        DRVOFF_PIN,
        GPIO_PIN_SET
    );


    /*
     * CAN transceiver standby.
     */
    HAL_GPIO_WritePin(
        CAN_STBY_PORT,
        CAN_STBY_PIN,
        GPIO_PIN_SET
    );


    /*
     * LED off.
     */
    HAL_GPIO_WritePin(
        STATUS_LED_PORT,
        STATUS_LED_PIN,
        GPIO_PIN_RESET
    );


    /* ====================================================================== */
    /* HALL SENSOR ANALOG INPUTS                                              */
    /* ====================================================================== */

    /*
     * PA3 = Hall B
     * PA4 = Hall A
     * PA5 = Hall C
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_3 |
        GPIO_PIN_4 |
        GPIO_PIN_5;


    GPIO_InitStruct.Mode =
        GPIO_MODE_ANALOG;


    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /* ====================================================================== */
    /* PA9 DRVOFF                                                             */
    /* ====================================================================== */

    GPIO_InitStruct.Pin =
        DRVOFF_PIN;


    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;


    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        DRVOFF_PORT,
        &GPIO_InitStruct
    );


    /* ====================================================================== */
    /* PA10 nFAULT                                                            */
    /* ====================================================================== */

    GPIO_InitStruct.Pin =
        NFAULT_PIN;


    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;


    /*
     * External pull-up should be fitted on the PCB.
     */
    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    HAL_GPIO_Init(
        NFAULT_PORT,
        &GPIO_InitStruct
    );


    /* ====================================================================== */
    /* PB0 CAN STBY                                                           */
    /* ====================================================================== */

    GPIO_InitStruct.Pin =
        CAN_STBY_PIN;


    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;


    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        CAN_STBY_PORT,
        &GPIO_InitStruct
    );


    /* ====================================================================== */
    /* PB3 STATUS LED                                                         */
    /* ====================================================================== */

    GPIO_InitStruct.Pin =
        STATUS_LED_PIN;


    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;


    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        STATUS_LED_PORT,
        &GPIO_InitStruct
    );
}


/* USER CODE BEGIN 4 */
/* USER CODE END 4 */


/* ========================================================================== */
/* ERROR                                                                      */
/* ========================================================================== */

void Error_Handler(void)
{
    closed_loop_enabled =
        0u;


    motor_running =
        0u;


    motor_state =
        MOTOR_STATE_FAULT;


    /*
     * Command zero phase-to-phase voltage first.
     */
    neutral();


    /*
     * Then physically disable DRV8316 MOSFET outputs.
     */
    driver_disable();


    __disable_irq();


    while (1)
    {
        /*
         * Fast LED blink indicates hard fault.
         */
        HAL_GPIO_TogglePin(
            STATUS_LED_PORT,
            STATUS_LED_PIN
        );

        for
        (
            volatile uint32_t delay = 0u;
            delay < 500000u;
            delay++
        )
        {
        }
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line
)
{
    (void)file;
    (void)line;
}

#endif