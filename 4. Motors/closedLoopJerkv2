/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Hall Start / Maximum Torque Pump Motor Controller
  ******************************************************************************
  *
  * STM32G431CBT6
  *
  * PWM:
  *   Phase A = PB7 = TIM4_CH2
  *   Phase B = PB6 = TIM4_CH1
  *   Phase C = PB5 = TIM3_CH2
  *
  * Analog Hall:
  *   Hall A = PA1 = ADC2_IN2
  *   Hall B = PA6 = ADC2_IN3
  *   Hall C = PA7 = ADC2_IN4
  *
  * Rotor:
  *   4 poles = 2 pole pairs
  *
  * STARTUP:
  *
  *   1. Hold stator field at 0 electrical degrees.
  *   2. Rotor aligns to this field.
  *   3. Read analog Hall electrical angle.
  *   4. Calculate Hall-to-stator electrical offset.
  *   5. Enter Hall closed-loop directly.
  *   6. Apply +90 degree torque-producing field.
  *   7. Ramp modulation to 100%.
  *   8. Maximum torque until target RPM.
  *   9. PI speed regulation near target.
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
/* MOTOR                                                                      */
/* ========================================================================== */

#define POLE_PAIRS                  2.0f

#define DIRECTION                   1.0f

#define HALL_DIRECTION              1.0f


/* ========================================================================== */
/* SPEED                                                                      */
/* ========================================================================== */

#define TARGET_RPM                  1000.0f

#define FULL_TORQUE_BAND_RPM        30.0f


/* ========================================================================== */
/* CONTROL TIMING                                                             */
/* ========================================================================== */

#define ISR_HZ                      20000.0f

#define CONTROL_DIV                 4u

#define CONTROL_HZ                  (ISR_HZ / (float)CONTROL_DIV)

#define SPEED_DIV                   5u

#define SPEED_HZ                    (CONTROL_HZ / (float)SPEED_DIV)

#define PI_HZ                       SPEED_HZ


/* ========================================================================== */
/* ALIGNMENT                                                                  */
/* ========================================================================== */

/*
 * Known stator electrical field angle used during alignment.
 */
#define ALIGN_ANGLE_RAD             0.0f


/*
 * Stronger than before because rotor is inside pump.
 *
 * If alignment is too violent, reduce to 0.45.
 */
#define ALIGN_MODULATION            0.60f


#define ALIGN_TIME_MS               1500u


/*
 * After alignment, average several Hall measurements.
 */
#define ALIGN_HALL_SAMPLES          100u

#define ALIGN_HALL_SAMPLE_DELAY_MS  2u


/* ========================================================================== */
/* STARTUP TORQUE                                                             */
/* ========================================================================== */

/*
 * Start closed-loop torque gently enough to avoid a large sudden step.
 */
#define START_TORQUE_MODULATION     0.55f


/*
 * Ramp to full modulation.
 */
#define FULL_TORQUE_RAMP_MS         1200u

#define FULL_TORQUE_RAMP_STEPS      \
    ((uint32_t)((CONTROL_HZ * (float)FULL_TORQUE_RAMP_MS) / 1000.0f))


/* ========================================================================== */
/* TORQUE ANGLE                                                               */
/* ========================================================================== */

/*
 * Ideal PMSM torque angle starting point.
 *
 * This can be changed live.
 */
#define DEFAULT_TORQUE_ADVANCE_DEG  90.0f


/* ========================================================================== */
/* MODULATION                                                                 */
/* ========================================================================== */

#define M_MIN                       0.15f

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

#define HA_OFFSET                   1241.0f
#define HB_OFFSET                   1241.0f
#define HC_OFFSET                   1241.0f

#define HA_GAIN                     1.0f
#define HB_GAIN                     1.0f
#define HC_GAIN                     1.0f

#define HALL_MIN_MAG                40.0f


/*
 * Do not stop motor because of one noisy Hall sample.
 */
#define HALL_FAULT_LIMIT            20u


/* USER CODE END PD */


/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;


/* USER CODE BEGIN PV */


/* ========================================================================== */
/* HALL ADC                                                                   */
/* ========================================================================== */

volatile uint16_t hall_adc[3] =
{
    0u,
    0u,
    0u
};


/* ========================================================================== */
/* USER VARIABLES                                                             */
/* ========================================================================== */

volatile float speed_reference_rpm =
    TARGET_RPM;


volatile float torque_advance_deg =
    DEFAULT_TORQUE_ADVANCE_DEG;


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


/*
 * Hall angle measured while rotor is aligned.
 */
volatile float aligned_hall_angle_rad =
    0.0f;


volatile float aligned_hall_angle_deg =
    0.0f;


/*
 * Permanent Hall-to-motor electrical calibration.
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


/* ========================================================================== */
/* INTERNAL                                                                   */
/* ========================================================================== */

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

static void speed_controller(void);

static void motor_align_and_calibrate(void);

static void motor_hall_start(void);

/* USER CODE END PFP */


/* USER CODE BEGIN 0 */


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


    return (uint32_t)(
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
     * Phase A = PB7 = TIM4 CH2
     */
    __HAL_TIM_SET_COMPARE(
        &htim4,
        TIM_CHANNEL_2,
        ccr(a)
    );


    /*
     * Phase B = PB6 = TIM4 CH1
     */
    __HAL_TIM_SET_COMPARE(
        &htim4,
        TIM_CHANNEL_1,
        ccr(b)
    );


    /*
     * Phase C = PB5 = TIM3 CH2
     */
    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_2,
        ccr(c)
    );
}


/* ========================================================================== */
/* NEUTRAL                                                                    */
/* ========================================================================== */

static void neutral(void)
{
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


    vmax = va;

    if (vb > vmax)
    {
        vmax = vb;
    }

    if (vc > vmax)
    {
        vmax = vc;
    }


    vmin = va;

    if (vb < vmin)
    {
        vmin = vb;
    }

    if (vc < vmin)
    {
        vmin = vc;
    }


    /*
     * SVPWM-style common-mode injection.
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


    a =
        (
            (float)hall_adc[0] -
            HA_OFFSET
        ) *
        HA_GAIN;


    b =
        (
            (float)hall_adc[1] -
            HB_OFFSET
        ) *
        HB_GAIN;


    c =
        (
            (float)hall_adc[2] -
            HC_OFFSET
        ) *
        HC_GAIN;


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


    if (
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
    /* MAXIMUM TORQUE                                                         */
    /* ====================================================================== */

    if (
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


    if (
        raw_output == output
    )
    {
        integrator =
            proposed_integrator;
    }

    else if (
        raw_output > M_MAX &&
        error < 0.0f
    )
    {
        integrator =
            proposed_integrator;
    }

    else if (
        raw_output < M_MIN &&
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


    /* ====================================================================== */
    /* ALIGN                                                                  */
    /* ====================================================================== */

    motor_state =
        MOTOR_STATE_ALIGN;


    modulation_command =
        ALIGN_MODULATION;


    start =
        HAL_GetTick();


    while (
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


    /* ====================================================================== */
    /* HOLD THE ALIGNMENT FIELD                                               */
    /* ====================================================================== */

    drive_svpwm(
        ALIGN_ANGLE_RAD,
        ALIGN_MODULATION
    );


    motor_state =
        MOTOR_STATE_HALL_CAL;


    /*
     * Average Hall angle circularly.
     *
     * We average sin(theta) and cos(theta)
     * so the +/-180 degree wrap does not
     * corrupt the average.
     */

    for (
        sample_count = 0u;
        sample_count < ALIGN_HALL_SAMPLES;
        sample_count++
    )
    {
        if (
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
     * If Hall field magnitude is invalid,
     * stop here.
     */
    if (
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
     * ================================================================
     * HALL ZERO CALIBRATION
     * ================================================================
     *
     * The stator field is currently at ALIGN_ANGLE_RAD.
     *
     * Therefore:
     *
     * rotor electrical angle should correspond to
     * ALIGN_ANGLE_RAD.
     *
     * hall_zero_offset converts measured Hall position
     * into our stator electrical coordinate frame.
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
     * Initialize speed measurement.
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


    if (
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
     * Immediately command torque from the
     * measured rotor position.
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


    /*
     * Begin at moderate torque rather than
     * jumping instantly from alignment to 100%.
     */
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


    if (
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


    if (
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


    if (
        control_div_count <
        CONTROL_DIV
    )
    {
        return;
    }


    control_div_count =
        0u;


    /* ====================================================================== */
    /* HALL                                                                   */
    /* ====================================================================== */

    if (
        !hall_angle(
            &theta
        )
    )
    {
        hall_fault_count++;


        if (
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
        }


        return;
    }


    /*
     * Valid Hall reading.
     */
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


    if (
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


        /*
         * Once full-torque startup ramp is over,
         * run speed controller.
         */
        if (
            motor_state ==
            MOTOR_STATE_CLOSED
        )
        {
            speed_controller();
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
     * ================================================================
     * ROTOR-LOCKED ELECTRICAL FIELD
     * ================================================================
     *
     * rotor angle
     * + learned Hall zero offset
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

    if (
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


        if (
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


            motor_state =
                MOTOR_STATE_CLOSED;
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


    if (
        heartbeat >=
        2500u
    )
    {
        heartbeat =
            0u;


        HAL_GPIO_TogglePin(
            GPIOB,
            GPIO_PIN_3
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

    MX_ADC2_Init();


    /* USER CODE BEGIN 2 */


    pwm_period =
        __HAL_TIM_GET_AUTORELOAD(
            &htim4
        )
        +
        1u;


    neutral();


    /* ====================================================================== */
    /* PWM                                                                    */
    /* ====================================================================== */

    if (
        HAL_TIM_PWM_Start(
            &htim4,
            TIM_CHANNEL_1
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    if (
        HAL_TIM_PWM_Start(
            &htim4,
            TIM_CHANNEL_2
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    if (
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

    if (
        HAL_ADCEx_Calibration_Start(
            &hadc2,
            ADC_SINGLE_ENDED
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    if (
        HAL_ADC_Start_DMA(
            &hadc2,
            (uint32_t *)hall_adc,
            3u
        )
        != HAL_OK
    )
    {
        Error_Handler();
    }


    HAL_Delay(
        100
    );


    motor_state =
        MOTOR_STATE_IDLE;


    neutral();


    /*
     * Startup pause.
     */
    HAL_Delay(
        2000
    );


    /* ====================================================================== */
    /* ALIGN ROTOR AND CALIBRATE HALL                                         */
    /* ====================================================================== */

    motor_align_and_calibrate();


    /*
     * Small pause while still holding rotor.
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


    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;


    RCC_OscInitStruct.HSEState =
        RCC_HSE_ON;


    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;


    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;


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


    if (
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


    if (
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
/* ADC2                                                                       */
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
        3;


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


    if (
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
     * Hall A = PA1 = ADC2_IN2
     */
    sConfig.Channel =
        ADC_CHANNEL_2;


    sConfig.Rank =
        ADC_REGULAR_RANK_1;


    if (
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
     * Hall B = PA6 = ADC2_IN3
     */
    sConfig.Channel =
        ADC_CHANNEL_3;


    sConfig.Rank =
        ADC_REGULAR_RANK_2;


    if (
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
     * Hall C = PA7 = ADC2_IN4
     */
    sConfig.Channel =
        ADC_CHANNEL_4;


    sConfig.Rank =
        ADC_REGULAR_RANK_3;


    if (
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


    htim3.Init.Period =
        65535;


    htim3.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;


    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if (
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


    if (
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


    if (
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


    htim4.Init.Period =
        65535;


    htim4.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;


    htim4.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if (
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


    if (
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


    if (
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


    if (
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
/* ========================================================================== */

static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig =
    {
        0
    };


    htim6.Instance =
        TIM6;


    htim6.Init.Prescaler =
        169;


    htim6.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    htim6.Init.Period =
        49;


    htim6.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if (
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


    if (
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


    HAL_NVIC_SetPriority(
        DMA1_Channel1_IRQn,
        0,
        0
    );


    HAL_NVIC_EnableIRQ(
        DMA1_Channel1_IRQn
    );
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


    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_9,
        GPIO_PIN_RESET
    );


    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_3,
        GPIO_PIN_RESET
    );


    /*
     * Hall sensors + PA8 original analog input.
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_1 |
        GPIO_PIN_6 |
        GPIO_PIN_7 |
        GPIO_PIN_8;


    GPIO_InitStruct.Mode =
        GPIO_MODE_ANALOG;


    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /*
     * PA9
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_9;


    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;


    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /*
     * PA10
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_10;


    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;


    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /*
     * PB3 heartbeat.
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_3;


    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;


    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct
    );
}


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


    neutral();


    __disable_irq();


    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line
)
{
}

#endif
