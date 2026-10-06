/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : DRV8316Control_SmoothStart.c
  * @brief          : Hall Start / Maximum Torque Pump Motor Controller
  *                    -- OPEN-LOOP RAMP STARTUP VARIANT, FOR BENCH COMPARISON --
  ******************************************************************************
  *
  * Baseline: DRV8316Control.c. SVPWM, Hall angle estimation, speed_controller,
  * fault handling, pinout and peripheral init are UNCHANGED. Only the startup
  * sequence after alignment is different. Flash this on one rotor and compare
  * the mechanical "jerk" against DRV8316Control.c on the same rotor/pump.
  *
  * WHY THE BASELINE JERKS:
  *
  * In DRV8316Control.c, motor_align_and_calibrate() holds the rotor at a
  * fixed field (ALIGN_ANGLE_RAD, ALIGN_MODULATION = 60%) while stationary.
  * Then motor_hall_start() immediately commands a DIFFERENT angle
  * (measured Hall angle + learned offset + 90 degrees torque advance) at a
  * DIFFERENT modulation (START_TORQUE_MODULATION = 55%). Both the field
  * angle and the drive strength step-change instantly while the rotor is
  * still at a standstill -- that abrupt torque step is the "jerk" felt at
  * startup, before the closed-loop torque ramp and PI controller take over.
  *
  * WHAT THIS FILE DOES INSTEAD:
  *
  * After alignment, instead of jumping straight to the Hall-derived command
  * angle, this version enters MOTOR_STATE_OPEN_LOOP_RAMP:
  *
  *   1. The commanded field starts at EXACTLY the same angle and modulation
  *      the alignment step was already holding (ALIGN_ANGLE_RAD,
  *      ALIGN_MODULATION) -- so there is NO discontinuity at the moment
  *      alignment ends. Zero step change.
  *
  *   2. The field angle is then rotated open-loop: its electrical frequency
  *      ramps linearly from 0 Hz up to OPEN_LOOP_HANDOVER_HZ over
  *      OPEN_LOOP_RAMP_MS, while modulation ramps linearly from
  *      ALIGN_MODULATION up to START_TORQUE_MODULATION over the same
  *      period. The rotor is dragged up to a modest handover speed
  *      (OPEN_LOOP_HANDOVER_RPM) gradually, like spinning up a wheel by
  *      hand rather than kicking it.
  *
  *   3. Once the forced field reaches handover frequency, the Hall-measured
  *      speed is checked against the forced speed every 1 kHz speed-loop
  *      tick. Only once the rotor has tracked the forced field within
  *      HANDOVER_SLIP_TOLERANCE_RPM for HANDOVER_CONFIRM_MS continuously
  *      (i.e. it demonstrably is NOT slipping) does the controller hand
  *      over to Hall closed-loop commutation -- exactly the same
  *      motor_hall_start()-style command-angle formula as the baseline,
  *      and then the same torque ramp up to M_MAX and PI speed control.
  *      Because the rotor is already synchronized at handover, the Hall
  *      closed-loop angle and the open-loop forced angle are already
  *      close to each other, so this handover is smooth too.
  *
  *   4. If the rotor fails to track the forced field (e.g. load/friction
  *      too high) within OPEN_LOOP_TIMEOUT_MS of entering this state, the
  *      controller faults safely instead of running a slipping/stalled
  *      open-loop field indefinitely.
  *
  * TUNABLES to adjust on the bench (how gentle vs. how quick):
  *
  *   OPEN_LOOP_RAMP_MS          -- longer = gentler acceleration
  *   OPEN_LOOP_HANDOVER_RPM     -- speed at which Hall handover is attempted
  *   HANDOVER_SLIP_TOLERANCE_RPM-- how exactly measured speed must match
  *   OPEN_LOOP_TIMEOUT_MS       -- safety abort if sync is never achieved
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

#include <math.h>
#include <stdint.h>

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

#define DRIVER_FAULT_LIMIT          25u

/* ========================================================================== */
/* MOTOR                                                                      */
/* ========================================================================== */

#define POLE_PAIRS                  2.0f
#define DIRECTION                   1.0f
#define HALL_DIRECTION              1.0f

/* ========================================================================== */
/* SPEED                                                                      */
/* ========================================================================== */

#define TARGET_RPM                  3000.0f
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

#define ALIGN_ANGLE_RAD             0.0f
#define ALIGN_MODULATION            0.60f
#define ALIGN_TIME_MS               1500u
#define ALIGN_HALL_SAMPLES          100u
#define ALIGN_HALL_SAMPLE_DELAY_MS  2u

/* ========================================================================== */
/* OPEN-LOOP STARTUP RAMP -- NEW IN THIS FILE                                */
/* ========================================================================== */

/* Mechanical RPM at which Hall closed-loop handover is attempted. Keep this
 * well below TARGET_RPM -- it only needs to be fast enough that the Hall
 * speed estimate is clean, not fast enough to be the operating speed.
 * TO TEST: if handover_confirmed never reaches 1 (watch it in the
 * debugger) and motor_state sits in MOTOR_STATE_OPEN_LOOP_RAMP until it
 * faults, try LOWERING this first -- the rotor may not be able to
 * follow the forced field this fast yet. */
#define OPEN_LOOP_HANDOVER_RPM      400.0f

#define OPEN_LOOP_HANDOVER_HZ       (OPEN_LOOP_HANDOVER_RPM * POLE_PAIRS / 60.0f)

/* Time to ramp the forced field from 0 Hz to OPEN_LOOP_HANDOVER_HZ.
 * Longer = gentler mechanical acceleration -- this is the main "how
 * gentle is the startup" knob. If you still feel/hear a jerk, increase
 * this first before touching anything else. */
#define OPEN_LOOP_RAMP_MS           2000u

#define OPEN_LOOP_RAMP_STEPS \
    ((uint32_t)((CONTROL_HZ * (float)OPEN_LOOP_RAMP_MS) / 1000.0f))

/* Rotor must track the forced field within this many RPM, continuously for
 * HANDOVER_CONFIRM_MS, before the controller trusts that it is actually
 * spinning synchronously (not slipping/stalled) and hands over to Hall
 * closed loop.
 * TO TEST: widen HANDOVER_SLIP_TOLERANCE_RPM if raw_speed_rpm oscillates
 * around OPEN_LOOP_HANDOVER_RPM by more than the default +/-100 and
 * handover never confirms despite the rotor visibly spinning fine.
 * Shorten HANDOVER_CONFIRM_MS for a quicker (less certain) handover, or
 * lengthen it if you want more proof of sync before trusting Hall. */
#define HANDOVER_SLIP_TOLERANCE_RPM 100.0f
#define HANDOVER_CONFIRM_MS         200u

#define HANDOVER_CONFIRM_TICKS \
    ((uint32_t)((SPEED_HZ * (float)HANDOVER_CONFIRM_MS) / 1000.0f))

/* Safety abort: if synchronism is never confirmed, fault instead of running
 * a slipping open-loop field indefinitely. */
#define OPEN_LOOP_TIMEOUT_MS        6000u

#define OPEN_LOOP_TIMEOUT_TICKS \
    ((uint32_t)((CONTROL_HZ * (float)OPEN_LOOP_TIMEOUT_MS) / 1000.0f))

/* ========================================================================== */
/* STARTUP TORQUE (Hall closed-loop torque ramp, same as baseline)           */
/* ========================================================================== */

#define START_TORQUE_MODULATION     0.55f
#define FULL_TORQUE_RAMP_MS         1200u
#define FULL_TORQUE_RAMP_STEPS      \
    ((uint32_t)((CONTROL_HZ * (float)FULL_TORQUE_RAMP_MS) / 1000.0f))

/* ========================================================================== */
/* TORQUE ANGLE                                                              */
/* ========================================================================== */

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
#define HALL_FAULT_LIMIT            20u

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

volatile uint16_t hall_ac_adc[2] = { 0u, 0u };
volatile uint16_t hall_b_adc = 0u;

volatile float speed_reference_rpm = TARGET_RPM;
volatile float torque_advance_deg = DEFAULT_TORQUE_ADVANCE_DEG;

volatile float measured_speed_rpm = 0.0f;
volatile float raw_speed_rpm = 0.0f;
volatile float speed_error_rpm = 0.0f;

volatile float electrical_angle_rad = 0.0f;
volatile float drive_angle_rad = 0.0f;
volatile float aligned_hall_angle_rad = 0.0f;
volatile float aligned_hall_angle_deg = 0.0f;

volatile float hall_zero_offset_rad = 0.0f;
volatile float hall_zero_offset_deg = 0.0f;

volatile float modulation_command = ALIGN_MODULATION;
volatile float hall_magnitude = 0.0f;

/* Debug watch: forced open-loop electrical frequency and resulting
 * mechanical RPM target, so you can see the ramp on the bench. */
volatile float open_loop_freq_hz = 0.0f;
volatile float open_loop_target_rpm = 0.0f;
volatile uint8_t handover_confirmed = 0u;

volatile uint8_t full_torque_mode = 0u;
volatile uint8_t hall_fault = 0u;
volatile uint32_t hall_fault_count = 0u;

volatile uint8_t driver_fault = 0u;
volatile uint32_t driver_fault_count = 0u;

volatile uint8_t motor_running = 0u;
volatile uint8_t motor_state = 0u;

#define MOTOR_STATE_IDLE            0u
#define MOTOR_STATE_ALIGN           1u
#define MOTOR_STATE_HALL_CAL        2u
#define MOTOR_STATE_OPEN_LOOP_RAMP  3u
#define MOTOR_STATE_TORQUE_RAMP     4u
#define MOTOR_STATE_CLOSED          5u
#define MOTOR_STATE_FAULT           6u

static uint32_t pwm_period = 65536u;

static uint32_t control_div_count = 0u;
static uint32_t speed_div_count = 0u;
static uint32_t torque_ramp_count = 0u;
static uint32_t heartbeat = 0u;

static float previous_hall_angle = 0.0f;
static float speed_angle_accumulator = 0.0f;
static float integrator = START_TORQUE_MODULATION;

static uint8_t closed_loop_enabled = 0u;
static uint8_t previous_full_torque_mode = 1u;

/* Open-loop ramp state. */
static float open_loop_angle_rad = ALIGN_ANGLE_RAD;
static uint32_t open_loop_ramp_count = 0u;
static uint32_t open_loop_elapsed_count = 0u;
static uint32_t handover_confirm_count = 0u;

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);

static float limit(float x, float lo, float hi);
static float wrap_pi(float x);
static float wrap_2pi(float x);
static uint32_t ccr(float duty);
static void phases(float a, float b, float c);
static void neutral(void);
static void drive_svpwm(float theta, float modulation);
static uint8_t hall_angle(float *theta);
static void speed_controller(void);
static void motor_align_and_calibrate(void);
static void motor_open_loop_start(void);
static void driver_enable(void);
static void driver_disable(void);
static uint8_t driver_fault_active(void);

/* ========================================================================== */
/* DRIVER                                                                     */
/* ========================================================================== */

static void driver_enable(void)
{
    HAL_GPIO_WritePin(DRVOFF_PORT, DRVOFF_PIN, GPIO_PIN_RESET);
}

static void driver_disable(void)
{
    HAL_GPIO_WritePin(DRVOFF_PORT, DRVOFF_PIN, GPIO_PIN_SET);
}

static uint8_t driver_fault_active(void)
{
    return HAL_GPIO_ReadPin(NFAULT_PORT, NFAULT_PIN) == GPIO_PIN_RESET;
}

/* ========================================================================== */
/* LIMIT / ANGLE WRAPPING                                                     */
/* ========================================================================== */

static float limit(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

static float wrap_pi(float x)
{
    while (x > PI_F) { x -= TWO_PI_F; }
    while (x < -PI_F) { x += TWO_PI_F; }
    return x;
}

static float wrap_2pi(float x)
{
    while (x >= TWO_PI_F) { x -= TWO_PI_F; }
    while (x < 0.0f) { x += TWO_PI_F; }
    return x;
}

/* ========================================================================== */
/* PWM                                                                        */
/* ========================================================================== */

static uint32_t ccr(float duty)
{
    duty = limit(duty, 0.01f, 0.99f);
    return (uint32_t)(duty * (float)pwm_period + 0.5f);
}

static void phases(float a, float b, float c)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr(c)); /* Phase A / PB7 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr(b)); /* Phase B / PB6 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr(a)); /* Phase C / PB5 */
}

static void neutral(void)
{
    phases(0.5f, 0.5f, 0.5f);
}

/* ========================================================================== */
/* SVPWM                                                                      */
/* ========================================================================== */

static void drive_svpwm(float theta, float modulation)
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

    modulation = limit(modulation, 0.0f, M_MAX);

    va = sinf(theta);
    vb = sinf(theta - 2.0f * PI_F / 3.0f);
    vc = sinf(theta + 2.0f * PI_F / 3.0f);

    vmax = va;
    if (vb > vmax) { vmax = vb; }
    if (vc > vmax) { vmax = vc; }

    vmin = va;
    if (vb < vmin) { vmin = vb; }
    if (vc < vmin) { vmin = vc; }

    common = -0.5f * (vmax + vmin);

    va += common;
    vb += common;
    vc += common;

    va *= modulation * SVPWM_GAIN;
    vb *= modulation * SVPWM_GAIN;
    vc *= modulation * SVPWM_GAIN;

    da = 0.5f + 0.5f * va;
    db = 0.5f + 0.5f * vb;
    dc = 0.5f + 0.5f * vc;

    phases(da, db, dc);
}

/* ========================================================================== */
/* HALL ANGLE                                                                 */
/* ========================================================================== */

static uint8_t hall_angle(float *theta)
{
    float a;
    float b;
    float c;
    float alpha;
    float beta;
    float mag_sq;
    float raw_theta;

    c = ((float)hall_ac_adc[0] - HA_OFFSET) * HA_GAIN; /* Hall A, PA4 */
    b = ((float)hall_b_adc - HB_OFFSET) * HB_GAIN;      /* Hall B, PA3 */
    a = ((float)hall_ac_adc[1] - HC_OFFSET) * HC_GAIN; /* Hall C, PA5 */

    alpha = (2.0f / 3.0f) * (a - 0.5f * b - 0.5f * c);
    beta = (2.0f / 3.0f) * SQRT3_2 * (b - c);

    mag_sq = alpha * alpha + beta * beta;
    hall_magnitude = sqrtf(mag_sq);

    if (mag_sq < HALL_MIN_MAG * HALL_MIN_MAG)
    {
        return 0u;
    }

    raw_theta = atan2f(beta, alpha);
    raw_theta *= HALL_DIRECTION;

    *theta = wrap_pi(raw_theta);

    return 1u;
}

/* ========================================================================== */
/* SPEED CONTROLLER (unchanged from baseline)                                */
/* ========================================================================== */

static void speed_controller(void)
{
    float error;
    float proposed_integrator;
    float raw_output;
    float output;

    error = speed_reference_rpm - measured_speed_rpm;
    speed_error_rpm = error;

    if (measured_speed_rpm < (speed_reference_rpm - FULL_TORQUE_BAND_RPM))
    {
        full_torque_mode = 1u;
        modulation_command = M_MAX;
        previous_full_torque_mode = 1u;
        return;
    }

    if (previous_full_torque_mode)
    {
        integrator = modulation_command - KP * error;
        integrator = limit(integrator, M_MIN, M_MAX);
        previous_full_torque_mode = 0u;
    }

    full_torque_mode = 0u;

    proposed_integrator = integrator + KI * error / PI_HZ;
    raw_output = KP * error + proposed_integrator;
    output = limit(raw_output, M_MIN, M_MAX);

    if (raw_output == output)
    {
        integrator = proposed_integrator;
    }
    else if (raw_output > M_MAX && error < 0.0f)
    {
        integrator = proposed_integrator;
    }
    else if (raw_output < M_MIN && error > 0.0f)
    {
        integrator = proposed_integrator;
    }

    integrator = limit(integrator, M_MIN, M_MAX);
    modulation_command = output;
}

/* ========================================================================== */
/* ALIGN + CALIBRATE HALL ZERO (unchanged from baseline)                     */
/* ========================================================================== */

static void motor_align_and_calibrate(void)
{
    uint32_t start;
    uint32_t sample_count;
    float theta;
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    float average_theta;

    motor_state = MOTOR_STATE_ALIGN;
    modulation_command = ALIGN_MODULATION;

    start = HAL_GetTick();

    while ((HAL_GetTick() - start) < ALIGN_TIME_MS)
    {
        drive_svpwm(ALIGN_ANGLE_RAD, ALIGN_MODULATION);
    }

    drive_svpwm(ALIGN_ANGLE_RAD, ALIGN_MODULATION);

    motor_state = MOTOR_STATE_HALL_CAL;

    for (sample_count = 0u; sample_count < ALIGN_HALL_SAMPLES; sample_count++)
    {
        if (hall_angle(&theta))
        {
            sin_sum += sinf(theta);
            cos_sum += cosf(theta);
        }

        HAL_Delay(ALIGN_HALL_SAMPLE_DELAY_MS);
    }

    if ((sin_sum * sin_sum + cos_sum * cos_sum) < 1.0f)
    {
        hall_fault = 1u;
        motor_state = MOTOR_STATE_FAULT;
        neutral();
        driver_disable();
        Error_Handler();
    }

    average_theta = atan2f(sin_sum, cos_sum);

    aligned_hall_angle_rad = average_theta;
    aligned_hall_angle_deg = average_theta * 180.0f / PI_F;

    hall_zero_offset_rad = wrap_pi(ALIGN_ANGLE_RAD - average_theta);
    hall_zero_offset_deg = hall_zero_offset_rad * 180.0f / PI_F;

    previous_hall_angle = average_theta;
    speed_angle_accumulator = 0.0f;
    measured_speed_rpm = 0.0f;
    raw_speed_rpm = 0.0f;
}

/* ========================================================================== */
/* START THE OPEN-LOOP RAMP                                                  */
/*                                                                            */
/* Continues the field EXACTLY where alignment left it (same angle, same     */
/* modulation) -- no step change -- then lets the TIM6 ISR ramp frequency    */
/* and modulation upward and manage handover into Hall closed loop.         */
/* ========================================================================== */

static void motor_open_loop_start(void)
{
    open_loop_angle_rad = ALIGN_ANGLE_RAD;
    open_loop_freq_hz = 0.0f;
    open_loop_target_rpm = OPEN_LOOP_HANDOVER_RPM;
    handover_confirmed = 0u;

    modulation_command = ALIGN_MODULATION;

    drive_svpwm(open_loop_angle_rad, modulation_command);

    open_loop_ramp_count = 0u;
    open_loop_elapsed_count = 0u;
    handover_confirm_count = 0u;

    torque_ramp_count = 0u;
    control_div_count = 0u;
    speed_div_count = 0u;
    hall_fault_count = 0u;
    driver_fault_count = 0u;

    integrator = START_TORQUE_MODULATION;
    previous_full_torque_mode = 1u;
    full_torque_mode = 0u;

    motor_state = MOTOR_STATE_OPEN_LOOP_RAMP;
    closed_loop_enabled = 1u;
    motor_running = 1u;

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================== */
/* TIM6 CONTROL ISR                                                           */
/* ========================================================================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    float theta;
    float delta;
    float calculated_rpm;
    float torque_advance_rad;
    float command_angle;
    float ramp;
    float slip_rpm;

    if (htim->Instance != TIM6)
    {
        return;
    }

    if (!closed_loop_enabled)
    {
        return;
    }

    control_div_count++;

    if (control_div_count < CONTROL_DIV)
    {
        return;
    }

    control_div_count = 0u;

    /* DRV8316 nFAULT. */
    if (driver_fault_active())
    {
        driver_fault_count++;

        if (driver_fault_count >= DRIVER_FAULT_LIMIT)
        {
            driver_fault = 1u;
            motor_state = MOTOR_STATE_FAULT;
            closed_loop_enabled = 0u;
            motor_running = 0u;

            neutral();
            driver_disable();

            return;
        }
    }
    else
    {
        driver_fault_count = 0u;
        driver_fault = 0u;
    }

    /* Hall -- measured regardless of state: needed for handover confirmation
     * during the open-loop ramp, and for commutation/PI once closed. */
    if (!hall_angle(&theta))
    {
        hall_fault_count++;

        if (hall_fault_count >= HALL_FAULT_LIMIT)
        {
            hall_fault = 1u;
            motor_state = MOTOR_STATE_FAULT;
            closed_loop_enabled = 0u;
            motor_running = 0u;

            neutral();
            driver_disable();
        }

        return;
    }

    hall_fault_count = 0u;
    hall_fault = 0u;
    electrical_angle_rad = theta;

    /* Speed estimate from Hall, same as baseline. */
    delta = wrap_pi(theta - previous_hall_angle);
    previous_hall_angle = theta;
    speed_angle_accumulator += delta;

    speed_div_count++;

    if (speed_div_count >= SPEED_DIV)
    {
        speed_div_count = 0u;

        calculated_rpm = speed_angle_accumulator * SPEED_HZ * 60.0f /
                         (TWO_PI_F * POLE_PAIRS);

        speed_angle_accumulator = 0.0f;
        calculated_rpm *= DIRECTION;

        raw_speed_rpm = calculated_rpm;

        measured_speed_rpm += SPEED_ALPHA * (calculated_rpm - measured_speed_rpm);

        if (motor_state == MOTOR_STATE_OPEN_LOOP_RAMP)
        {
            /*
             * Only start trusting the Hall speed once the forced field has
             * actually reached handover frequency -- before that the rotor
             * is still accelerating and raw_speed_rpm just lags the forced
             * ramp, which is expected and not a slip fault.
             */
            if (open_loop_ramp_count >= OPEN_LOOP_RAMP_STEPS)
            {
                slip_rpm = fabsf(raw_speed_rpm - open_loop_target_rpm);

                if (slip_rpm <= HANDOVER_SLIP_TOLERANCE_RPM)
                {
                    handover_confirm_count++;
                }
                else
                {
                    handover_confirm_count = 0u;
                }
            }
        }
        else if (motor_state == MOTOR_STATE_CLOSED)
        {
            speed_controller();
        }
    }

    /* Hall-derived closed-loop command angle -- computed every tick so it
     * is ready the instant handover occurs. */
    torque_advance_deg = limit(torque_advance_deg, 30.0f, 150.0f);
    torque_advance_rad = torque_advance_deg * PI_F / 180.0f;

    command_angle = theta + hall_zero_offset_rad + DIRECTION * torque_advance_rad;
    command_angle = wrap_2pi(command_angle);

    if (motor_state == MOTOR_STATE_OPEN_LOOP_RAMP)
    {
        open_loop_elapsed_count++;

        if (open_loop_elapsed_count >= OPEN_LOOP_TIMEOUT_TICKS)
        {
            /* Rotor never demonstrated synchronous rotation -- fault safe
             * instead of continuing to force a slipping/stalled field. */
            hall_fault = 1u;
            motor_state = MOTOR_STATE_FAULT;
            closed_loop_enabled = 0u;
            motor_running = 0u;

            neutral();
            driver_disable();

            return;
        }

        open_loop_ramp_count++;

        ramp = (float)open_loop_ramp_count / (float)OPEN_LOOP_RAMP_STEPS;
        ramp = limit(ramp, 0.0f, 1.0f);

        open_loop_freq_hz = ramp * OPEN_LOOP_HANDOVER_HZ;

        modulation_command = ALIGN_MODULATION +
                              (START_TORQUE_MODULATION - ALIGN_MODULATION) * ramp;

        open_loop_angle_rad = wrap_2pi(
            open_loop_angle_rad +
            DIRECTION * TWO_PI_F * open_loop_freq_hz / CONTROL_HZ
        );

        drive_angle_rad = open_loop_angle_rad;

        if (handover_confirm_count >= HANDOVER_CONFIRM_TICKS)
        {
            /*
             * Rotor confirmed synchronous with the forced field. Hand over
             * to Hall closed loop at the SAME modulation it is already
             * running at (START_TORQUE_MODULATION, reached at the end of
             * the ramp) and continue ramping that up to full torque exactly
             * like the baseline's torque-ramp phase -- but starting from an
             * already-spinning rotor instead of a standstill.
             */
            handover_confirmed = 1u;

            torque_ramp_count = 0u;
            integrator = START_TORQUE_MODULATION;
            previous_full_torque_mode = 1u;
            full_torque_mode = 1u;

            motor_state = MOTOR_STATE_TORQUE_RAMP;
        }

        drive_svpwm(open_loop_angle_rad, modulation_command);

        heartbeat++;

        if (heartbeat >= 2500u)
        {
            heartbeat = 0u;
            HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
        }

        return;
    }

    drive_angle_rad = command_angle;

    /* Torque ramp, same mechanics as baseline -- just reached from a
     * spinning rotor instead of a standstill. */
    if (motor_state == MOTOR_STATE_TORQUE_RAMP)
    {
        torque_ramp_count++;

        ramp = (float)torque_ramp_count / (float)FULL_TORQUE_RAMP_STEPS;
        ramp = limit(ramp, 0.0f, 1.0f);

        modulation_command = START_TORQUE_MODULATION +
                              (M_MAX - START_TORQUE_MODULATION) * ramp;

        full_torque_mode = 1u;

        if (torque_ramp_count >= FULL_TORQUE_RAMP_STEPS)
        {
            modulation_command = M_MAX;
            full_torque_mode = 1u;
            previous_full_torque_mode = 1u;
            motor_state = MOTOR_STATE_CLOSED;
        }
    }

    drive_svpwm(command_angle, modulation_command);

    heartbeat++;

    if (heartbeat >= 2500u)
    {
        heartbeat = 0u;
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
    }
}

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

    HAL_GPIO_WritePin(CAN_STBY_PORT, CAN_STBY_PIN, GPIO_PIN_SET);

    driver_disable();

    pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1u;

    neutral();

    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) { Error_Handler(); }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) { Error_Handler(); }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&hall_b_adc, 1u) != HAL_OK) { Error_Handler(); }
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)hall_ac_adc, 2u) != HAL_OK) { Error_Handler(); }

    HAL_Delay(100);

    motor_state = MOTOR_STATE_IDLE;
    neutral();

    driver_enable();

    HAL_Delay(10);
    HAL_Delay(2000);

    motor_align_and_calibrate();

    HAL_Delay(50);

    motor_open_loop_start();

    while (1)
    {
        HAL_Delay(100);
    }
}

/* ========================================================================== */
/* SYSTEM CLOCK                                                               */
/* ========================================================================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================== */
/* ADC1 -- PA3 = Hall B = ADC1_IN4                                           */
/* ========================================================================== */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = { 0 };

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================== */
/* ADC2 -- Rank 1 = PA4 = Hall A = ADC2_IN17, Rank 2 = PA5 = Hall C = IN13   */
/* ========================================================================== */

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = { 0 };

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation = 0;
    hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.ContinuousConvMode = ENABLE;
    hadc2.Init.NbrOfConversion = 2;
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

    sConfig.Channel = ADC_CHANNEL_17;
    sConfig.Rank = ADC_REGULAR_RANK_1;

    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_13;
    sConfig.Rank = ADC_REGULAR_RANK_2;

    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================== */
/* TIM3 -- PB5 = Phase C = TIM3_CH2                                          */
/* ========================================================================== */

static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_OC_InitTypeDef sConfigOC = { 0 };

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

    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim3);
}

/* ========================================================================== */
/* TIM4 -- PB6 = Phase B = TIM4_CH1, PB7 = Phase A = TIM4_CH2                */
/* ========================================================================== */

static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_OC_InitTypeDef sConfigOC = { 0 };

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

    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

    HAL_TIM_MspPostInit(&htim4);
}

/* ========================================================================== */
/* TIM6 -- 20 kHz control interrupt                                          */
/* ========================================================================== */

static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

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

    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
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
}

/* ========================================================================== */
/* GPIO                                                                       */
/* ========================================================================== */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(DRVOFF_PORT, DRVOFF_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAN_STBY_PORT, CAN_STBY_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DRVOFF_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DRVOFF_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NFAULT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(NFAULT_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAN_STBY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAN_STBY_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = STATUS_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STATUS_LED_PORT, &GPIO_InitStruct);
}

/* ========================================================================== */
/* ERROR                                                                      */
/* ========================================================================== */

void Error_Handler(void)
{
    closed_loop_enabled = 0u;
    motor_running = 0u;
    motor_state = MOTOR_STATE_FAULT;

    neutral();
    driver_disable();

    __disable_irq();

    while (1)
    {
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);

        for (volatile uint32_t delay = 0u; delay < 500000u; delay++)
        {
        }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
