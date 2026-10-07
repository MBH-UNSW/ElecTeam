/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : DRV8316Control_Pulsatile_v2.c
  * @brief          : Sinusoidal pulsatile flow pump controller (v2)
  ******************************************************************************
  *
  * What changed vs DRV8316Control_Pulsatile_Best.c, and why:
  *
  *   1. BASELINE STARTUP / TIMING. The Best/Pulsatile files stacked three
  *      untested changes on top of the hardware-proven baseline
  *      (DRV8316Control.c): an open-loop ramp with a slip-checked handover
  *      (fault + motor stop if the handover is not confirmed in 6 s), a fast
  *      atan2 approximation, and 10 kHz commutation. v2 goes back to the
  *      baseline's align -> Hall closed loop -> torque ramp start, exact
  *      atan2f, and 5 kHz commutation. The open-loop path is still in the
  *      file behind START_WITH_OPEN_LOOP (default 0).
  *
  *   2. FEEDFORWARD SPEED CONTROL during pulsation. The old controller was
  *      bang-bang (100 % modulation whenever below the target by >30 RPM)
  *      plus a slow PI whose integrator was reset to ~1.0 every time it left
  *      bang-bang. On a MOVING target that means: overshoot on every upstroke,
  *      then a long unwind. v2 instead sets modulation = reference / K, where
  *      K (RPM per unit modulation) is MEASURED by the controller itself
  *      while it holds the mean speed, and the PI only trims the remainder.
  *
  *   3. SAFE DEFAULT PROFILE: 1800..3000 RPM at 60 BPM. 3000 RPM is the speed
  *      you have already run (~3 L/min); the old 3500 RPM peak was never
  *      shown to be reachable. Widen once the waveform tracks.
  *
  *   4. OPTIONAL ACTIVE BRAKING (BRAKE_ENABLED, default 0). If the motor only
  *      coasts down, the trough of each beat can never be reached, no matter
  *      the controller. Braking drives the torque angle negative while the
  *      speed is well above the reference. This pushes energy back into the
  *      supply: only enable it on a battery or a supply/bus that can absorb
  *      it, otherwise the bus voltage will rise.
  *
  * TO SEE WHAT IS HAPPENING (add to the debugger watch window):
  *   speed_reference_rpm vs measured_speed_rpm   -> is the beat tracked?
  *   pulse_calibrated, rpm_per_mod               -> 0 means it never settled
  *                                                  at the mean speed
  *   modulation_command at M_MAX                 -> asking for more than the
  *                                                  motor can give: lower
  *                                                  SYSTOLIC_RPM
  *   modulation_command at M_MIN                 -> motor cannot slow down
  *                                                  fast enough: raise the
  *                                                  DIASTOLIC_RPM, lower the
  *                                                  BPM, or try BRAKE_ENABLED
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

#include <math.h>
#include <stdint.h>

/* ========================================================================== */
/* OPTIMIZATION SWITCHES                                                     */
/* ========================================================================== */

/* Set to 0 to fall back to exact atan2f() for A/B comparison on the bench. */
#define USE_FAST_TRIG               0

/* Recompute hall_magnitude (debug/telemetry only) every N control ticks. */
#define HALL_MAG_DEBUG_DIV          8u

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

/* 25 ticks @ 5 kHz = 5 ms. */
#define DRIVER_FAULT_LIMIT          25u

/* ========================================================================== */
/* MOTOR                                                                      */
/* ========================================================================== */

#define POLE_PAIRS                  2.0f
#define DIRECTION                   1.0f
#define HALL_DIRECTION              1.0f

/* ========================================================================== */
/* PULSATILE FLOW -- EDIT THESE TO MATCH THE DESIRED CARDIAC PROFILE         */
/* ========================================================================== */

/* Set to 0 to disable pulsation and run continuous flow at MEAN_RPM
 * instead, for an apples-to-apples bench comparison. */
#define PULSATILE_MODE_ENABLED      1u

/* Heart rate. Raise/lower to see the pump cycle faster/slower; watch
 * raw_speed_rpm on the debugger to confirm the motor can actually track
 * this rate before trusting the flow numbers. */
#define PULSE_RATE_BPM              60.0f

/* Trough (diastole) and peak (systole) target RPM. Start with a SMALL gap
 * between these two (e.g. 500 RPM) to confirm the PI loop tracks a moving
 * target at all, then widen the gap toward your real target waveform.
 * Watch speed_error_rpm / full_torque_mode -- if full_torque_mode is stuck
 * at 1 for most of the cycle, the gap is too large (or PULSE_RATE_BPM too
 * fast) for the motor/pump inertia to keep up. */
#define DIASTOLIC_RPM               1800.0f
#define SYSTOLIC_RPM                3000.0f

#define MEAN_RPM                    (0.5f * (DIASTOLIC_RPM + SYSTOLIC_RPM))
#define PULSE_AMPLITUDE_RPM         (0.5f * (SYSTOLIC_RPM - DIASTOLIC_RPM))

/* Startup still targets the mean RPM via the shared open-loop + torque-ramp
 * code below, same as the continuous-flow firmware. */
#define TARGET_RPM                  MEAN_RPM

/* How long the reference holds flat at MEAN_RPM after reaching
 * MOTOR_STATE_CLOSED before pulsation begins. Increase this if you can
 * see (on measured_speed_rpm) that the torque-ramp-to-PI transition still
 * has overshoot/ringing in it when pulsation starts. */
#define PULSE_SETTLE_MS             1500u

#define PULSE_SETTLE_TICKS \
    ((uint32_t)((SPEED_HZ * (float)PULSE_SETTLE_MS) / 1000.0f))

/* Number of heartbeats over which the swing amplitude ramps from 0 to
 * full, once pulsation starts. Each beat is still a pure cosine the whole
 * time -- only the depth of the swing grows. Increase for a gentler
 * introduction of pulsation; set to 1 for an almost-immediate full-depth
 * first beat. Watch pulse_amplitude_scale_debug in the debugger to see the
 * ramp-in happening. */
#define PULSE_AMPLITUDE_RAMP_BEATS  3.0f

#define FULL_TORQUE_BAND_RPM        30.0f

/* 0 = baseline start (align, then Hall closed loop + torque ramp), the
 * start you have already run on the pump. 1 = open-loop ramp + handover. */
#define START_WITH_OPEN_LOOP        0u

/* The mean speed counts as "settled" only when the error stays within this
 * band, in PI mode (not full torque), for PULSE_SETTLE_MS in a row. */
#define PULSE_SETTLE_BAND_RPM       80.0f

/* Pulsation trim PI (feedforward does the bulk of the work). The integrator
 * is clamped to +/- PULSE_TRIM_LIMIT of modulation. These are starting
 * values, not bench-tuned. If speed_error_rpm rings, lower PULSE_KI. If it
 * lags and stays one-sided, raise it. */
#define PULSE_KP                    0.0020f
#define PULSE_KI                    0.0400f
#define PULSE_TRIM_LIMIT            0.25f

/* Active braking, see header. 0 = off (recommended until the bus is known
 * to absorb regeneration). */
#define BRAKE_ENABLED               0u
#define BRAKE_BAND_RPM              150.0f
#define BRAKE_ADVANCE_DEG           270.0f
#define BRAKE_MODULATION            0.35f

/* ========================================================================== */
/* CONTROL TIMING                                                             */
/* ========================================================================== */

#define ISR_HZ                      20000.0f
#define CONTROL_DIV                 4u   /* 5 kHz, as baseline */
#define CONTROL_HZ                  (ISR_HZ / (float)CONTROL_DIV)
#define SPEED_DIV                   5u   /* 1 kHz, as baseline */
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
/* OPEN-LOOP STARTUP RAMP                                                    */
/* ========================================================================== */

/* TO TEST: if handover_confirmed never reaches 1, try LOWERING this --
 * the rotor may not be able to follow the forced field this fast yet. */
#define OPEN_LOOP_HANDOVER_RPM      400.0f

#define OPEN_LOOP_HANDOVER_HZ       (OPEN_LOOP_HANDOVER_RPM * POLE_PAIRS / 60.0f)

/* Main "how gentle is the startup" knob -- increase if you still feel a
 * jerk at power-on. */
#define OPEN_LOOP_RAMP_MS           2000u

#define OPEN_LOOP_RAMP_STEPS \
    ((uint32_t)((CONTROL_HZ * (float)OPEN_LOOP_RAMP_MS) / 1000.0f))

/* TO TEST: widen HANDOVER_SLIP_TOLERANCE_RPM if handover never confirms
 * despite the rotor visibly spinning fine. */
#define HANDOVER_SLIP_TOLERANCE_RPM 100.0f
#define HANDOVER_CONFIRM_MS         200u

#define HANDOVER_CONFIRM_TICKS \
    ((uint32_t)((SPEED_HZ * (float)HANDOVER_CONFIRM_MS) / 1000.0f))

#define OPEN_LOOP_TIMEOUT_MS        6000u

#define OPEN_LOOP_TIMEOUT_TICKS \
    ((uint32_t)((CONTROL_HZ * (float)OPEN_LOOP_TIMEOUT_MS) / 1000.0f))

/* ========================================================================== */
/* STARTUP TORQUE (Hall closed-loop torque ramp)                            */
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

/* 20 ticks @ 5 kHz = 4 ms. */
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

/* Runtime-tunable copies of the pulse profile, watchable/settable live
 * from the debugger without reflashing. */
volatile float pulse_rate_bpm = PULSE_RATE_BPM;
volatile float diastolic_rpm = DIASTOLIC_RPM;
volatile float systolic_rpm = SYSTOLIC_RPM;

volatile float pulse_phase_rad = 0.0f;
volatile float pulse_amplitude_scale_debug = 0.0f;
volatile uint8_t pulse_active = 0u;

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

static uint32_t hall_mag_debug_count = 0u;

static float previous_hall_angle = 0.0f;
static float speed_angle_accumulator = 0.0f;
static float integrator = START_TORQUE_MODULATION;

static uint8_t closed_loop_enabled = 0u;
static uint8_t previous_full_torque_mode = 1u;

static float open_loop_angle_rad = ALIGN_ANGLE_RAD;
static uint32_t open_loop_ramp_count = 0u;
static uint32_t open_loop_elapsed_count = 0u;
static uint32_t handover_confirm_count = 0u;

/* Pulsatile reference state. */
static uint32_t pulse_settle_count = 0u;
static uint32_t pulse_cycle_count = 0u;

/* Feedforward calibration. */
volatile uint8_t pulse_calibrated = 0u;
volatile float rpm_per_mod = 0.0f;
volatile uint8_t braking_active = 0u;
static float mod_avg = 0.0f;
static float pulse_trim = 0.0f;

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
static void update_pulse_reference(void);
static void speed_controller(void);
static void motor_align_and_calibrate(void);
static void motor_open_loop_start(void);
static void driver_enable(void);
static void driver_disable(void);
static uint8_t driver_fault_active(void);

#if USE_FAST_TRIG
static float fast_atan2f(float y, float x);
#endif

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
/* LIMIT                                                                      */
/* ========================================================================== */

static float limit(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

/* ========================================================================== */
/* ANGLE WRAPPING -- fmodf based, O(1) instead of a while-loop                */
/* ========================================================================== */

static float wrap_pi(float x)
{
    float y = fmodf(x + PI_F, TWO_PI_F);

    if (y < 0.0f)
    {
        y += TWO_PI_F;
    }

    return y - PI_F;
}

static float wrap_2pi(float x)
{
    float y = fmodf(x, TWO_PI_F);

    if (y < 0.0f)
    {
        y += TWO_PI_F;
    }

    return y;
}

/* ========================================================================== */
/* FAST ATAN2                                                                 */
/*                                                                            */
/* Octant minimax approximation. Max error ~0.0038 rad (~0.22 degrees        */
/* electrical) -- smaller than this design's own Hall angular noise.        */
/* ========================================================================== */

#if USE_FAST_TRIG
static float fast_atan2f(float y, float x)
{
    const float QUARTER_PI = PI_F / 4.0f;
    const float THREE_QUARTER_PI = 3.0f * PI_F / 4.0f;

    float abs_y = fabsf(y) + 1.0e-10f;
    float r;
    float angle;

    if (x >= 0.0f)
    {
        r = (x - abs_y) / (x + abs_y);
        angle = QUARTER_PI - QUARTER_PI * r;
    }
    else
    {
        r = (x + abs_y) / (abs_y - x);
        angle = THREE_QUARTER_PI - QUARTER_PI * r;
    }

    return (y < 0.0f) ? -angle : angle;
}
#endif

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
/* SVPWM -- one sinf() + one cosf() call, not three sinf() calls             */
/* ========================================================================== */

static void drive_svpwm(float theta, float modulation)
{
    float s;
    float c;
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

    s = sinf(theta);
    c = cosf(theta);

    va = s;
    vb = -0.5f * s - SQRT3_2 * c;
    vc = -0.5f * s + SQRT3_2 * c;

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

    hall_mag_debug_count++;

    if (hall_mag_debug_count >= HALL_MAG_DEBUG_DIV)
    {
        hall_mag_debug_count = 0u;
        hall_magnitude = sqrtf(mag_sq);
    }

    if (mag_sq < HALL_MIN_MAG * HALL_MIN_MAG)
    {
        return 0u;
    }

#if USE_FAST_TRIG
    raw_theta = fast_atan2f(beta, alpha);
#else
    raw_theta = atan2f(beta, alpha);
#endif

    raw_theta *= HALL_DIRECTION;

    *theta = wrap_pi(raw_theta);

    return 1u;
}

/* ========================================================================== */
/* PULSATILE SPEED REFERENCE                                                  */
/*                                                                            */
/* Called once per speed-loop tick (1 kHz) while in MOTOR_STATE_CLOSED.      */
/*                                                                            */
/* Phase 1 (PULSE_SETTLE_TICKS): hold flat at MEAN_RPM so the PI controller */
/* settles from the torque ramp before a sinusoidal swing is introduced.    */
/*                                                                            */
/* Phase 2: pulsation begins. Swing amplitude ramps from 0 to full over      */
/* PULSE_AMPLITUDE_RAMP_BEATS complete cycles -- each individual beat stays  */
/* a pure cosine shape throughout, only its depth grows beat by beat.       */
/* ========================================================================== */

static void update_pulse_reference(void)
{
    float mean_rpm;
    float amplitude_rpm;
    float dphase;
    float raw_phase;
    float amplitude_scale;

    mean_rpm = 0.5f * (diastolic_rpm + systolic_rpm);
    amplitude_rpm = 0.5f * (systolic_rpm - diastolic_rpm);

    if (!PULSATILE_MODE_ENABLED)
    {
        speed_reference_rpm = mean_rpm;
        return;
    }

    if (!pulse_active)
    {
        speed_reference_rpm = mean_rpm;

        /*
         * Count settle time only while the PI (not full-torque) is holding
         * the mean speed inside the band, and modulation is not pinned at a
         * limit. Anything else restarts the count, so pulsation never begins
         * on top of a transient, and never begins at all if the mean speed
         * is not reachable (pulse_active stays 0 -- that is the symptom).
         */
        if (!full_torque_mode &&
            fabsf(speed_error_rpm) < PULSE_SETTLE_BAND_RPM &&
            modulation_command > (M_MIN + 0.02f) &&
            modulation_command < (M_MAX - 0.02f))
        {
            pulse_settle_count++;
            mod_avg += 0.02f * (modulation_command - mod_avg);
        }
        else
        {
            pulse_settle_count = 0u;
            mod_avg = modulation_command;
        }

        if (pulse_settle_count >= PULSE_SETTLE_TICKS)
        {
            /* Measure K = RPM per unit modulation at the operating point. */
            rpm_per_mod = measured_speed_rpm / mod_avg;
            pulse_trim = 0.0f;
            pulse_calibrated = 1u;

            pulse_active = 1u;
            pulse_phase_rad = 0.0f; /* begin each run at the diastolic trough */
            pulse_cycle_count = 0u;
        }

        return;
    }

    dphase = TWO_PI_F * (pulse_rate_bpm / 60.0f) / SPEED_HZ;

    raw_phase = pulse_phase_rad + dphase;

    if (raw_phase >= TWO_PI_F)
    {
        pulse_cycle_count++;
    }

    pulse_phase_rad = wrap_2pi(raw_phase);

    amplitude_scale = ((float)pulse_cycle_count + pulse_phase_rad / TWO_PI_F) /
                       PULSE_AMPLITUDE_RAMP_BEATS;

    amplitude_scale = limit(amplitude_scale, 0.0f, 1.0f);

    pulse_amplitude_scale_debug = amplitude_scale;

    /* phase = 0 -> diastolic trough, phase = pi -> systolic peak. */
    speed_reference_rpm = mean_rpm - amplitude_rpm * amplitude_scale * cosf(pulse_phase_rad);
}

/* ========================================================================== */
/* SPEED CONTROLLER (unchanged from baseline)                                */
/* ========================================================================== */

/*
 * Pulsation controller: modulation = reference / K  (feedforward)
 *                                  + KP * error + trim (PI on the remainder).
 * No bang-bang and no integrator reset, so a moving reference does not
 * produce an overshoot on every upstroke.
 */
static void pulse_speed_controller(void)
{
    float error;
    float feedforward;
    float raw_output;
    float output;

    error = speed_reference_rpm - measured_speed_rpm;
    speed_error_rpm = error;

    full_torque_mode = 0u;
    previous_full_torque_mode = 0u;

#if BRAKE_ENABLED
    if (measured_speed_rpm > (speed_reference_rpm + BRAKE_BAND_RPM))
    {
        braking_active = 1u;
    }
    else if (measured_speed_rpm < (speed_reference_rpm + 0.5f * BRAKE_BAND_RPM))
    {
        braking_active = 0u;
    }

    if (braking_active)
    {
        modulation_command = BRAKE_MODULATION;
        return;
    }
#endif

    feedforward = speed_reference_rpm / rpm_per_mod;
    raw_output = feedforward + PULSE_KP * error + pulse_trim;
    output = limit(raw_output, M_MIN, M_MAX);

    if (raw_output == output ||
        (raw_output > M_MAX && error < 0.0f) ||
        (raw_output < M_MIN && error > 0.0f))
    {
        pulse_trim += PULSE_KI * error / PI_HZ;
        pulse_trim = limit(pulse_trim, -PULSE_TRIM_LIMIT, PULSE_TRIM_LIMIT);
    }

    modulation_command = output;
}

static void speed_controller(void)
{
    float error;
    float proposed_integrator;
    float raw_output;
    float output;

    if (pulse_active && pulse_calibrated)
    {
        pulse_speed_controller();
        return;
    }

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
/* ALIGN + CALIBRATE HALL ZERO                                               */
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

    average_theta = atan2f(sin_sum, cos_sum); /* calibration-time only: exact atan2f */

    aligned_hall_angle_rad = average_theta;
    aligned_hall_angle_deg = average_theta * 180.0f / PI_F;

    hall_zero_offset_rad = wrap_pi(ALIGN_ANGLE_RAD - average_theta);
    hall_zero_offset_deg = hall_zero_offset_rad * 180.0f / PI_F;

    previous_hall_angle = average_theta;
    speed_angle_accumulator = 0.0f;
    measured_speed_rpm = 0.0f;
    raw_speed_rpm = 0.0f;

    pulse_phase_rad = 0.0f;
    pulse_active = 0u;
    pulse_settle_count = 0u;
    pulse_cycle_count = 0u;
    pulse_calibrated = 0u;
    rpm_per_mod = 0.0f;
    pulse_trim = 0.0f;
    mod_avg = 0.0f;
    braking_active = 0u;
}

/* ========================================================================== */
/* START THE OPEN-LOOP RAMP                                                  */
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

    if (START_WITH_OPEN_LOOP)
    {
        motor_state = MOTOR_STATE_OPEN_LOOP_RAMP;
    }
    else
    {
        /* Baseline start: straight into Hall closed loop + torque ramp. */
        handover_confirmed = 1u;
        full_torque_mode = 1u;
        modulation_command = START_TORQUE_MODULATION;
        motor_state = MOTOR_STATE_TORQUE_RAMP;
    }

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

    /* Hall. */
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
            /*
             * Sweep the reference sinusoidally (with settle delay and
             * amplitude ramp-in) before the existing PI controller runs
             * against it.
             */
            update_pulse_reference();

            speed_controller();
        }
    }

    torque_advance_deg = limit(torque_advance_deg, 30.0f, 150.0f);

    if (braking_active)
    {
        torque_advance_rad = BRAKE_ADVANCE_DEG * PI_F / 180.0f;
    }
    else
    {
        torque_advance_rad = torque_advance_deg * PI_F / 180.0f;
    }

    command_angle = theta + hall_zero_offset_rad + DIRECTION * torque_advance_rad;
    command_angle = wrap_2pi(command_angle);

    if (motor_state == MOTOR_STATE_OPEN_LOOP_RAMP)
    {
        open_loop_elapsed_count++;

        if (open_loop_elapsed_count >= OPEN_LOOP_TIMEOUT_TICKS)
        {
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
