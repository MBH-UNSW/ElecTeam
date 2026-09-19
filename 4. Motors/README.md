# Hall-Sensor Closed-Loop Motor Control

## Purpose

This feature implements closed-loop control of a 3-phase pump motor using three analog Hall sensors and space-vector PWM (SVPWM) on the STM32G431CBT6.

The controller:

- Aligns the rotor to a known electrical angle at startup.
- Measures and calibrates the Hall sensor electrical angle.
- Starts directly in Hall-based closed-loop control.
- Applies a configurable torque-producing electrical angle advance.
- Ramps the motor from moderate startup torque to maximum modulation.
- Estimates motor speed from the Hall electrical angle.
- Uses maximum torque below the target speed and PI regulation near the target RPM.
- Monitors Hall signal magnitude and enters a fault state if the Hall feedback becomes invalid.
- Toggles a status LED while the control loop is running.

The current software is configured for a **4-pole motor (2 pole pairs)** with a default speed reference of **1000 RPM**.

## Hardware

**MCU:** STM32G431CBT6  
**Package:** LQFP48

**Required hardware:**

- STM32G431CBT6-based motor control board
- 3-phase motor / pump motor
- 3-phase motor driver or inverter stage accepting three PWM commands
- Three analog Hall sensors positioned approximately 120 electrical degrees apart
- External HSE clock source / crystal as configured in CubeMX
- SWD programmer/debugger such as ST-LINK

### Connections

| Pin | Function | Description |
| --- | --- | --- |
| PB7 | TIM4_CH2 | Phase A PWM output |
| PB6 | TIM4_CH1 | Phase B PWM output |
| PB5 | TIM3_CH2 | Phase C PWM output |
| PA1 | ADC2_IN2 | Hall A analog input |
| PA6 | ADC2_IN3 | Hall B analog input |
| PA7 | ADC2_IN4 | Hall C analog input |
| PB3 | GPIO Output | Status / heartbeat LED |
| PA9 | GPIO Output | General-purpose output, currently unused by motor control |
| PA10 | GPIO Input | General-purpose input, currently unused by motor control |
| PF0 | RCC_OSC_IN | HSE oscillator input |
| PF1 | RCC_OSC_OUT | HSE oscillator output |

The CubeMX pinout screenshot also assigns **PA11/PA12 to FDCAN1 RX/TX**. FDCAN is not used or initialized by the current `main.c` motor-control implementation.

## CubeMX Configuration

### Clock

- HSE enabled.
- PLL enabled with HSE as the PLL source.
- PLLM = 6.
- PLLN = 85.
- PLLR = 2.
- AHB prescaler = 1.
- APB1 prescaler = 1.
- APB2 prescaler = 1.
- Voltage scaling set to `PWR_REGULATOR_VOLTAGE_SCALE1_BOOST`.

With a **24 MHz HSE**, this configuration produces a **170 MHz system clock**.

### ADC2

ADC2 is used to continuously sample the three analog Hall sensors.

- Resolution: 12-bit
- Scan conversion mode: Enabled
- Continuous conversion mode: Enabled
- Number of conversions: 3
- External trigger: Software start
- DMA continuous requests: Enabled
- Overrun behaviour: Data overwritten
- Oversampling: Disabled
- Sampling time: 47.5 ADC cycles
- Input mode: Single-ended

ADC conversion order:

| Rank | Pin | ADC Channel | Signal |
| --- | --- | --- | --- |
| 1 | PA1 | ADC2_IN2 | Hall A |
| 2 | PA6 | ADC2_IN3 | Hall B |
| 3 | PA7 | ADC2_IN4 | Hall C |

ADC2 DMA stores the three measurements continuously in `hall_adc[3]`.

### DMA

- DMA1 enabled.
- DMAMUX1 enabled.
- DMA1 Channel 1 interrupt enabled.
- Used for continuous ADC2 Hall sensor acquisition.

### TIM3

TIM3 generates the Phase C PWM signal.

- Prescaler: 0
- Counter mode: Up
- Period: 65535
- PWM mode: PWM1
- Channel 2 enabled
- Output pin: PB5 / TIM3_CH2

### TIM4

TIM4 generates the Phase A and Phase B PWM signals.

- Prescaler: 0
- Counter mode: Up
- Period: 65535
- PWM mode: PWM1
- Channel 1 enabled → PB6 / Phase B
- Channel 2 enabled → PB7 / Phase A

> **Note:** With a 170 MHz timer clock and a period of 65535, TIM3/TIM4 produce a PWM carrier frequency of approximately **2.59 kHz**. The **20 kHz** value in the control code refers to the TIM6 control interrupt rate, not the TIM3/TIM4 PWM carrier frequency.

### TIM6

TIM6 provides the motor-control interrupt timing.

- Prescaler: 169
- Period: 49
- Update interrupt enabled
- Interrupt rate: 20 kHz at a 170 MHz timer clock

The interrupt is divided in software:

- TIM6 ISR: 20 kHz
- Hall/angle control update: 5 kHz
- Speed update and PI controller: 1 kHz

### GPIO

- PB3 configured as Output Push-Pull for the status / heartbeat LED.
- PA9 configured as Output Push-Pull.
- PA10 configured as GPIO Input.
- PA1, PA6 and PA7 used as analog Hall inputs.
- PA8 is configured as analog GPIO in the generated code but is not currently used as one of the three Hall ADC channels.

## Code Functionality

### Startup Sequence

At startup the controller:

1. Starts the three PWM timer channels.
2. Calibrates ADC2 and starts continuous ADC acquisition using DMA.
3. Holds the motor outputs at a neutral 50% duty state.
4. Waits 2 seconds before starting the motor sequence.
5. Aligns the rotor to `0°` electrical using 60% modulation for 1.5 seconds.
6. Samples the Hall angle 100 times while the rotor remains aligned.
7. Calculates the Hall-to-stator electrical zero offset.
8. Starts Hall-based closed-loop commutation.
9. Applies a default +90° torque-producing electrical advance.
10. Ramps modulation from 55% to 100% over 1.2 seconds.
11. Runs at maximum modulation until the motor approaches the target speed.
12. Uses PI speed regulation within 30 RPM of the target.

### Hall Angle Calculation

`hall_angle()`:

- Reads the three ADC Hall measurements.
- Removes the configured Hall offsets.
- Applies individual Hall gain values.
- Performs a Clarke-style transformation to obtain `alpha` and `beta` components.
- Calculates the electrical rotor angle using `atan2f()`.
- Checks Hall vector magnitude before accepting the reading.

Current Hall calibration constants:

```c
HA_OFFSET = 1241
HB_OFFSET = 1241
HC_OFFSET = 1241

HA_GAIN = 1.0
HB_GAIN = 1.0
HC_GAIN = 1.0
```

These values should be recalibrated if the Hall sensors, magnet arrangement, signal conditioning, or mechanical alignment changes.

### SVPWM Output

`drive_svpwm()` generates three sinusoidal phase references separated by 120° and applies common-mode injection before converting them into PWM duty cycles.

PWM mapping:

- Phase A → PB7 / TIM4_CH2
- Phase B → PB6 / TIM4_CH1
- Phase C → PB5 / TIM3_CH2

### Speed Measurement

Motor speed is calculated from the change in Hall electrical angle:

```text
Electrical angle change
        ↓
Mechanical angle change using POLE_PAIRS
        ↓
RPM calculation
        ↓
Low-pass filtering
```

The current motor configuration uses:

```c
POLE_PAIRS = 2
TARGET_RPM = 1000 RPM
SPEED_ALPHA = 0.18
```

### Speed Control

The controller uses two operating regions:

- **Maximum torque mode:** 100% modulation while measured speed is more than 30 RPM below the target.
- **PI speed-control mode:** modulation is adjusted once the motor is close to the target speed.

Current PI gains:

```c
KP = 0.0020
KI = 0.0150
```

The modulation command is limited between:

```c
M_MIN = 0.15
M_MAX = 1.00
```

### Torque Advance

The default electrical torque advance is:

```c
DEFAULT_TORQUE_ADVANCE_DEG = 90.0f
```

During operation it is limited to the range **30° to 150°**.

### Hall Fault Detection

If the Hall vector magnitude is below the configured minimum, the sample is considered invalid.

- Minimum Hall magnitude: `40`
- Consecutive invalid sample limit: `20`

After the fault limit is reached, the controller:

- Disables closed-loop control.
- Sets the motor state to fault.
- Sets the phase outputs to the neutral 50% duty state.

### Status LED

PB3 is used as a heartbeat output and toggles every 2500 control updates. With the 5 kHz control loop, the LED changes state approximately every 0.5 seconds.

## Important Code Modifications

The generated CubeMX project contains additional application code for:

- Hall sensor angle reconstruction
- Automatic Hall electrical-zero calibration
- Rotor alignment at startup
- Direct Hall closed-loop startup
- SVPWM generation
- Torque-angle control
- Hall-based RPM estimation
- Low-pass speed filtering
- Maximum-torque startup
- PI speed regulation
- Hall sensor fault handling
- Status LED heartbeat

Most motor-control parameters are defined near the top of `main.c`, including:

```c
#define POLE_PAIRS                  2.0f
#define TARGET_RPM                  1000.0f
#define ALIGN_MODULATION            0.60f
#define ALIGN_TIME_MS               1500u
#define START_TORQUE_MODULATION     0.55f
#define FULL_TORQUE_RAMP_MS         1200u
#define DEFAULT_TORQUE_ADVANCE_DEG  90.0f
#define KP                          0.0020f
#define KI                          0.0150f
```

These parameters can be tuned for different motor, Hall sensor, pump load, and speed requirements.

## Build & Test

1. Open the project `.ioc` file in STM32CubeMX or STM32CubeIDE and verify the pinout and peripheral configuration.
2. Generate the STM32 project code if required.
3. Build the project.
4. Connect the ST-LINK programmer/debugger using SWD.
5. Connect the three Hall sensor outputs to PA1, PA6 and PA7.
6. Connect PB7, PB6 and PB5 to the corresponding Phase A, Phase B and Phase C motor-driver PWM inputs.
7. Connect the status LED circuitry to PB3 if it is not already present on the PCB.
8. Power the motor driver and controller using the required external supplies.
9. Flash the firmware to the STM32G431CBT6.
10. Keep the pump/motor mechanically clear during startup because the rotor will first align to a fixed electrical angle.
11. After the 2-second startup delay, verify that the rotor aligns, transitions into Hall closed-loop control, and accelerates toward the target RPM.
12. Use the debugger/watch window to monitor variables such as:
    - `measured_speed_rpm`
    - `raw_speed_rpm`
    - `speed_error_rpm`
    - `electrical_angle_rad`
    - `drive_angle_rad`
    - `hall_magnitude`
    - `hall_zero_offset_deg`
    - `modulation_command`
    - `motor_state`
    - `hall_fault`
13. Confirm that PB3 toggles during normal closed-loop operation.
14. Verify the three PWM outputs using an oscilloscope or logic analyser before connecting to the full-power motor stage.

## Pinout View + Clock Configuration Screenshots

### Pinout View

![STM32G431CBT6 Pinout](Pinout_View.png)

### Clock Configuration

Add the STM32CubeMX **Clock Configuration** screenshot to this folder and reference it here, for example:

```markdown
![STM32G431CBT6 Clock Configuration](Clock_Configuration.png)
```

## Notes

- The pinout screenshot shows FDCAN1 on PA11/PA12, but the current motor-control source does not initialize or use FDCAN1.
- PA8 is configured as analog GPIO in `main.c` but is not included in the three-channel ADC2 Hall conversion sequence.
- The TIM6 control ISR operates at 20 kHz, while the current TIM3/TIM4 PWM configuration corresponds to approximately 2.59 kHz at a 170 MHz timer clock.
- Hall offsets and PI gains are hardware-dependent and may need to be retuned when the motor, rotor magnets, Hall sensor placement, or pump load changes.
