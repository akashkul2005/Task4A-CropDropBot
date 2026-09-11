# STM32 PID Line Follower — CropDrop Bot Task 4A

A real-time **embedded line-following robot** implemented on the **STM32F103RB** using **Embedded C and STM32 HAL**. The system reads a 5-channel analog IR sensor array through **ADC + DMA**, calculates line-position error, applies a PID controller, and drives two DC motors through PWM. Junction and stop conditions are handled as part of the control loop, with UART telemetry for debugging.

> **Focus:** Embedded C • STM32 • ADC/DMA • PWM • PID Control • Motor Control • UART Debugging

---

## Project Overview

The robot is designed to follow a **white line on a dark surface** and navigate a predefined arena. The firmware continuously:

1. Samples five IR sensors using **ADC1 in continuous scan mode**.
2. Transfers sensor readings into memory using **DMA**.
3. Converts the sensor pattern into a line-position error from **-2 to +2**.
4. Calculates PID correction.
5. Adjusts left/right motor PWM to keep the robot centered on the line.
6. Detects predefined junction patterns and performs 90° turns.
7. Detects the stop condition and disables motor output.
8. Sends raw sensor and state information over **USART2** for debugging.

---

## System Architecture

```text
        5-Channel IR Sensor Array
                  │
                  ▼
        STM32F103RB ADC1 + DMA
                  │
                  ▼
          Line Error Estimation
             (-2 ... +2)
                  │
                  ▼
            PID Controller
                  │
           ┌──────┴──────┐
           ▼             ▼
      Left PWM       Right PWM
           │             │
           └──────┬──────┘
                  ▼
            Motor Driver
                  │
                  ▼
          Differential Drive

       UART2 ──► Debug Telemetry
```

---

## Hardware

| Component | Implementation |
|---|---|
| Microcontroller | STM32F103RB |
| Line Sensors | 5-channel analog IR sensor array |
| ADC | ADC1, 5-channel scan, continuous conversion |
| Data Transfer | DMA for ADC buffer updates |
| Motor Driver | Dual H-Bridge (L298N or equivalent) |
| Motors | 2 × DC geared motors, differential drive |
| PWM | TIM2 and TIM4 |
| Debug Interface | USART2, 115200 baud |
| Development IDE | STM32CubeIDE |

### Sensor Mapping

The firmware stores the five ADC readings in `adcBuffer[0..4]`.

| Sensor | ADC Channel | Line Error |
|---|---:|---:|
| S0 | ADC Channel 10 | -2 |
| S1 | ADC Channel 11 | -1 |
| S2 | ADC Channel 12 | 0 |
| S3 | ADC Channel 8 | +1 |
| S4 | ADC Channel 9 | +2 |

The center sensor has priority when multiple normal line sensors detect white.

---

## PID Control

The line position is represented as a discrete error:

```text
S0 → -2     S1 → -1     S2 → 0     S3 → +1     S4 → +2
```

The controller calculates:

```text
Integral   = Integral + Error
Derivative = Error - PreviousError

Correction = Kp × Error
           + Ki × Integral
           + Kd × Derivative
```

Motor commands are then generated as:

```text
Left  = BASE_SPEED + LEFT_BIAS  - Correction
Right = BASE_SPEED + RIGHT_BIAS + Correction
```

Motor commands are clamped to the valid **0–100** range before being converted to the timer compare value.

### Current Firmware Parameters

```c
float Kp = 30.0;
float Ki = 0.0;
float Kd = 0.0;

uint16_t BASE_SPEED = 25;

int LEFT_BIAS  = 0;
int RIGHT_BIAS = 2;
```

These values are hardware-tuned parameters and can be adjusted for different motor, battery, sensor, and track characteristics.

---

## Junction & Stop Detection

The firmware checks sensor combinations before executing the normal PID update.

| Sensor Pattern | Interpretation | Action |
|---|---|---|
| S0 + S1 + S2 = WHITE | Right junction | Execute 90° right turn |
| S2 + S3 + S4 = WHITE | Left junction | Execute 90° left turn |
| S0 + S4 = WHITE | Stop condition | Stop both motors |

This priority-based logic prevents the PID controller from fighting the intentional turn manoeuvre.

---

## PWM Motor Control

The firmware uses two STM32 timers for motor control:

- **TIM2** → left motor PWM
- **TIM4** → right motor PWM
- PWM compare values are generated from the requested 0–100 speed command.
- `Motor_Stop()` sets all motor PWM outputs to zero.

The implementation uses a differential-drive approach, where the relative speed of the two motors controls the robot's steering.

---

## UART Debugging

`printf()` is redirected to **USART2** so sensor and control information can be monitored during testing.

Example telemetry:

```text
SYSTEM STARTED
RAW  : 1200 3800 4500 1100  900 | BW : B B W B B
RAW  :  900  800 4600 4700 4800 | BW : B B W W W
JUNCTION: LEFT TURN
STOP CONDITION DETECTED
```

This makes it easier to verify sensor thresholds, line position, junction detection, and robot behaviour without a debugger.

---

## Project Structure

```text
Task4A-CropDropBot/
└── Code/
    └── 2540#2_CB_TASK4A_#2540/
        └── last/
            ├── Core/
            │   ├── Inc/
            │   │   ├── main.h
            │   │   ├── stm32f1xx_hal_conf.h
            │   │   └── stm32f1xx_it.h
            │   ├── Src/
            │   │   ├── main.c
            │   │   ├── stm32f1xx_hal_msp.c
            │   │   ├── stm32f1xx_it.c
            │   │   ├── syscalls.c
            │   │   ├── sysmem.c
            │   │   └── system_stm32f1xx.c
            │   └── Startup/
            │       └── startup_stm32f103rbtx.s
            ├── Debug/
            ├── .cproject
            ├── .mxproject
            └── .project
└── README.md
```

The main application logic is located in:

```text
Code/2540#2_CB_TASK4A_#2540/last/Core/Src/main.c
```

---

## Getting Started

### 1. Clone

```bash
git clone https://github.com/akashkul2005/Task4A-CropDropBot.git
cd Task4A-CropDropBot
```

### 2. Import into STM32CubeIDE

Open **STM32CubeIDE** and import the existing Eclipse project from the `last` project directory.

```text
File → Import → Existing Projects into Workspace
```

Select:

```text
Code/2540#2_CB_TASK4A_#2540/last
```

### 3. Build and Flash

1. Connect the STM32 board through ST-Link.
2. Build the project in STM32CubeIDE.
3. Flash/run the firmware.
4. Open a serial terminal at **115200 baud**.
5. Place the robot on the white-line test track.

---

## Tuning Guide

| Symptom | Possible adjustment |
|---|---|
| Robot oscillates | Reduce `Kp` |
| Robot reacts too slowly | Increase `Kp` |
| Robot has steady offset | Introduce a small `Ki` |
| Robot overshoots | Introduce/tune `Kd` |
| Robot consistently drifts | Adjust `LEFT_BIAS` / `RIGHT_BIAS` |
| 90° turn is too short/long | Tune the turn timing in the turn functions |
| Line is detected incorrectly | Recalibrate `IR_THRESHOLD` |

Current threshold:

```c
#define IR_THRESHOLD 4000
```

> Sensor polarity and threshold values depend on the physical IR sensor array and track surface.

---

## Key Embedded Concepts Demonstrated

- **Embedded C programming**
- **STM32 HAL** peripheral configuration
- **ADC multi-channel scanning**
- **DMA-based sensor acquisition**
- **PWM motor control**
- **PID feedback control**
- **Differential-drive motion control**
- **GPIO configuration**
- **UART/USART debugging**
- **Real-time sensor processing**
- **Embedded system parameter tuning**

---

## Demo

A recorded demonstration of the line-following robot is available below:

[![PID Line Follower Demo](https://img.youtube.com/vi/4LUhw2QmUVs/0.jpg)](https://www.youtube.com/watch?v=4LUhw2QmUVs)

---

## Context

This project was developed as **Task 4A of the e-Yantra Robotics Competition (eYRC) CropDrop Bot theme, 2025–26**.

The implementation focuses specifically on the embedded control and firmware side of the robot: sensor acquisition, feedback control, motor actuation, junction handling, and debugging.

---

## Author

**Akash Kulkarni**  
Electronics & Telecommunication Engineering  
GitHub: [@akashkul2005](https://github.com/akashkul2005)

---

## License

This project is shared for educational and portfolio purposes. Please respect the original e-Yantra competition rules and intellectual-property requirements when reusing competition material.
