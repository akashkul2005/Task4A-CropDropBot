# 🤖 STM32 PID Line Follower — Task 4A

A PID-controlled line following robot built on the **STM32F103RB** microcontroller using the STM32 HAL framework. The bot autonomously follows a white line, detects junctions (left/right turns), and halts at a defined stop condition.

---

## 🎥 Output Demo

[![PID Line Follower Demo](https://img.youtube.com/vi/4LUhw2QmUVs/0.jpg)](https://www.youtube.com/watch?v=4LUhw2QmUVs)

> Click the thumbnail above to watch the bot in action!

---

## 📌 Task Objective

Design and implement a **PID-based line following algorithm** for a custom-built LineFollower Bot that:
- Follows a **white line** on a dark surface
- Navigates **90° left and right turns** at junctions
- Stops at a defined **endpoint (B)** on the arena

---

## 🗺️ Arena Layout

The bot starts at point **A** and must reach point **B** by following the white line path.

![Arena Layout](arena.png)

> **Figure 1:** Starting point **A** (right side) → Ending point **B** (left side) — eYRC 25-26 CropDrop Bot Arena

---

## 🛠️ Hardware

| Component | Details |
|---|---|
| Microcontroller | STM32F103RB (Blue Pill) |
| IR Sensors | 5-channel analog IR sensor array |
| ADC | ADC1 with DMA (channels 8–12) |
| Motor Driver | Dual H-Bridge (L298N or similar) |
| PWM Timers | TIM2 (Left Motor), TIM4 (Right Motor) |
| Debug Interface | USART2 @ 115200 baud |

### Sensor Mapping (ADC Channels)

```
Sensor:   [0]   [1]   [2]   [3]   [4]
           ←  Left     Center    Right →
ADC CH:   CH10  CH11  CH12  CH8   CH9
```

---

## ⚙️ PID Algorithm

The core control loop calculates a correction value from sensor error and applies it to motor speeds:

```
Error   →  getLineError()     →  [-2, -1, 0, +1, +2]
Correct →  Kp*e + Ki*∫e + Kd*de/dt
Speed   →  Left  = BASE + LEFT_BIAS  - correction
           Right = BASE + RIGHT_BIAS + correction
```

### Tunable Parameters

```c
float Kp        = 30.0;   // Proportional gain
float Ki        = 0.0;    // Integral gain
float Kd        = 0.0;    // Derivative gain
uint16_t BASE_SPEED = 25; // Base motor speed (0–100%)
int LEFT_BIAS   = 0;      // Hardware bias correction (left)
int RIGHT_BIAS  = 2;      // Hardware bias correction (right)
```

### Error Mapping

| Active Sensor | Error Value | Meaning |
|---|---|---|
| Sensor 2 (center) | `0` | On track |
| Sensor 1 | `-1` | Slightly left |
| Sensor 0 | `-2` | Far left |
| Sensor 3 | `+1` | Slightly right |
| Sensor 4 | `+2` | Far right |
| None | `99` (hold last) | Line lost |

---

## 🔀 Junction & Stop Detection

| Condition | Sensors | Action |
|---|---|---|
| Right Turn | S0 + S1 + S2 all WHITE | Execute 90° right turn |
| Left Turn | S2 + S3 + S4 all WHITE | Execute 90° left turn |
| Stop | S0 + S4 both WHITE | Halt motors |

---

## 📂 Project Structure

```
Task4a_PID/
├── Core/
│   ├── Inc/
│   │   └── main.h
│   ├── Src/
│   │   └── main.c          ← Main logic lives here
│   ├── Startup/
│   │   └── startup_stm32xxxx.s
│   └── system_stm32xxxx.c
├── Drivers/
├── Task4a_PID.ioc
├── .project
├── .cproject
└── README.md
```

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/<your-username>/Task4a_PID.git
cd Task4a_PID
```

### 2. Open in STM32CubeIDE

```
File → Import → Existing Projects into Workspace
→ Navigate to the cloned folder → Finish
```

### 3. Flash to Board

1. Connect your STM32 board via USB (ST-Link)
2. Click the green **▶ Run** button in STM32CubeIDE
3. Open a Serial Monitor (115200 baud) to view debug output

### 4. Debug via UART

The bot prints real-time sensor values and state over USART2:

```
SYSTEM STARTED
RAW  : 1200 3800 4500 1100  900 | BW : B B W B B
RAW  :  900  800 4600 4700 4800 | BW : B B W W W
JUNCTION: LEFT TURN
```

---

## 🔧 Tuning Tips

- **Oscillates / Wiggles** → Reduce `Kp`
- **Slow to correct / drifts** → Increase `Kp`
- **Steady-state offset** → Enable `Ki` (start small: `0.001`)
- **Overshoots on curves** → Enable `Kd` (start at `5.0–10.0`)
- **One motor faster** → Adjust `LEFT_BIAS` / `RIGHT_BIAS`
- **Turn angle wrong** → Tune `HAL_Delay()` in `Turn_Left_90` / `Turn_Right_90`

---

## 📋 Prerequisites

- STM32CubeIDE v1.x+
- STM32 HAL Library (included in project)
- ST-Link V2 programmer/debugger
- Serial terminal (e.g., PuTTY, Tera Term) at 115200 baud

---

## 📄 License

This project is part of an embedded systems task submission. Feel free to use and adapt the code for learning purposes.

---

## 🙌 Acknowledgements

- STMicroelectronics HAL Library
- STM32CubeIDE
- Embedded Systems coursework — Task 4A
