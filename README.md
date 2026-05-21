# Ball-on-Beam PID Controller
> Real-time embedded control system built with Arduino Uno, HC-SR04 ultrasonic sensor and a servo motor.

![Status](https://img.shields.io/  badge/status-active-brightgreen)
![Platform](https://img.shields.io/badge/platform-Arduino%20Uno-blue)
![Language](https://img.shields.io/badge/language-C%2B%2B-orange)

---

## Demo
https://youtube.com/shorts/jRve37195GU?feature=share

---

## About
This project implements a **PID controller** that balances a ball at a target position on a beam (tube). An ultrasonic sensor continuously measures the ball's distance and a servo motor adjusts the beam angle to correct any deviation from the setpoint.

Built as a personal project to deepen understanding of **control theory**, **embedded systems** and **mechatronics** — concepts studied at KTH Royal Institute of Technology.

---

## System Overview

```
┌─────────────┐     distance      ┌─────────────┐
│  HC-SR04    │ ───────────────▶  │             │
│  Ultrasonic │                   │  Arduino    │  PWM signal  ┌─────────┐
│  Sensor     │                   │  Uno        │ ───────────▶ │  Servo  │
└─────────────┘                   │  (PID loop) │              └─────────┘
                                  └─────────────┘                   │
                                                                     ▼
                                                              ┌─────────────┐
                                                              │    Beam     │
                                                              │  (tilts)    │
                                                              └─────────────┘
```

**Control loop running at 50 Hz.**

---

## Hardware

| Component | Description |
|---|---|
| Arduino Uno | Microcontroller |
| HC-SR04 | Ultrasonic distance sensor |
| SG90 Servo | Actuator controlling beam angle |
| Cardboard tube | Beam/track for the ball |
| Ping pong ball | Controlled object |

**Wiring:**

| HC-SR04 | Arduino |
|---|---|
| VCC | 5V |
| GND | GND |
| TRIG | Pin 6 |
| ECHO | Pin 7 |

| Servo | Arduino |
|---|---|
| Signal (orange) | Pin 9 |
| VCC (red) | 5V |
| GND (brown) | GND |

---

## Software

**Built with:**
- [PlatformIO](https://platformio.org/) in VS Code
- C++ / Arduino framework

**Dependencies:**
- No external libraries required

---

## PID Implementation

The controller calculates the beam angle correction at every loop iteration:

```cpp
float error      = setpoint - measuredDistance;
integral        += error * dt;
float derivative = (error - prevError) / dt;

float output = Kp * error + Ki * integral + Kd * derivative;
```

**Anti-windup** is applied to the integral term to prevent runaway when the ball is out of range:
```cpp
integral = constrain(integral, -20, 20);
```

### Tuned Parameters

| Parameter | Value | Effect |
|---|---|---|
| Kp | 2.3 | Response speed |
| Ki | 0.0 | Steady-state error correction |
| Kd | 0.6 | Damping / overshoot prevention |
| maxTipp | 25° | Max beam angle |

---

## Getting Started

1. Clone the repository
```bash
git clone https://github.com/ditt-namn/ball-on-beam.git
```

2. Open in VS Code with PlatformIO installed

3. Adjust these parameters in `main.cpp` to match your setup:
```cpp
int   servoMid      = 51;   // Tune until beam is horizontal
float setpoint      = 11.0; // Target position in cm from sensor
float rannans_langd = 18.0; // Beam length in cm
```

4. Upload to Arduino Uno and open Serial Plotter to monitor live data

---

## Results & Tuning

The system is tuned using Serial Plotter in PlatformIO. Key observations:

- **P only** – ball moves toward target but oscillates
- **P + D** – oscillation dampened, smooth approach
- **P + D + I** – eliminates steady-state offset if needed

*Tip: Always tune Kp first, then Kd, and add Ki last.*

---

## Future Improvements

- [ ] Port to **ESP32** with WiFi for live data streaming to browser
- [ ] Replace complementary filter with **Kalman filter**
- [ ] Add web interface for real-time PID tuning
- [ ] Add physical position sensor (SoftPot) along beam for direct ball tracking
- [ ] Design and 3D print a proper mechanical enclosure

---

## Background

This project applies theory from:
- **Control Theory** (Reglerteknik) – PID design and tuning
- **Mechatronics** – integration of mechanics, electronics and software
- **Embedded Systems** – real-time C++ programming

*Studied at KTH Royal Institute of Technology, Mechanical Engineering.*
