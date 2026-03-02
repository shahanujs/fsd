# RoboRacer — FPV Teleoperated Robot Car

A real-time teleoperated robot car with a **first-person-view (FPV) cockpit GUI**, built with a Raspberry Pi, Python, and pygame. Drive remotely over WiFi with live video, real-time telemetry, and advanced driver-assist features.

## Features

- **FPV Video Streaming** — Live 1280x720 MJPEG from Pi camera with optical flow stabilization
- **Tesla-Style Speedometer** — Real-time speed from motor encoders, D/R gear indicator, analog gauge needle
- **Smooth Throttle Ramping** — Configurable acceleration, deceleration, and braking rates for realistic driving feel
- **Adjustable Speed Limiter** — Drag slider or use +/- keys to cap max throttle (0–100%)
- **Adjustable Steering Angle** — Drag slider to limit max steering (0–80°)
- **Obstacle Avoidance** — Dual HC-SR04 ultrasonic sensors with configurable stop distance and ON/OFF toggle
- **Motor Encoder Telemetry** — Real RPM and cm/s from JGA25-370 quadrature encoders via 5V→3.3V level shifter
- **Resizable GUI** — Proportional layout that scales to any window size, F11 fullscreen toggle
- **One-Command Launch** — `python controller.py` auto-starts all Pi services via SSH and kills them on exit

## Architecture

```
┌─────────────────────┐         UDP:5005          ┌──────────────────────┐
│   Ubuntu Laptop     │ ◄─────────────────────► │   Raspberry Pi       │
│                     │    throttle/steer cmd     │                      │
│  controller.py      │    telemetry response     │  cockpit_receiver.py │
│  (pygame cockpit)   │                           │  (motors/servo/      │
│                     │ ◄──── HTTP:8000 ────────  │   sonar/encoders)    │
│                     │    MJPEG video stream      │                      │
│                     │                           │  video_stream.py     │
│                     │                           │  (Flask + OpenCV)    │
└─────────────────────┘                           └──────────────────────┘
```

## Hardware

| Component | Details |
|---|---|
| **SBC** | Raspberry Pi (any model with GPIO + camera) |
| **Motors** | 2x DC motors via L298N driver |
| **Steering** | Servo on GPIO 12 |
| **Encoders** | 2x JGA25-370 quadrature encoders (PPR=390) via 5V→3.3V level shifter |
| **Sonar** | 2x HC-SR04 ultrasonic sensors (Left + Right) |
| **Camera** | Pi Camera or USB webcam (V4L2) |

### GPIO Pin Map (BCM)

| Function | GPIO Pins |
|---|---|
| Motor A (IN1, IN2, EN) | 17, 27, 18 |
| Motor B (IN1, IN2, EN) | 22, 23, 25 |
| Servo | 12 |
| Sonar L (TRIG, ECHO) | 5, 26 |
| Sonar R (TRIG, ECHO) | 19, 13 |
| Encoder L (Phase A, B) | 16, 24 |
| Encoder R (Phase A, B) | 21, 20 |

## Setup

### Prerequisites
- Python 3.10+
- Pi accessible via SSH (hostname `rpi`, user with passwordless sudo)
- WiFi network connecting laptop and Pi

### Laptop (Ubuntu)
```bash
pip install pygame opencv-python numpy
```

### Raspberry Pi
```bash
pip install flask opencv-python RPi.GPIO
```

### Run
```bash
python controller.py
```
This automatically:
1. Kills any old Pi processes
2. Starts `cockpit_receiver.py` (motors, sonar, encoders) via SSH
3. Starts `video_stream.py` (camera stream) via SSH
4. Opens the cockpit GUI
5. Cleans up all Pi processes on exit (Ctrl+C or window close)

## Controls

| Key | Action |
|---|---|
| W / Up | Throttle forward |
| S / Down | Throttle reverse |
| A / Left | Steer left |
| D / Right | Steer right |
| + / = | Increase speed limit |
| - | Decrease speed limit |
| F11 | Toggle fullscreen |
| ESC | Quit |

Mouse: Drag sliders for MAX SPEED, MAX STEER, STOP DIST. Click toggle for obstacle avoidance ON/OFF.

## Defaults

| Setting | Default |
|---|---|
| Speed Limit | 50% |
| Max Steering | 32° (40% of 80°) |
| Obstacle Avoidance | OFF |
| Stop Distance | 30 cm |

## License

MIT