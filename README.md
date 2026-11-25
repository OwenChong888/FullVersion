# Dual-Mode Autonomous Delivery Robot (ESP32-S3)

An ESP32-S3 powered dual-mode mobile robot developed for the **AIROST 2026/2027 IoT Track**. The robot autonomously tracks a black line using closed-loop PID control, identifies color-coded target destinations using a TCS34725 RGB sensor, switches to Bluetooth Low Energy (BLE) manual control for precision ping-pong ball pickup with a servo gripper, and displays live telemetry on an SSD1306 OLED screen.

## Key Features
- **Autonomous PID Line Following:** 5-channel IR array with weighted error calculation, dynamic corner braking, and ghost-error memory for traversing 1 cm dashed-line gaps.
- **RGB Color Destination Sorting:** TCS34725 digital color sensor (Red → Ball 1, Blue → Ball 2, Green → Ball 3) with automatic destination stopping and handover to manual mode.
- **BLE Teleoperation:** Low-latency control via **Adafruit Bluefruit LE Connect** (Nordic UART Service) using dual-core FreeRTOS thread-safe state flags.
- **Single-Button Servo Gripper:** SG90 claw driven on an isolated hardware PWM timer (50 Hz Timer 2 vs. 20 kHz Timer 0 for motors).
- **Real-Time OLED Telemetry:** Live BLE connection status, operating mode, detected color, and gripper state.
- **Power Integrity:** Parallel decoupling capacitor bank on the 5V bus (`100 µF + 2 × 220 µF`) and `1000 µF` bulk capacitance on the 12V traction rail to prevent BLE/servo brownouts.

## Hardware Pinout (ESP32-S3 DevKitC-1)

| Subsystem | Component / Signal | ESP32-S3 GPIO | Power Rail | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Motor Driver** | L298N `ENA` (Left PWM) | `GPIO 4` | 12V Motor | LEDC Ch 0, Timer 0 (20 kHz) |
| **Motor Driver** | L298N `IN1` / `IN2` | `GPIO 5` / `GPIO 6` | 5V Logic | Left motor direction |
| **Motor Driver** | L298N `IN3` / `IN4` | `GPIO 7` / `GPIO 15` | 5V Logic | Right motor direction |
| **Motor Driver** | L298N `ENB` (Right PWM) | `GPIO 16` | 12V Motor | LEDC Ch 1, Timer 0 (20 kHz) |
| **Line Tracking** | 5-Ch IR Array (`IR1`–`IR5`) | `17, 18, 8, 9, 10` | 5V Bus | Active-low digital inputs |
| **OLED Display** | SSD1306 (`0x3C`) `SDA` / `SCL` | `GPIO 11` / `GPIO 12` | 3.3V Bus | Hardware `Wire` |
| **Color Sensor** | TCS34725 (`0x29`) `SDA` / `SCL` | `GPIO 1` / `GPIO 2` | 3.3V Bus | Hardware `Wire1` |
| **Gripper** | SG90 Micro Servo Signal | `GPIO 13` | 5V (Cap Bank) | LEDC Timer 2 (50 Hz, 0°/90°) |

## Bluefruit LE Connect Controls
Connect to **`ESP32-S3-Robot`** in the app and open **Controller → Control Pad**:
- **Button 1:** Switch to Manual Mode (stops motors)
- **Button 2:** Switch to Autonomous PID Mode
- **Button 3:** Toggle Gripper (Open `0°` ↔ Closed `90°`)
- **D-Pad (Up/Down/Left/Right):** Manual driving

## Build & Upload (PlatformIO)
1. Clone this repository and open the folder in **VS Code** with the **PlatformIO IDE** extension installed.
2. Required libraries (auto-installed via `platformio.ini`):
   - `adafruit/Adafruit GFX Library`
   - `adafruit/Adafruit SSD1306`
   - `adafruit/Adafruit TCS34725`
   - `madhephaestus/ESP32Servo`
3. Connect the ESP32-S3 via USB and click **PlatformIO: Upload**.

## Project Showcase
<!-- Once you add photos to the /images folder, uncomment the lines below -->
<!-- ![Assembled Robot](images/robot_overview.jpg) -->
<!-- ![System Architecture](images/system_block_diagram.png) -->