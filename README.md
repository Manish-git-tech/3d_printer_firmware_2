# Custom 3D Printer Firmware for Raspberry Pi Pico with Quadrature Encoders and Advanced Motion Control

This 3D Printer Firmware implements precise multi-axis motion control for my custom built 3D printer using Raspberry Pi Pico/RP2040, featuring quadrature encoder feedback, Bresenham-based path planning, cascaded PID control, and robust configuration storage using LittleFS. Designed for PlatformIO with modular architecture and real-time performance.

---

## Table of Contents
1. [Installation Guide](#installation-guide)
2. [Building the Firmware](#building-the-firmware)
3. [Flashing to Raspberry Pi Pico](#flashing-to-raspberry-pi-pico)
4. [Hardware Configuration](#hardware-configuration)
5. [Core System Architecture](#core-system-architecture)
6. [Advanced Configuration](#advanced-configuration)
7. [Troubleshooting](#troubleshooting)
8. [Customization for Other Boards](#customization-for-other-boards)

---

## Installation Guide 

### Prerequisites
- **PlatformIO Core** (VSCode extension or standalone)
- **Python 3.7+**
- **Git** for version control

### Repository Setup
```bash
git clone https://github.com/Manish-git-tech/3d_printer_firmware_2.git
cd 3d_printer_firmware_2
```

### PlatformIO Configuration
1. Install dependencies:
```bash
pio pkg install
```
2. Configure board in `platformio.ini`:
```ini
[env:pico]
platform = raspberrypi
board = pico
framework = arduino
```

### File Structure
```
src/
├── comms/              # G-code parsing and serial communication
│   ├── comms.cpp       # Command state machine
│   └── comms_parser.h  # G-code syntax definitions
├── motion/             # Motor control and feedback
│   ├── encoder.cpp     # Quadrature decoding
│   └── pid_control.cpp # Cascaded PID implementation
├── path_planning/      # Motion algorithms
│   └── bresenham.cpp   # Coordinated movement
├── config/             # Hardware parameters
│   ├── pins.h          # GPIO mappings
│   └── motion_params.h # Kinematic settings
└── main.cpp            # Firmware entry point
```

---

## Building the Firmware 

### Compilation Commands
```bash
# Build for Raspberry Pi Pico
pio run -e pico

# Build for ESP8266 (WiFi module)
pio run -e esp8266

# Clean build artifacts
pio run -t clean
```

### Build Flags
| Environment Variable | Description |
|----------------------|-------------|
| `ENABLE_DEBUG_LOG` | Verbose serial output |
| `FORCE_EEPROM_RESET` | Clear stored settings |

---

## Flashing to Raspberry Pi Pico 

### Method 1: PlatformIO Upload
```bash
pio run -e pico -t upload
```

### Method 2: Manual UF2 Installation
1. Hold BOOTSEL button while connecting USB
2. Copy `.pio/build/pico/firmware.uf2` to RPI-RP2 drive
3. Wait for automatic reboot


### Required Components
- Raspberry Pi Pico (Main Controller)
- ESP8266 (WiFi Communication)
- MX1508 Motor Drivers (2 per axis)
- DC Motors with Quadrature Encoders
- 24V Power Supply

### Pin Mapping Example (`config/pins.h`)
```cpp
// X-axis Configuration
#define X_ENC_A 2    // Encoder Phase A
#define X_ENC_B 3    // Encoder Phase B  
#define X_MOT_IN1 4  // MX1508 Direction 1
#define X_MOT_IN2 5  // MX1508 Direction 2
```

### Encoder Specifications
| Axis | Slits/mm | CPR | Maximum RPM |
|------|----------|-----|-------------|
| X    | 120      | 480 | 300         |
| Y    | 150      | 600 | 250         |
| Z    | 200      | 800 | 150         |

---

## Core System Architecture 

### 1. Quadrature Encoder Handling
**Implementation:** `motion/encoder.cpp`

- Uses pin change interrupts for real-time decoding
- 4x counting mode with state transition table:
  ```cpp
  const int8_t ENCODER_STATES[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  ```
- Velocity calculation via position differentiation:
  ```cpp
  float getVelocity() {
    return (current_pos - last_pos) / (micros() - last_time) * 1e6;
  }
  ```

### 2. Bresenham Algorithm Implementation
**Implementation:** `path_planning/bresenham.cpp`

- Coordinated multi-axis movement
- Error accumulation for step synchronization:
  ```cpp
  void BresenhamPlanner::planMove(int32_t target[]) {
    steps[X] = abs(target[X] - current[X]);
    steps[Y] = abs(target[Y] - current[Y]);
    
    err = steps[X] - steps[Y];
    
    while(current != target) {
      if(err * 2 >= -steps[Y]) {
        err -= steps[Y];
        current[X] += x_dir;
      }
      if(err * 2 

### Motion Parameters (`config/motion_params.h`)
```cpp
// X-axis Settings
constexpr uint16_t X_STEPS_PER_MM = 80;
constexpr float X_MAX_ACCEL = 1500.0f; // mm/s²
constexpr float X_JERK = 20.0f; // mm/s

// PID Defaults
constexpr float DEFAULT_KP = 10.0f;
constexpr float DEFAULT_KI = 0.05f;
```

### Serial Communication Protocol
| Command | Parameters | Description |
|---------|------------|-------------|
| G0      | X Y Z F    | Linear move |
| G1      | X Y Z E F  | Linear move with extrusion |
| M2000   | Pid values | change and save pid values in EEPROM |
| M2001   | --         | Load pid values |

---

## Troubleshooting 

### Common Issues
| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Position drift | Encoder noise | Add 0.1μF capacitors to encoder inputs |
| Motor stalling | PWM below 47% | Increase minimum power in `motor_control.cpp` |
| EEPROM corruption | Improper shutdown | Implement atomic writes with checksums |

### Diagnostic Commands
```bash
# Report encoder counts
M119

# Dump EEPROM contents
M503

# Test axis movement
G0 X100 F500
```

---

## Customization for Other Boards 

### Adapting for ESP32
1. Modify `platformio.ini`:
   ```ini
   [env:esp32]
   platform = espressif32
   board = esp32dev
   framework = arduino
   ```
2. Update pin mappings in `config/pins.h`

### Supporting New Encoders
1. Implement encoder interface:
   ```cpp
   class CustomEncoder : public EncoderBase {
     public:
       int32_t read() override;
       void init() override;
   };
   ```
2. Register in `motion/motion.cpp`:
   ```cpp
   Encoder* x_encoder = new CustomEncoder(X_ENC_A, X_ENC_B);
   ```

**MIT License** - Free for personal and commercial use

**Special Thanks:**
- Marlin Firmware Team for motion control foundations
- PlatformIO for build system
- Raspberry Pi Foundation for Pico SDK

> **Note:** Always verify electrical connections before powering on the system. Incorrect wiring may damage components.
