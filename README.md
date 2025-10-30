# Orthanc - Autonomous Turret Tracking System

## Overview
Orthanc is an autonomous target tracking system in development, featuring pan/tilt servo control with plans for AI-powered object detection and laser designation. Currently implementing the foundational servo control architecture in C++.

**Current Features:**
- ✅ Modular ServoController class for precise servo management
- ✅ Signal generator for testing and motion profiling
- ✅ Clean C++ architecture with proper resource management
- ✅ pigpiod daemon integration for reliable GPIO control

**Planned Features:**
- 🔄 YOLOv8 object detection integration
- 🔄 Kalman filter for predictive tracking
- 🔄 Laser pointer for target designation
- 🔄 Multi-turret coordination (ROS2)

## Bill of Materials

| Component | Link | Notes |
|-----------|------|-------|
| Raspberry Pi 4 (4GB+ recommended) | [Vilros](https://vilros.com/products/raspberry-pi-4-model-b-1) | Main controller |
| USB-C Power Supply (5.1V, 3A) | [Newark](https://www.newark.com/raspberry-pi/sc0218/rpi-power-supply-usb-c-5-1v-3a/dp/03AH7034) | Official RPi charger |
| MicroSD Card (32GB+) | [SanDisk Ultra](https://shop.sandisk.com/products/memory-cards/microsd-cards/sandisk-ultra-uhs-i-microsd) | For Raspberry Pi OS |
| Pan/Tilt Camera Mount | [SparkFun](https://www.sparkfun.com/pan-tilt-bracket-kit-single-attachment.html) | Includes 2x SG90 servos |
| Jumper Wires | Female-to-female | For servo connections |
| GPIO Breakout Board (optional) | [Treedix](https://www.treedixofficial.com/products/treedix-rpi-gpio-terminal-block-breakout-board-module-expansion-board-compatible-with-raspberry-pi-4b-3b-3b-2b-zero-zero-w) | Easier wiring |

**Future additions:**
- USB Webcam (for YOLOv8 integration)
- Laser pointer module (for target designation)

## Hardware Setup

### Wiring Diagram
```
Raspberry Pi GPIO Connections:
- Pan Servo  → GPIO 17 (Physical Pin 11)
- Tilt Servo → GPIO 27 (Physical Pin 13)
- Ground     → GND (Physical Pin 6, 9, 14, etc.)
- 5V Power   → 5V (Physical Pin 2 or 4)
```

### Assembly
1. Assemble the pan/tilt bracket following [this video tutorial](https://www.youtube.com/watch?v=HUTcWrGf2Hk)
2. Mount servos to bracket (included in kit)
3. Connect servo signal wires to GPIO pins as shown above
4. Connect servo power (red) to 5V, ground (brown/black) to GND

**⚠️ Servo Power Note:** For 2 servos, the Pi's 5V rail is usually sufficient for testing. For production or adding more servos, use an external 5V power supply with common ground.

## Software Setup

### Prerequisites
```bash
# Update system
sudo apt-get update
sudo apt-get upgrade

# Install pigpio daemon (required for servo control)
sudo apt-get install pigpio python3-pigpio

# Enable and start pigpiod
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Verify daemon is running
sudo systemctl status pigpiod
```

### Installation
```bash
# Clone repository
git clone https://github.com/yourusername/orthanc.git
cd orthanc

# Compile the project
g++ -o turret main.cpp Turret.cpp ServoController.cpp SignalGenerator.cpp \
    -lpigpiod_if2 -std=c++17

# Run test program
./turret
```

## Project Structure
```
orthanc/
├── main.cpp              # Main control loop with test patterns
├── Tower.cpp          # High-level turret control (in development)
├── ServoController.h/cpp # Low-level servo interface ✅
├── SignalGenerator.h/cpp # Test pattern generation ✅
└── README.md
```

## Current Status

### Completed ✅
- **ServoController Class**: Robust servo control with:
  - Angle-based control (-90° to +90°)
  - Direct PWM control (500-2500μs)
  - Automatic pulse width clamping for safety
  - Resource management (RAII pattern)
  - Shared pigpiod daemon handle
  
- **SignalGenerator Class**: Time-based signal generation for testing:
  - Square wave patterns
  - Sine wave patterns
  - Triangle wave patterns
  - No pre-computation (evaluates at any time t)

### In Progress 🔄
- **Turret Class**: High-level API wrapping pan/tilt/laser control
- **Motion Profiling**: Smooth trajectory generation

### Planned 📋
- **Computer Vision**: YOLOv8 object detection pipeline
- **Tracking Algorithm**: Kalman filter for velocity estimation and prediction
- **Laser Control**: Target designation system
- **Multi-Agent**: ROS2 integration for turret swarm coordination

## Usage

### Current Test Mode
```bash
./turret
# Executes signal generator test patterns on pan/tilt servos
# Press Ctrl+C to stop and center servos
```

The test program demonstrates:
- Smooth servo motion using signal generator
- Proper initialization and cleanup
- Safe shutdown on interrupt (Ctrl+C)

## Development Roadmap

**November 2024:**
- [x] ServoController architecture
- [x] Signal generator implementation
- [ ] Complete Turret class wrapper
- [ ] Integrate computer vision (webcam + YOLOv8)
- [ ] Implement basic tracking

**December 2024:**
- [ ] Add Kalman filter for predictive tracking
- [ ] Laser pointer integration
- [ ] Deploy second turret for multi-agent testing

**2025:**
- [ ] ROS2 integration for turret coordination
- [ ] SLAM implementation for spatial awareness
- [ ] Web dashboard for monitoring

## Troubleshooting

**Servos not responding:**
```bash
# Check if pigpiod is running
pgrep pigpiod

# Restart daemon if needed
sudo systemctl restart pigpiod
```

**"Failed to connect to pigpio daemon":**
```bash
# Remove stale PID file
sudo rm /var/run/pigpio.pid
sudo pigpiod
```

**Jittery servo motion:**
- Check power supply (weak 5V rail can cause jitter)
- Verify GPIO connections are secure
- Ensure pigpiod is running with sufficient privileges

## Architecture Notes

### Design Decisions
- **Single daemon connection**: Turret class owns the pigpiod handle, shared across all ServoControllers
- **RAII resource management**: Automatic cleanup prevents resource leaks
- **Copy prevention**: ServoControllers cannot be copied (prevents dual control conflicts)
- **Time-based signals**: SignalGenerator evaluates at any time without pre-computing arrays

### Why C++?
- Real-time performance requirements
- Low-level hardware control
- Preparation for ROS2 integration (C++ native)
- Embedded systems best practices

## Contributing
This is an active development project. Contributions welcome after core tracking system is complete.

## License
MIT License - See LICENSE file for details

## Acknowledgments
- Inspired by defense-tech applications and Tolkien's Orthanc tower
- Built with pigpio library for reliable GPIO control
- Signal generation architecture influenced by control systems theory
