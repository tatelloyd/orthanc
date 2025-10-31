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
git clone https://github.com/tatelloyd/orthanc.git
cd orthanc

# Compile the project
g++ -o orthanc main.cpp Turret.cpp ServoController.cpp SignalGenerator.cpp \
    -lpigpiod_if2 -std=c++20

# Run test program
./orthanc
```

## Project Structure
```
orthanc/
├── main.cpp              # Main control loop  
├── Turret.h/cpp          # High-level turret control (in development)
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
./orthanc
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
=======
**Overview**

This robot can move with pan/tilt functionality via motion controllers in C++.

**Bill of Materials**

-RaspberryPi 4: https://vilros.com/products/raspberry-pi-4-model-b-1?variant=40809478750302&country=US&currency=USD&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&tw_source=google&tw_adid=&tw_campaign=19684058556&gad_source=4&gad_campaignid=19684058613&gbraid=0AAAAAD1QJAjEBBM33-wmkODVKD7MW20ge&gclid=Cj0KCQjw5onGBhDeARIsAFK6QJYBMe_ULjNbXKXG4-nAZ0vRzUujs97eeRkRN07h2lzJH7UucQjYrW0aAqtJEALw_wcB

-RaspberryPi 4 charger: https://www.newark.com/raspberry-pi/sc0218/rpi-power-supply-usb-c-5-1v-3a/dp/03AH7034?CMP=KNC-GUSA-PMAX-SHOPPING-ONBOARD-COMP-NEW&mckv=_dc|pcrid||plid||kword||match||slid||product|03AH7034|pgrid||ptaid||&gad_source=1&gad_campaignid=22957739450&gbraid=0AAAAAD5U_g3noAyNJXRbrbLDB7WvxXL9c&gclid=Cj0KCQjw5onGBhDeARIsAFK6QJZuPxSgGmA-fchVQFMMbaFAxiFpXCWWFYP2dVo4Yf00-OqNIMlm94UaAhiNEALw_wcB

-MicroSD card: https://shop.sandisk.com/products/memory-cards/microsd-cards/sandisk-ultra-uhs-i-microsd?sku=SDSQUAC-256G-GN6MA&ef_id=Cj0KCQjw5onGBhDeARIsAFK6QJZq9oXigKwOKMaYQoy68BFhKIx-FvX3doK0AE07Pt_gKbEKBOpIPTEaAn0YEALw_wcB:G:s&s_kwcid=AL!15012!3!!!!x!!!21840826498!&utm_medium=pdsh2&utm_source=gads&utm_campaign=Google-B2C-Conversion-Pmax-NA-US-EN-Memory_Card-All-All-Brand&utm_content=&utm_term=SDSQUAC-256G-GN6MA&cp2=&gad_source=4&gad_campaignid=21836907008&gbraid=0AAAAA-HVYqnR4xOjgBaxD24l-IEJuHxfs&gclid=Cj0KCQjw5onGBhDeARIsAFK6QJZq9oXigKwOKMaYQoy68BFhKIx-FvX3doK0AE07Pt_gKbEKBOpIPTEaAn0YEALw_wcB

-Mini pan/tilt camera platform anti-vibration camera mount: https://www.sparkfun.com/pan-tilt-bracket-kit-single-attachment.html?gad_source=1&gad_campaignid=21251727806&gbraid=0AAAAADsj4ESYE2Ukpu_jOgP2MIjbUoXqX&gclid=Cj0KCQjwmYzIBhC6ARIsAHA3IkQzGkBx5MLOkwfqv3i62o_xMlFFdkE2P1p6vAhUKDXnprwdBpdwXNcaAm2bEALw_wcB

-Logitech Webcamera: https://www.digikey.com/en/products/detail/seeed-technology-co-ltd/402990004/5488098

-(Optional) RasberryPi GPIO expansion board: https://www.treedixofficial.com/products/treedix-rpi-gpio-terminal-block-breakout-board-module-expansion-board-compatible-with-raspberry-pi-4b-3b-3b-2b-zero-zero-w?variant=42584983240958&country=US&currency=USD&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&srsltid=AfmBOoo-a8o3SQ5zQ-2aiB8bfsIo50FG7bkaSU9nsPlDt4HaPSbPV9VgRE8

-Jumper cables (as needed)


**Getting Started**
Plg the pan and tilt servos into pins 11 (GPIO 17) 13(GPIO 27) respectively.

The stand itself can be put together via this helper video: https://www.youtube.com/watch?v=HUTcWrGf2Hk



>>>>>>> 3e64848276017836244cf8cc2de0b73bc0eab069
