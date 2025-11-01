# Orthanc - Autonomous Turret Tracking System

## Overview
Orthanc is an autonomous target tracking system in development, featuring pan/tilt servo control with AI-powered object detection and laser designation capabilities. The system combines real-time C++ servo control with Python-based YOLOv8 computer vision.

**Current Features:**
- ✅ Modular ServoController class for precise servo management
- ✅ Signal generator for testing and motion profiling
- ✅ Clean C++ architecture with proper resource management
- ✅ pigpiod daemon integration for reliable GPIO control
- ✅ YOLOv8 object detection pipeline (testing phase)
- ✅ Headless operation support for SSH development

**Planned Features:**
- 🔄 Real-time target tracking integration (C++ ↔ Python IPC)
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
| OV5647 Camera Module | [Seeed Studio](https://www.digikey.com/en/products/detail/seeed-technology-co-ltd/402990004/5488098) | For YOLOv8 detection |
| Jumper Wires | Female-to-female | For servo connections |
| GPIO Breakout Board (optional) | [Treedix](https://www.treedixofficial.com/products/treedix-rpi-gpio-terminal-block-breakout-board-module-expansion-board-compatible-with-raspberry-pi-4b-3b-3b-2b-zero-zero-w) | Easier wiring |

**Future additions:**
- Laser pointer module (for target designation)
- External 5V power supply (for multi-servo scaling)

## Hardware Setup

### Wiring Diagram
```
Raspberry Pi GPIO Connections:
- Pan Servo  → GPIO 17 (Physical Pin 11)
- Tilt Servo → GPIO 27 (Physical Pin 13)
- Ground     → GND (Physical Pin 6, 9, 14, etc.)
- 5V Power   → 5V (Physical Pin 2 or 4)
- Camera     → USB Port
```

### Assembly
1. Assemble the pan/tilt bracket following [this video tutorial](https://www.youtube.com/watch?v=HUTcWrGf2Hk)
2. Mount servos to bracket (included in kit)
3. Connect servo signal wires to GPIO pins as shown above
4. Connect servo power (red) to 5V, ground (brown/black) to GND
5. Connect camera module to Raspberry Pi CSI port

**⚠️ Servo Power Note:** For 2 servos, the Pi's 5V rail is usually sufficient for testing. For production or adding more servos, use an external 5V power supply with common ground.

## Software Setup

### Prerequisites
```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install pigpio daemon (required for servo control)
sudo apt-get install pigpio python3-pigpio -y

# Enable camera interface
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable
# Reboot after enabling
```

### Enable pigpiod Service (Recommended)
Create systemd service for automatic startup:

```bash
sudo nano /etc/systemd/system/pigpiod.service
```

Paste the following configuration:
```ini
[Unit]
Description=pigpio daemon

[Service]
ExecStart=/usr/bin/pigpiod
ExecStop=/usr/bin/killall pigpiod
Type=forking

[Install]
WantedBy=multi-user.target
```

Enable and start the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

Verify the daemon is running:
```bash
sudo systemctl status pigpiod
```

### Project Installation
```bash
# Clone repository
git clone https://github.com/tatelloyd/orthanc.git
cd orthanc

# Set up Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt

# Compile C++ code
cd src
g++ -o orthanc main.cpp Turret.cpp ServoController.cpp SignalGenerator.cpp \
    -lpigpiod_if2 -std=c++20
```

### Testing

**Test 1: Servo Control**
```bash
cd src
./orthanc
# Expected: Servos execute test motion patterns
# Press Ctrl+C to stop and center servos
```

**Test 2: Camera (Headless)**
```bash
source venv/bin/activate
cd tests
python test_camera_headless.py
# Expected: Creates camera_test_output/ with test images and video
```

**Test 3: YOLOv8 Detection (Headless)**
```bash
python test_yolo_headless.py
# Expected: Creates yolo_test_output/ with annotated detection images
# First run will download YOLOv8n model (~6MB)
```

**Note:** Headless tests save images instead of displaying (works over SSH). To view results:
```bash
# Copy test output to your local machine
scp -r pi@<raspberry-pi-ip>:~/orthanc/camera_test_output ./
scp -r pi@<raspberry-pi-ip>:~/orthanc/yolo_test_output ./
```

## Project Architecture

```
orthanc/
├── .gitignore               # Git exclusions
├── README.md                # This file
├── requirements.txt         # Python dependencies
├── venv/                    # Python virtual environment (not in git)
├── src/
│   ├── main.cpp             # Main control loop with signal interrupt handling
│   ├── Turret.h/.cpp        # High-level turret API (in development)
│   ├── ServoController.h/.cpp  # Low-level servo PWM interface ✅
│   └── SignalGenerator.h/.cpp  # Test pattern generation ✅
├── scripts/
│   └── yolo_detector.py     # YOLOv8 detection → C++ bridge (planned)
├── tests/
│   ├── test_servo.cpp       # Basic PWM motion test ✅
│   ├── test_camera_headless.py     # Camera validation ✅
│   └── test_yolo_headless.py       # YOLO detection validation ✅
├── camera_test_output/      # Camera test results (not in git)
└── yolo_test_output/        # YOLO test results (not in git)
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

- **YOLOv8 Integration**: Computer vision pipeline:
  - Camera validation and frame capture
  - YOLOv8 object detection (tested)
  - Headless operation for remote development

### In Progress 🔄
- **Turret Class**: High-level API wrapping pan/tilt/laser control
- **IPC Bridge**: Python YOLO → C++ servo control communication
- **Smooth Tracking**: Exponential smoothing for natural target following

### Planned 📋
- **Tracking Algorithm**: Kalman filter for velocity estimation and prediction
- **Laser Control**: Target designation system
- **Multi-Agent**: ROS2 integration for turret swarm coordination

## Development Roadmap

**November 2024:**
- [x] ServoController architecture
- [x] Signal generator implementation
- [x] Complete Turret class wrapper
- [x] Camera integration and testing
- [x] YOLOv8 detection pipeline validation
- [ ] Python → C++ IPC implementation (named pipes)
- [ ] Real-time target tracking

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
sudo systemctl status pigpiod

# Restart daemon if needed
sudo systemctl restart pigpiod
```

**"Failed to connect to pigpio daemon":**
```bash
# Remove stale PID file
sudo rm /var/run/pigpio.pid
sudo pigpiod
```

**Camera not detected:**
```bash
# Check camera status
vcgencmd get_camera
# Expected: supported=1 detected=1

# Enable camera if needed
sudo raspi-config
# Interface Options → Camera → Enable → Reboot
```

**Qt/Display errors when running tests over SSH:**
- Use the headless test scripts (`test_camera_headless.py`, `test_yolo_headless.py`)
- These scripts save output images instead of displaying them

**YOLOv8 model download fails:**
```bash
# Manually download model
cd ~
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Model auto-downloads to ~/.cache/ultralytics/ on first run
```

## Architecture Notes

### Design Decisions
- **Single daemon connection**: Turret class owns the pigpiod handle, shared across all ServoControllers
- **RAII resource management**: Automatic cleanup prevents resource leaks
- **Copy prevention**: ServoControllers cannot be copied (prevents dual control conflicts)
- **Time-based signals**: SignalGenerator evaluates at any time without pre-computing arrays
- **Headless testing**: All vision tests save output files for remote development
- **Separate processes**: Python (YOLO) and C++ (servos) run independently, communicate via IPC

### Why C++?
- Real-time performance requirements (50Hz servo control loop)
- Low-level hardware control
- Preparation for ROS2 integration (C++ native)
- Embedded systems best practices

### Why Python for Vision?
- YOLOv8 ecosystem and pretrained models
- Rapid prototyping for computer vision
- OpenCV integration
- Decoupled from real-time servo control

## Contributing
This is an active development project. Contributions welcome after core tracking system is complete.

## License
MIT License - See LICENSE file for details

## Acknowledgments
- Inspired by defense-tech applications and Tolkien's Orthanc tower
- Built with pigpio library for reliable GPIO control
- Signal generation architecture influenced by control systems theory
- Object detection powered by Ultralytics YOLOv8
