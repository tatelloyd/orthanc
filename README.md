# Orthanc - Autonomous Turret Tracking System

## Overview
Orthanc is an autonomous target tracking system featuring pan/tilt servo control with AI-powered object detection. The system combines real-time C++ servo control with Python-based YOLOv8 computer vision for interactive testing and demonstration.

### Current Features:
✅ Interactive menu-driven control system  
✅ Modular ServoController class for precise servo management  
✅ Signal generator for testing and motion profiling  
✅ YOLOv8 object detection with live camera feed  
✅ Simultaneous servo movement + object detection demo  
✅ Headless operation support for SSH development  
✅ Clean C++ architecture with proper resource management  

### Planned Features:
🔄 Real-time target tracking (servo follows detected objects)  
🔄 Kalman filter for predictive tracking  
🔄 Laser pointer for target designation  
🔄 Multi-turret coordination (ROS2)  

---

## Bill of Materials
| Component | Link | Notes |
|-----------|------|-------|
| Raspberry Pi 4 (4GB+ recommended) | [Vilros](https://vilros.com/collections/raspberry-pi-kits) | Main controller |
| USB-C Power Supply (5.1V, 3A) | [Newark](https://www.newark.com/raspberry-pi/sc0212/power-supply-raspberry-pi-4-usb/dp/07AH9685) | Official RPi charger |
| MicroSD Card (32GB+) | [SanDisk Ultra](https://www.amazon.com/SanDisk-Ultra-microSDXC-Memory-Adapter/dp/B073JYVKNX) | For Raspberry Pi OS |
| Pan/Tilt Camera Mount | [SparkFun](https://www.sparkfun.com/products/14391) | Includes 2x SG90 servos |
| USB Webcam | Any USB webcam | For YOLOv8 detection |
| Jumper Wires | Female-to-female | For servo connections |
| GPIO Breakout Board (optional) | [Treedix](https://www.amazon.com/Treedix-GPIO-Breakout-Compatible-Raspberry/dp/B07WR7QTNY) | Easier wiring |

**Future additions:**
- Laser pointer module (for target designation)
- External 5V power supply (for multi-servo scaling)

---

## Hardware Setup

### Wiring Diagram
**Raspberry Pi GPIO Connections:**
```
- Pan Servo  → GPIO 17 (Physical Pin 11)
- Tilt Servo → GPIO 27 (Physical Pin 13)
- Ground     → GND (Physical Pin 6, 9, 14, etc.)
- 5V Power   → 5V (Physical Pin 2 or 4)
- Camera     → USB Port
```

### Assembly
1. Assemble the pan/tilt bracket following [this video tutorial](https://www.youtube.com/watch?v=1jFRUm_VJ9I)
2. Mount servos to bracket (included in kit)
3. Connect servo signal wires to GPIO pins as shown above
4. Connect servo power (red) to 5V, ground (brown/black) to GND
5. Connect USB camera to Raspberry Pi

⚠️ **Servo Power Note:** For 2 servos, the Pi's 5V rail is usually sufficient for testing. For production or adding more servos, use an external 5V power supply with common ground.

---

## Software Setup

### Prerequisites
```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install pigpio daemon (required for servo control)
sudo apt-get install pigpio python3-pigpio -y
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
g++ -o orthanc src/main.cpp version/Turret.cpp version/ServoController.cpp version/SignalGenerator.cpp \
    -lpigpiod_if2 -std=c++20 -O2

# Make launcher executable
chmod +x run_tracking.sh
```

---

## Running the System

### Quick Start
```bash
# Launch the interactive control menu
./run_tracking.sh
```

The launcher script will:
- Check if pigpiod is running (starts it if needed)
- Verify the executable exists
- Launch the interactive menu

### Interactive Menu Options

```
╔════════════════════════════════════════════════════════╗
║          ORTHANC TURRET CONTROL MENU                  ║
╠════════════════════════════════════════════════════════╣
║  1. Pan back and forth for 10 seconds                 ║
║  2. Tilt back and forth for 10 seconds                ║
║  3. Run YOLO detection (static camera)                ║
║  4. Set pan angle (0-180 degrees)                     ║
║  5. Set tilt angle (0-180 degrees)                    ║
║  6. Center turret (90°, 90°)                          ║
║  7. YOLO detection + servo movement (20s demo)        ║
║  0. Exit                                              ║
╚════════════════════════════════════════════════════════╝
```

**Recommended Testing Order:**
1. **Option 6**: Center the turret to verify servos respond
2. **Option 1**: Test pan servo movement
3. **Option 2**: Test tilt servo movement
4. **Option 3**: Test YOLO detection (point camera at objects)
5. **Option 7**: See simultaneous detection + movement! 🎯

---

## Testing Components Individually

### Test 1: Servo Control (from tests/)
```bash
cd tests
g++ -o test_servo test_servo.cpp ../version/ServoController.cpp -lpigpiod_if2 -std=c++20
./test_servo
# Expected: Servos execute test motion patterns
# Press Ctrl+C to stop and center servos
```

### Test 2: Camera (Headless)
```bash
source venv/bin/activate
cd tests
python test_camera_headless.py
# Expected: Creates camera_test_output/ with test images and video
```

### Test 3: YOLOv8 Detection (Headless)
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

---

## Project Architecture

```
orthanc/
├── .gitignore               # Git exclusions
├── README.md                # This file
├── requirements.txt         # Python dependencies
├── run_tracking.sh          # Launcher script (checks pigpiod, runs executable)
├── orthanc                  # Compiled executable (not in git)
├── venv/                    # Python virtual environment (not in git)
├── src/
│   └── main.cpp             # Interactive menu system + control logic
├── version/
│   ├── Turret.h/.cpp        # High-level turret API
│   ├── ServoController.h/.cpp  # Low-level servo PWM interface ✅
│   └── SignalGenerator.h/.cpp  # Test pattern generation ✅
├── scripts/
│   └── yolo_detector.py     # YOLOv8 detection service (10s runtime)
├── tests/
│   ├── test_servo.cpp       # Basic PWM motion test ✅
│   ├── test_camera_headless.py     # Camera validation ✅
│   └── test_yolo_headless.py       # YOLO detection validation ✅
├── camera_test_output/      # Camera test results (not in git)
└── yolo_test_output/        # YOLO test results (not in git)
```

---

## Current Status

### Completed ✅
- **Interactive Control System**: Menu-driven interface for testing all components
- **ServoController Class**: Robust servo control with:
  - Angle-based control (0-180°)
  - Direct PWM control (600-2400μs)
  - Automatic pulse width clamping for safety
  - Resource management (RAII pattern)
- **SignalGenerator Class**: Time-based signal generation for testing:
  - Square wave patterns
  - Sine wave patterns
  - Triangle wave patterns
- **YOLOv8 Integration**: Computer vision pipeline:
  - Camera validation and frame capture
  - YOLOv8 object detection (tested)
  - Upside-down camera support (image flipping)
  - Headless operation for remote development
- **Simultaneous Demo**: Servos move while YOLO detects objects (Option 7)

### In Progress 🔄
- **Turret Class**: High-level API wrapping pan/tilt control
- **Real-time Tracking**: Servos follow detected objects automatically
- **Smooth Tracking**: PID control for natural target following

### Planned 📋
- **Tracking Algorithm**: Kalman filter for velocity estimation and prediction
- **Laser Control**: Target designation system
- **Multi-Agent**: ROS2 integration for turret swarm coordination

---

## Development Roadmap

**November 2024:**
- ✅ ServoController architecture
- ✅ Signal generator implementation
- ✅ Turret class wrapper
- ✅ Camera integration and testing
- ✅ YOLOv8 detection pipeline validation
- ✅ Interactive menu system
- ✅ Simultaneous servo + detection demo
- 🔄 Real-time target tracking (Python → C++ IPC)

**December 2024:**
- 🔄 Add Kalman filter for predictive tracking
- 🔄 Laser pointer integration
- 🔄 Deploy second turret for multi-agent testing

**2025:**
- 🔄 ROS2 integration for turret coordination
- 🔄 SLAM implementation for spatial awareness
- 🔄 Web dashboard for monitoring

---

## Troubleshooting

### Servos not responding:
```bash
# Check if pigpiod is running
sudo systemctl status pigpiod

# Restart daemon if needed
sudo systemctl restart pigpiod
```

### "Failed to connect to pigpio daemon":
```bash
# Remove stale PID file
sudo rm /var/run/pigpio.pid
sudo pigpiod
```

### Camera not detected:
```bash
# Check camera status (for USB webcams, check lsusb)
lsusb
# Should show your webcam in the list

# Test camera with simple capture
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera FAIL')"
```

### Qt/Display errors when running tests over SSH:
- Use the headless test scripts (`test_camera_headless.py`, `test_yolo_headless.py`)
- These scripts save output images instead of displaying them

### YOLOv8 model download fails:
```bash
# Manually download model
cd ~
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Model auto-downloads to ~/.cache/ultralytics/ on first run
```

### Compilation errors:
```bash
# Ensure you're compiling from project root
cd ~/orthanc
g++ -o orthanc src/main.cpp version/Turret.cpp version/ServoController.cpp version/SignalGenerator.cpp \
    -lpigpiod_if2 -std=c++20 -O2
```

---

## Architecture Notes

### Design Decisions
- **Single daemon connection**: Turret class owns the pigpiod handle, shared across all ServoControllers
- **RAII resource management**: Automatic cleanup prevents resource leaks
- **Copy prevention**: ServoControllers cannot be copied (prevents dual control conflicts)
- **Time-based signals**: SignalGenerator evaluates at any time without pre-computing arrays
- **Headless testing**: All vision tests save output files for remote development
- **Template functions**: Reduce code duplication for similar servo operations
- **Menu-driven testing**: Interactive system for validating each component

### Camera Mounting
The camera is mounted **upside-down** for better physical fit. The Python detector automatically flips the image using `cv2.flip(frame, -1)` so YOLO sees objects right-side-up. This is critical for detection accuracy!

### Why C++?
- Real-time performance requirements (50Hz servo control loop)
- Low-level hardware control
- Preparation for ROS2 integration (C++ native)
- Embedded systems best practices

### Why Python for Vision?
- YOLOv8 ecosystem and pretrained models
- Rapid prototyping for computer vision
- OpenCV integration
- Can run independently from real-time servo control

---

## Contributing
This is an active development project. Contributions welcome after core tracking system is complete.

## License
MIT License - See LICENSE file for details

## Acknowledgments
- Inspired by defense-tech applications and Tolkien's Orthanc tower
- Built with [pigpio library](http://abyz.me.uk/rpi/pigpio/) for reliable GPIO control
- Signal generation architecture influenced by control systems theory
- Object detection powered by [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)

---

**Author:** Tate Lloyd  
**Repository:** https://github.com/tatelloyd/orthanc  
**Status:** Active Development 🚀