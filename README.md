# Orthanc - Autonomous Turret Tracking System

A high-performance object tracking system using servo-controlled pan/tilt mechanics, computer vision, and real-time control. Built for Raspberry Pi with pigpio hardware PWM and YOLOv8 detection.

![Project Status](https://img.shields.io/badge/status-active-success.svg)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi-red.svg)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)

## 🎯 Features

### Current Features ✅
- **Precision Servo Control**: Hardware PWM via pigpio daemon for smooth, jitter-free motion
- **Real-Time Object Detection**: YOLOv8 integration for vision-based tracking
- **Interactive Menu System**: User-friendly testing and demonstration interface
- **Configurable System**: JSON-based configuration for easy parameter tuning
- **Comprehensive Test Suite**: Servo, camera, and detection pipeline validation
- **Headless Operation**: Remote deployment friendly with saved output verification
- **Modern C++ Architecture**: CMake build system, clean abstractions, extensible design
- **Simultaneous Control**: Servo movement + object detection running in parallel

### Planned Features 🔄
- Real-time target tracking (servo follows detected objects)
- Kalman filter for predictive motion tracking
- Laser pointer for target designation
- IMU integration for stabilization
- Multi-turret coordination (ROS2 architecture)

## 🏗️ Architecture

```
orthanc/
├── src/
│   ├── cpp/                     # C++ servo control and system logic
│   │   ├── main.cpp             # Interactive menu system
│   │   ├── Turret.cpp/.hpp      # High-level turret API
│   │   ├── ServoController.cpp/.hpp  # Low-level PWM control
│   │   └── SignalGenerator.cpp/.hpp  # Motion pattern generation
│   └── python/
│       ├── yolo_detector.py     # YOLOv8 detection service
│       └── calibrate_fov_headless.py  # FOV calibration tool
├── config/
│   └── turret_config.json       # System configuration
├── tests/
│   ├── cpp/
│   │   └── test_servo.cpp       # Servo unit tests
│   └── python/
│       ├── test_camera_headless.py   # Camera validation
│       └── test_yolo_headless.py     # YOLO detection tests
├── CMakeLists.txt               # Modern C++ build system
├── run_tracking.sh              # System launcher script
└── requirements.txt             # Python dependencies
```

## 🛠️ Bill of Materials

| Component | Link | Notes |
|-----------|------|-------|
| Raspberry Pi 4 (4GB+) | [Vilros](https://vilros.com/collections/raspberry-pi-kits) | Main controller |
| USB-C Power Supply (5.1V, 3A) | [Newark](https://www.newark.com/raspberry-pi/sc0212/power-supply-raspberry-pi-4-usb/dp/07AH9685) | Official RPi charger |
| MicroSD Card (32GB+) | [SanDisk Ultra](https://www.amazon.com/SanDisk-Ultra-microSDXC-Memory-Adapter/dp/B073JYVKNX) | For Raspberry Pi OS |
| Pan/Tilt Camera Mount | [SparkFun](https://www.sparkfun.com/products/14391) | Includes 2x SG90 servos |
| USB Webcam | Any USB webcam | For YOLOv8 detection |
| Jumper Wires | Female-to-female | For servo connections |
| GPIO Breakout Board (optional) | [Treedix](https://www.amazon.com/Treedix-GPIO-Breakout-Compatible-Raspberry/dp/B07WR7QTNY) | Easier wiring |

**Future additions:**
- Laser pointer module (for target designation)
- IMU module (MPU6050 or similar)
- External 5V power supply (for multi-servo scaling)

## 🔌 Hardware Setup

### Wiring Diagram
**Raspberry Pi GPIO Connections (BCM numbering):**
```
- Pan Servo  → GPIO 18 (Physical Pin 12)
- Tilt Servo → GPIO 19 (Physical Pin 35)
- Ground     → GND (Physical Pin 6, 9, 14, etc.)
- 5V Power   → 5V (Physical Pin 2 or 4)
- Camera     → USB Port
```

### Assembly
1. Assemble the pan/tilt bracket following [this video tutorial](https://www.youtube.com/watch?v=1jFRUm_VJ9I)
2. Mount servos to bracket (included in kit)
3. Connect servo signal wires to GPIO pins as shown above
4. Connect servo power (red) to 5V, ground (brown/black) to GND
5. Mount USB camera to bracket (upside-down mounting is OK - auto-corrected in software)

⚠️ **Power Note:** For 2 servos, the Pi's 5V rail is usually sufficient for testing. For production or adding more servos, use an external 5V power supply with common ground.

## 🚀 Software Setup

### Prerequisites

**System packages:**
```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install build tools and pigpio
sudo apt-get install cmake libpigpio-dev pigpio python3-pip -y
```

### Enable pigpiod Service (Required)

Create systemd service for automatic startup:

```bash
sudo nano /etc/systemd/system/pigpiod.service
```

Paste this configuration:
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

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Verify it's running
sudo systemctl status pigpiod
```

### Project Installation

```bash
# Clone repository
git clone https://github.com/tatelloyd/orthanc.git
cd orthanc

# Setup Python environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Build C++ components
cmake -B build -S .
cmake --build build

# Make launcher executable
chmod +x run_tracking.sh
```

## 🎮 Usage

### Main Control Interface

```bash
./run_tracking.sh
```

The launcher script will:
- Check if pigpiod is running (starts it if needed)
- Verify the executable exists
- Activate Python virtual environment
- Launch the interactive menu

### Interactive Menu Options

```
╔════════════════════════════════════════════════════════╗
║          ORTHANC TURRET CONTROL MENU                   ║
╠════════════════════════════════════════════════════════╣
║  1. PWM Test (basic servo movement)                    ║
║  2. Signal Generator Test (sine/triangle patterns)     ║
║  3. YOLO Detection (10-second capture)                 ║
║  4. Camera FOV Measurement                             ║
║  5. Manual Control (keyboard angle input)              ║
║  6. Center Position (return to 0°)                     ║
║  7. YOLO + Servo Movement (simultaneous demo)          ║
║  0. Exit                                               ║
╚════════════════════════════════════════════════════════╝
```

**Recommended Testing Order:**
1. **Option 6**: Center turret to verify servo response
2. **Option 1**: Test basic PWM control
3. **Option 2**: Test motion patterns
4. **Option 3**: Test YOLO detection (point camera at objects)
5. **Option 7**: See simultaneous detection + movement! 🎯

### FOV Calibration (Headless)

```bash
# Method 1: Checkerboard pattern (most accurate)
python src/python/calibrate_fov_headless.py \
    --method checkerboard \
    --square-size 2.5 \
    --distance 50 \
    --pattern-width 7 \
    --pattern-height 7 \
    --update-config

# Method 2: Known object dimensions
python src/python/calibrate_fov_headless.py \
    --method manual \
    --object-width 30 \
    --distance 50 \
    --update-config

# Method 3: Camera specifications (Pi Camera V2 example)
python src/python/calibrate_fov_headless.py \
    --method calculate \
    --sensor-width 3.68 \
    --focal-length 3.04 \
    --update-config

# Verify current FOV settings
python src/python/calibrate_fov_headless.py --method verify
```

Results saved to `fov_calibration_output/` for remote verification via SSH.

## ⚙️ Configuration

Edit `config/turret_config.json` to customize system behavior:

```json
{
  "camera": {
    "fov_horizontal": 62.2,
    "fov_vertical": 48.8,
    "resolution": {
      "width": 640,
      "height": 480
    }
  },
  "servos": {
    "pan": {
      "pin": 18,
      "range": [-90, 90],
      "trim": 0.0,
      "speed_limit": 180.0
    },
    "tilt": {
      "pin": 19,
      "range": [-30, 30],
      "trim": 0.0,
      "speed_limit": 120.0
    }
  },
  "tracking": {
    "smoothing_factor": 0.3,
    "update_rate_hz": 20
  }
}
```

**Key Parameters:**
- `pin`: GPIO pin number (BCM numbering)
- `range`: Min/max angles in degrees (relative to center)
- `trim`: Calibration offset for centering (degrees)
- `speed_limit`: Maximum angular velocity (deg/s)
- `smoothing_factor`: Motion smoothing (0=none, 1=max)
- `fov_horizontal/vertical`: Camera field of view (calibrate with headless tool)

## 🧪 Testing

### Run All Tests

```bash
# C++ tests
cd build && ctest

# Individual test executables
./bin/test_servo

# Python tests (headless - saves images instead of displaying)
source venv/bin/activate
python tests/python/test_camera_headless.py
python tests/python/test_yolo_headless.py
```

### Test Outputs

Test results are saved to:
- `camera_test_output/` - Camera capture verification images
- `yolo_test_output/` - Detection result images with bounding boxes
- `fov_calibration_output/` - FOV calibration verification images

**View results over SSH:**
```bash
# Copy test output to your local machine
scp -r pi@<raspberry-pi-ip>:~/orthanc/camera_test_output ./
scp -r pi@<raspberry-pi-ip>:~/orthanc/yolo_test_output ./
scp -r pi@<raspberry-pi-ip>:~/orthanc/fov_calibration_output ./
```

## 🔧 Development

### Building

```bash
# Configure (first time only)
cmake -B build -S .

# Build
cmake --build build

# Clean rebuild
rm -rf build bin
cmake -B build -S .
cmake --build build
```

### Adding New Features

1. **New C++ Classes**: Add to `src/cpp/`, update `CMakeLists.txt`
2. **New Python Modules**: Add to `src/python/`, update `requirements.txt`
3. **New Tests**: Add to `tests/{cpp,python}/`
4. **New Config Parameters**: Update `config/turret_config.json` schema

### Code Style

- **C++**: C++17 standard, RAII patterns, Google-style naming
- **Python**: PEP 8, type hints preferred
- **CMake**: Modern CMake (3.15+) idioms
- **Commits**: Conventional commits format (feat:, fix:, docs:, etc.)

## 📊 Technical Details

### Servo Control
- **PWM Frequency**: 50 Hz (20ms period)
- **Pulse Width Range**: 500-2500 µs (configurable per servo)
- **Control Method**: Hardware PWM via pigpio daemon (jitter-free)
- **Update Rate**: Configurable (default 20 Hz)
- **Safety Features**: Pulse width clamping, range limiting, automatic centering on exit

### Vision Pipeline
- **Detection Model**: YOLOv8n (nano - optimized for Pi)
- **Input Resolution**: 640x480 (configurable)
- **Processing**: Runs in parallel with servo control via system() backgrounding
- **Output**: Bounding boxes with confidence scores
- **Camera Orientation**: Auto-corrects for upside-down mounting

### System Requirements
- **CPU**: Raspberry Pi 3B+ minimum (4B recommended for YOLO)
- **Memory**: 2GB RAM minimum (4GB recommended)
- **Storage**: 2GB for system + models
- **Power**: 5V 3A for Pi + separate 5V supply for servos (production)
- **OS**: Raspberry Pi OS (Bullseye or newer)

### Design Decisions

**Why C++ for Control?**
- Real-time performance requirements (50Hz servo control loop)
- Low-level hardware control via pigpio
- Preparation for ROS2 integration (C++ native)
- Embedded systems best practices

**Why Python for Vision?**
- YOLOv8 ecosystem and pretrained models
- Rapid prototyping for computer vision
- OpenCV integration
- Can run independently from real-time servo control

**Architecture Principles:**
- **Single daemon connection**: Turret class owns the pigpiod handle, shared across all ServoControllers
- **RAII resource management**: Automatic cleanup prevents resource leaks
- **Copy prevention**: ServoControllers cannot be copied (prevents dual control conflicts)
- **Time-based signals**: SignalGenerator evaluates at any time without pre-computing arrays
- **Headless testing**: All vision tests save output files for remote development
- **Config-driven**: No hardcoded values, all parameters externalized

## 🗺️ Development Roadmap

### October/November 2025 ✅
- ✅ Modern CMake build system
- ✅ ServoController architecture with RAII
- ✅ Signal generator for motion patterns
- ✅ Turret class wrapper
- ✅ Camera integration and testing
- ✅ YOLOv8 detection pipeline validation
- ✅ Interactive menu system
- ✅ Simultaneous servo + detection demo
- ✅ Headless FOV calibration tool
- ✅ JSON configuration system

### November 2025 🔄
- 🔄 Integrate AI Agent for real-time target tracking (IPC between Python detection and C++ servos)
- 🔄 Kalman filter for predictive tracking
- 🔄 Laser pointer integration
- 🔄 IMU integration for stabilization

### December 2025 🔄
- 🔄 Deploy second turret for multi-agent testing
- 🔄 ROS2 architecture migration
- 🔄 Multi-turret coordination


## 🐛 Troubleshooting

### Servos not responding
```bash
# Check if pigpiod is running
sudo systemctl status pigpiod

# Restart daemon if needed
sudo systemctl restart pigpiod

# Remove stale PID file if restart fails
sudo rm /var/run/pigpio.pid
sudo pigpiod
```

### Build errors
```bash
# Ensure you're building from project root
cd ~/orthanc

# Clean rebuild
rm -rf build bin
cmake -B build -S .
cmake --build build

# Check for missing dependencies
sudo apt-get install cmake libpigpio-dev
```

### Camera not detected
```bash
# Check USB camera
lsusb  # Should show your webcam

# Test camera access
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera FAIL')"
```

### YOLO detection not working
```bash
# Activate venv first
source venv/bin/activate

# Check if ultralytics is installed
pip list | grep ultralytics

# Reinstall if needed
pip install ultralytics opencv-python

# Model auto-downloads to ~/.cache/ultralytics/ on first run
# If download fails, manually get it:
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
mv yolov8n.pt ~/.cache/ultralytics/
```

### Python module not found when running bin/orthanc directly
```bash
# Always use run_tracking.sh instead of bin/orthanc directly
./run_tracking.sh

# Or activate venv manually:
source venv/bin/activate
./bin/orthanc
```

### Qt/Display errors over SSH
- Use the headless test scripts (`test_camera_headless.py`, `test_yolo_headless.py`)
- These scripts save output images instead of displaying them
- All calibration tools support headless operation with `--method verify`

## 🤝 Contributing

This is an active development project and personal portfolio piece. Contributions, suggestions, and feedback are welcome after core tracking system is complete!

**Contact:**
- Open an issue for bugs or feature requests
- Star the repo if you find it useful
- Check back for updates as new features are added

## 📄 License

MIT License - See LICENSE file for details

## 🙏 Acknowledgments

- Inspired by defense-tech applications and Tolkien's Orthanc tower
- Built with [pigpio library](http://abyz.me.uk/rpi/pigpio/) for reliable GPIO control
- Object detection powered by [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- Signal generation architecture influenced by control systems theory

---

**Author:** Tate Lloyd  
**GitHub:** [@tatelloyd](https://github.com/tatelloyd)  
**Repository:** https://github.com/tatelloyd/orthanc  
**Status:** Active Development 🚀

---

*Built for defense tech and robotics portfolio demonstrations.*  
*Showcasing real-time control, computer vision, and embedded systems engineering.*