# Orthanc - Autonomous Multi-Agent Tracking System

> **⚠️ Active Development**: Core tracking functionality operational. Multi-turret coordination architecture in progress.

A high-performance autonomous tracking system featuring computer vision, behavior tree control architecture, and real-time servo actuation. Built for Raspberry Pi with hardware PWM control and YOLOv8 object detection.

**Project Focus**: Demonstrating embedded autonomy, real-time control systems, and preparation for multi-agent coordination via ROS2.

[![Project Status](https://img.shields.io/badge/status-active%20development-orange)]() [![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%204-red)]() [![C++](https://img.shields.io/badge/C%2B%2B-17-blue)]() [![Python](https://img.shields.io/badge/Python-3.9%2B-blue)]()

---

## 🎯 Project Overview

Orthanc demonstrates core principles of autonomous systems engineering:
- **Behavior Tree Architecture**: Modular decision-making framework enabling reactive autonomy
- **Real-Time Vision Pipeline**: YOLOv8 object detection at 10 Hz with JSON-based inter-process communication
- **Hardware PWM Control**: Jitter-free servo actuation via pigpio daemon (50 Hz control loop)
- **Embedded Linux Deployment**: Headless operation on resource-constrained hardware
- **Scalable Design**: Architecture designed for multi-agent extension via ROS2

### Current Capabilities ✅
- ✅ **Autonomous Person Tracking**: Behavior tree evaluates detections and commands proportional control
- ✅ **Real-Time Vision**: YOLOv8n model running at 10 Hz on Raspberry Pi 4
- ✅ **Smooth Servo Control**: Hardware PWM with configurable motion profiles and safety limits
- ✅ **Scanning Behavior**: Fallback search pattern when target lost
- ✅ **File-Based IPC**: JSON detection sharing between Python vision and C++ control processes
- ✅ **Headless Operation**: Remote deployment with saved output verification

### Target Architecture 🔄
- 🔄 **Multi-Turret Coordination**: Two autonomous agents with distributed decision-making
- 🔄 **ROS2 Integration**: Migrating to publish/subscribe architecture for agent communication
- 🔄 **Predictive Tracking**: Kalman filtering for motion prediction and occlusion handling
- 🔄 **Advanced Behaviors**: Cooperative search patterns, target handoff, coverage optimization

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    VISION PIPELINE (Python)                 │
│  ┌──────────┐      ┌──────────┐      ┌────────────────┐     │
│  │  USB Cam │─────▶│ YOLOv8n  │─────▶│ detections.json│     │
│  └──────────┘      └──────────┘      └────────┬───────┘     │
└─────────────────────────────────────────────────┼───────────┘
                                                  │ IPC
┌─────────────────────────────────────────────────┼───────────┐
│              CONTROL SYSTEM (C++)               │           │
│  ┌─────────────────────────────────────────────▼───────┐    │
│  │           Behavior Tree Tick (20 Hz)                │    │
│  │  ┌────────────────┐                                 │    │
│  │  │   Fallback     │                                 │    │
│  │  │   ┌────────┬─────────────────┐                   │    │
│  │  │   │Sequence│  SimpleScan     │                   │    │
│  │  │   │   ├─HasPersonDetection   │                   │    │
│  │  │   │   └─ProportionalTrack    │                   │    │
│  │  └───┴────────┴─────────────────┘                   │    │
│  └──────────────────────┬──────────────────────────────┘    │
│                         ▼                                   │
│  ┌─────────────────────────────────────────────────────┐    │
│  │              Turret Hardware Layer                  │    │
│  │  ┌──────────────────┐    ┌──────────────────┐       │    │
│  │  │ ServoController  │    │ ServoController  │       │    │
│  │  │   (Pan/GPIO 17)  │    │  (Tilt/GPIO 27)  │       │    │
│  │  └────────┬─────────┘    └─────────┬────────┘       │    │
│  └───────────┼────────────────────────┼────────────────┘    │
│              ▼                        ▼                     │
│       ┌──────────┐              ┌──────────┐                │
│       │ Pan Servo│              │Tilt Servo│                │
│       └──────────┘              └──────────┘                │
└─────────────────────────────────────────────────────────────┘
```

### Key Components

**Behavior Tree Control** (`src/cpp/bt_nodes.hpp`)
- **HasPersonDetection**: Reads JSON detections with file locking, validates timestamps (<500ms staleness)
- **ProportionalTrack**: Implements adaptive proportional control with deadband and limit checking
- **SimpleScan**: Executes systematic search pattern when target lost

**Hardware Abstraction** (`src/cpp/Turret.cpp`, `ServoController.cpp`)
- Single pigpio daemon connection shared across all servos
- RAII resource management prevents GPIO conflicts
- Hardware PWM for jitter-free motion (500-2500μs pulse width range)

**Vision Pipeline** (`src/python/yolo_detector.py`)
- YOLOv8n model optimized for embedded deployment
- Upside-down camera auto-correction
- Confidence filtering and center-point extraction
- Atomic JSON writes with file locking

---

## 🛠️ Bill of Materials

| Component | Specs | Notes |
|-----------|-------|-------|
| **Raspberry Pi 4** | 4GB+ RAM | Main controller, runs vision + control |
| **Pan/Tilt Mount** | [SparkFun ROB-14045](https://www.sparkfun.com/products/14045) | Includes 2x SG90 servos |
| **USB Webcam** | 320x240 @ 15fps | Any UVC-compatible camera |
| **Power Supply** | 5V 3A USB-C | Official Raspberry Pi PSU recommended |
| **MicroSD Card** | 32GB+ Class 10 | For Raspberry Pi OS (Debian 12) |
| **Jumper Wires** | Female-to-Female | GPIO connections |

**Future Multi-Agent Setup**: Second identical turret + ROS2 networking

---

## 🚀 Quick Start

### Prerequisites

```bash
# System dependencies
sudo apt-get update && sudo apt-get install -y \
    cmake libpigpio-dev pigpio python3-pip \
    nlohmann-json3-dev

# Enable pigpiod daemon (required for hardware PWM)
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### Installation

```bash
git clone https://github.com/tatelloyd/orthanc.git
cd orthanc

# Python environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# C++ build
cmake -B build -S .
cmake --build build
```

### Running the System

**Terminal 1 - Vision Pipeline:**
```bash
source venv/bin/activate && cd build
python3 ../src/python/yolo_detector_streaming.py
```

**Terminal 2 - Behavior Tree Controller:**
```bash
./build/bt_tracker
```

The turret will:
1. Execute scanning pattern (no target detected)
2. Lock onto person when detected
3. Track proportionally to keep target centered
4. Resume scanning if target lost

**Stop**: Press `Ctrl+C` in either terminal (servos auto-center on shutdown)

---

## 📊 Technical Implementation

### Behavior Tree Logic

**Decision Flow** (evaluated at 20 Hz):
```
Fallback (executes first successful child)
├─ Sequence: ActiveTracking
│  ├─ HasPersonDetection?  (reads detections.json)
│  └─ ProportionalTrack    (commands servos)
└─ SimpleScan              (fallback search pattern)
```

**Control Algorithm** (`ProportionalTrack`):
```cpp
// Normalized error (camera space: 0.0 to 1.0)
// Camera is upsidedown so invert for servo convention.
double x_error = -(detected_x - 0.5); 
double y_error = -(detected_y - 0.5);

// Adaptive gains (faster response for large errors)
double pan_gain = std::abs(x_error) > 0.1 ? 20.0 : 10.0;

// Compute adjustment with clamping
double pan_adj = std::clamp(x_error * pan_gain, -15.0, 15.0);
double new_pan = std::clamp(current_pan + pan_adj, 10.0, 170.0);

turret->setPanAngle(new_pan);
```

**Deadband & Smoothing**:
- 5% deadband prevents micro-adjustments
- Exponential smoothing (α=0.4) on detection coordinates
- Requires 5 consecutive centered frames before declaring lock

### Inter-Process Communication

**Detection Format** (`detections.json`):
```json
{
  "timestamp": 1733891234.567,
  "detections": [
    {
      "label": "person",
      "confidence": 0.87,
      "x": 0.52,
      "y": 0.48
    }
  ]
}
```

**Synchronization Strategy**:
- Python: Exclusive locks during write (`fcntl.LOCK_EX`)
- C++: Shared locks during read (`flock(fd, LOCK_SH)`)
- Timestamp validation (rejects data >500ms old)
- Graceful failure if file unavailable

### Hardware Control

**PWM Specifications**:
- Frequency: 50 Hz (20ms period)
- Pulse Width: 500μs (0°) to 2500μs (180°)
- Resolution: Hardware PWM via pigpio (DMA-based, CPU-independent)

**Safety Features**:
- Range limiting (10° to 170° mechanical stops)
- Auto-centering on process exit
- Single daemon connection (prevents GPIO conflicts)

---

## 🧪 Current Status & Known Issues

### Working Features ✅
- Autonomous person tracking with behavior tree control
- Real-time YOLOv8 detection at 15 Hz
- Smooth proportional servo control
- Fallback scanning behavior
- Timestamp-based staleness checking

### In-Progress Debugging 🔄

**Tracking Performance**:
- System exhibits periodic tracking instability (investigating control loop timing)
- Occasional servo oscillation near center (tuning deadband/gains)
- Detection latency spikes under Pi CPU load (profiling vision pipeline)

**File I/O**:
- Testing reliability of JSON IPC under rapid updates
- Evaluating migration to shared memory or ROS2 topics for lower latency

**Architecture Refinement**:
- Current 100ms sleep in control loop may be suboptimal (moving to event-driven architecture)
- Investigating predictive tracking (Kalman filter) to smooth motion during detection gaps

### Next Steps
1. Transition to ROS2 topics for vision-control communication
2. Deploy second turret and test coordination primitives

---

## 🗺️ Development Roadmap

### Phase 1: Single-Turret Functionality ✅ (100% Complete)
- [x] Behavior tree framework integration
- [x] Real-time vision pipeline
- [x] Proportional tracking control
- [x] Scanning fallback behavior
- [x] Control loop performance optimization

### Phase 2: Multi-Agent Foundation 🔄 (Next)
- [ ] ROS2 migration (nodes, topics, services)
- [ ] Deploy second turret hardware
- [ ] Deploy AI agents for each tower
- [ ] Distributed state sharing
- [ ] Cooperative search patterns
- [ ] Target handoff protocols

### Phase 3: Advanced Autonomy 📋 (Future)
- [ ] IMU-based stabilization
- [ ] Laser designation integration

---

## 📁 Project Structure

```
orthanc/
├── src/
│   ├── cpp/
│   │   ├── bt_tracker_main.cpp     # Main behavior tree executor
│   │   ├── bt_nodes.hpp            # Condition/action node implementations
│   │   ├── Turret.{cpp,hpp}        # High-level turret API
│   │   ├── ServoController.{cpp,hpp} # PWM control abstraction
│   │   └── trees/
│   │       └── basic_track.xml     # Behavior tree definition
│   └── python/
│       └── yolo_detector_streaming.py        # Vision detection service
├── tests/
│   ├── cpp/
│   │   └── menu.cpp                # Interactive hardware testing
│   └── python/
│       └── test_*.py               # Vision pipeline validation
├── CMakeLists.txt                  # Build configuration
├── requirements.txt                # Python dependencies
└── README.md                       # This file
```

---

## 🔧 Configuration

Key tunable parameters in source files (config file migration planned):

**Tracking Control** (`bt_nodes.hpp`):
```cpp
const double deadband = 0.05;           // Center tolerance (5%)
const double alpha = 0.4;                // Smoothing factor
double pan_gain = large_error ? 20.0 : 10.0;  // Adaptive gains
```

**Vision Detection** (`yolo_detector.py`):
```python
target_fps = 10              # Detection rate
detection_duration = 1000.0  # Run indefinitely (set high)
conf = 0.35                  # YOLO confidence threshold
imgsz = 320                  # Model input size (speed/accuracy tradeoff)
```

**Servo Hardware**:
- Pan: GPIO 17, range 10-170°
- Tilt: GPIO 27, range 10-170°

---

## 🤝 Contributing

This project demonstrates embedded autonomy principles for robotics and defense applications. It's an active portfolio piece under development.

**Feedback Welcome**:
- Architecture suggestions (especially ROS2 migration patterns)
- Control algorithm improvements
- Multi-agent coordination strategies

**Contact**: Open an issue or reach out via [LinkedIn](https://www.linkedin.com/in/tatelloyd/)

---

## 📚 Technical References

**Frameworks & Libraries**:
- [BehaviorTree.CPP](https://www.behaviortree.dev/) - Reactive control architecture
- [pigpio](https://abyz.me.uk/rpi/pigpio/) - Hardware PWM library
- [Ultralytics YOLOv8](https://docs.ultralytics.com/) - Object detection

**Relevant Papers**:
- Colledanchise & Ögren (2018) - *Behavior Trees in Robotics and AI*
- Thrun et al. (2005) - *Probabilistic Robotics* (for planned Kalman integration)

---

## 📄 License

MIT License - See LICENSE file for details

---

## 👤 Author

**Tate Lloyd**  
Robotics & Embedded Systems Engineer

- GitHub: [@tatelloyd](https://github.com/tatelloyd)
- LinkedIn: [/in/tatelloyd](https://www.linkedin.com/in/tatelloyd/)
- Email: tate.lloyd@yale.edu

*Built to demonstrate real-time autonomy, embedded control systems, and robotics software engineering for defense-tech applications.*

---

**Status**: Active Development 🚀 | Portfolio Project  
**Last Updated**: December 2025