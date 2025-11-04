#!/bin/bash
# Simple launcher for Orthanc turret control system

echo "╔════════════════════════════════════════════════════════╗"
echo "║          ORTHANC TURRET LAUNCHER v1.0                  ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

# Check if pigpiod is running
if ! pgrep -x "pigpiod" > /dev/null; then
    echo "⚠️  pigpiod is not running. Starting it now..."
    sudo pigpiod
    sleep 1
    
    if pgrep -x "pigpiod" > /dev/null; then
        echo "✅ pigpiod started successfully"
    else
        echo "❌ Failed to start pigpiod"
        echo "   Try running: sudo pigpiod"
        exit 1
    fi
else
    echo "✅ pigpiod is already running"
fi

echo ""

# Check if executable exists
if [ ! -f "./orthanc" ]; then
    echo "❌ Executable 'orthanc' not found!"
    echo ""
    echo "Please compile first:"
    echo "  g++ -o orthanc src/cpp/main.cpp src/cpp/Turret.cpp src/cpp/ServoController.cpp src/cpp/SignalGenerator.cpp -lpigpiod_if2 -std=c++20 -O2"
    echo ""
    exit 1
fi

# Check if Python script exists
if [ ! -f "src/python/yolo_detector.py" ]; then
    echo "⚠️  Warning: src/python/yolo_detector.py not found"
    echo "   Option 3 (YOLO detection) will not work"
    echo ""
fi

# Run the program
echo "🚀 Starting Orthanc turret control system..."
echo ""
./orthanc

# Cleanup message
echo ""
echo "✅ Launcher exited cleanly"