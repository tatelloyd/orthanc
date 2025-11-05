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

# Check if executable exists in new location
if [ ! -f "./bin/orthanc" ]; then
    echo "❌ Executable 'bin/orthanc' not found!"
    echo ""
    echo "Please build first using CMake:"
    echo "  cmake -B build -S ."
    echo "  cmake --build build"
    echo ""
    exit 1
fi

# Check if config file exists
if [ ! -f "config/turret_config.json" ]; then
    echo "⚠️  Warning: config/turret_config.json not found"
    echo "   Using default configuration"
    echo ""
fi

# Check if Python script exists
if [ ! -f "src/python/yolo_detector.py" ]; then
    echo "⚠️  Warning: src/python/yolo_detector.py not found"
    echo "   Option 3 (YOLO detection) will not work"
    echo ""
fi

# Activate Python virtual environment if it exists
if [ -d "venv" ]; then
    echo "🐍 Activating Python virtual environment..."
    source venv/bin/activate
    echo "✅ Virtual environment activated"
    echo ""
fi

# Run the program from new location
echo "🚀 Starting Orthanc turret control system..."
echo ""
./bin/orthanc "$@"

# Cleanup message
echo ""
echo "✅ Launcher exited cleanly"