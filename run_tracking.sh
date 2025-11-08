#!/bin/bash
# Launch AI tracking system

set -e

echo "🚀 Starting ORTHANC AI Tracking System"

# Check API key
if [ -z "$ANTHROPIC_API_KEY" ]; then
    echo "❌ Error: ANTHROPIC_API_KEY not set"
    echo "   Set it with: export ANTHROPIC_API_KEY='your-key-here'"
    exit 1
fi

# Start pigpio daemon
echo "Starting pigpio daemon..."
sudo pigpiod
sleep 1

# Start detection service in background
echo "Starting YOLO detection service..."
venv/bin/python src/python/detection_service.py --fps 10 &
YOLO_PID=$!

sleep 2  # Let detector initialize

# Start tracking system
echo "Starting AI tracking agent..."
./build/orthanc_tracker

# Cleanup on exit
kill $YOLO_PID 2>/dev/null || true
sudo killall pigpiod 2>/dev/null || true

echo "✅ Shutdown complete"