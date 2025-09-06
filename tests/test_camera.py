#!/usr/bin/env python3
"""
Camera Test Script for Logitech Webcam on Raspberry Pi
Run this first to ensure your camera is working!
"""

import cv2
import numpy as np
import sys
import subprocess
from pathlib import Path

def check_camera_devices():
    """Check available camera devices in /dev/"""
    print("🔍 Checking for video devices...")
    
    try:
        result = subprocess.run(['ls', '/dev/video*'], 
                              capture_output=True, 
                              text=True, 
                              shell=True)
        
        if result.returncode == 0:
            devices = result.stdout.strip().split('\n')
            print(f"Found devices: {devices}")
            return devices
        else:
            print("No video devices found in /dev/")
            return []
    except Exception as e:
        print(f"Error checking devices: {e}")
        return []

def test_camera_index(index):
    """Test a specific camera index"""
    print(f"\n📷 Testing camera index {index}...")
    
    try:
        cap = cv2.VideoCapture(index)
        
        if not cap.isOpened():
            print(f"  ❌ Cannot open camera at index {index}")
            return False
        
        # Try to read a frame
        ret, frame = cap.read()
        
        if not ret:
            print(f"  ❌ Camera opened but cannot read frames")
            cap.release()
            return False
        
        # Get camera properties
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        
        print(f"  ✅ Camera working!")
        print(f"     Resolution: {width}x{height}")
        print(f"     FPS: {fps}")
        print(f"     Frame shape: {frame.shape}")
        
        # Show live preview for 5 seconds
        print(f"  📺 Showing live preview (press 'q' to close)...")
        
        start_time = cv2.getTickCount()
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Add text overlay
            cv2.putText(frame, f"Camera {index}: {width}x{height} @ {fps}fps", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, "Press 'q' to continue", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow(f'Camera Test - Index {index}', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # Auto-close after 5 seconds
            elapsed = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()
            if elapsed > 5:
                break
        
        cv2.destroyAllWindows()
        cap.release()
        return True
        
    except Exception as e:
        print(f"  ❌ Error: {e}")
        return False

def test_v4l2_controls(index):
    """Test V4L2 controls for the camera"""
    print(f"\n🎛️  Testing V4L2 controls for camera {index}...")
    
    try:
        # Check if v4l2-ctl is installed
        result = subprocess.run(['which', 'v4l2-ctl'], 
                              capture_output=True, 
                              text=True)
        
        if result.returncode != 0:
            print("  ℹ️  v4l2-ctl not installed. Install with: sudo apt-get install v4l-utils")
            return
        
        # Get camera controls
        result = subprocess.run([f'v4l2-ctl -d /dev/video{index} --list-ctrls'], 
                              capture_output=True, 
                              text=True,
                              shell=True)
        
        if result.returncode == 0:
            print("  Available controls:")
            print(result.stdout[:500])  # Print first 500 chars
        else:
            print("  Could not get V4L2 controls")
            
    except Exception as e:
        print(f"  Error checking V4L2: {e}")

def diagnose_issues():
    """Diagnose common camera issues"""
    print("\n🔧 Diagnostic Information:")
    
    # Check USB devices
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        if "Logitech" in result.stdout:
            print("  ✅ Logitech device detected on USB")
        else:
            print("  ⚠️  No Logitech device found on USB")
            print("     Run 'lsusb' to see all USB devices")
    except:
        pass
    
    # Check permissions
    try:
        import os
        video_group = subprocess.run(['groups'], capture_output=True, text=True)
        if 'video' in video_group.stdout:
            print("  ✅ User is in 'video' group")
        else:
            print("  ⚠️  User may not be in 'video' group")
            print(f"     Run: sudo usermod -a -G video {os.getlogin()}")
            print("     Then logout and login again")
    except:
        pass
    
    # Check OpenCV build info
    print("\n  OpenCV Build Info:")
    print(f"    Version: {cv2.__version__}")
    
    # Check if V4L2 backend is available
    backends = [cv2.CAP_V4L2, cv2.CAP_V4L, cv2.CAP_ANY]
    backend_names = ['V4L2', 'V4L', 'ANY']
    
    print("  Testing backends:")
    for backend, name in zip(backends, backend_names):
        cap = cv2.VideoCapture(0, backend)
        if cap.isOpened():
            print(f"    ✅ {name} backend works")
            cap.release()
        else:
            print(f"    ❌ {name} backend doesn't work")

def save_test_image(index):
    """Save a test image from the camera"""
    print(f"\n💾 Saving test image from camera {index}...")
    
    cap = cv2.VideoCapture(index)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            Path("data/images").mkdir(parents=True, exist_ok=True)
            filename = "data/images/camera_test.jpg"
            cv2.imwrite(filename, frame)
            print(f"  ✅ Test image saved to: {filename}")
        cap.release()

def main():
    print("=" * 50)
    print("🎥 LOGITECH CAMERA TEST FOR RASPBERRY PI")
    print("=" * 50)
    
    # Check for video devices
    devices = check_camera_devices()
    
    if not devices:
        print("\n❌ No camera devices found!")
        diagnose_issues()
        return
    
    # Test each camera index
    working_cameras = []
    for i in range(4):
        if test_camera_index(i):
            working_cameras.append(i)
    
    if working_cameras:
        print(f"\n✅ Found working cameras at indices: {working_cameras}")
        
        # Use first working camera for additional tests
        best_index = working_cameras[0]
        test_v4l2_controls(best_index)
        save_test_image(best_index)
        
        print(f"\n🎯 Recommended camera index: {best_index}")
        print(f"   Use this in your main script: cv2.VideoCapture({best_index})")
    else:
        print("\n❌ No working cameras found!")
        diagnose_issues()
        
        print("\n📋 Troubleshooting steps:")
        print("1. Check USB connection")
        print("2. Try different USB ports (USB 3.0 preferred)")
        print("3. Check power supply (webcam needs sufficient power)")
        print("4. Run with sudo: sudo python3 test_camera.py")
        print("5. Install v4l-utils: sudo apt-get install v4l-utils")
        print("6. Check dmesg for errors: dmesg | grep -i camera")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
