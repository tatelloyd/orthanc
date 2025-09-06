# test_setup.py
import cv2
import numpy as np
from ultralytics import YOLO
import sys

def test_camera():
    """Test camera access"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Camera not accessible")
        return False
    
    ret, frame = cap.read()
    cap.release()
    
    if ret:
        print("✅ Camera working")
        return True
    else:
        print("❌ Camera not working")
        return False

def test_yolo():
    """Test YOLO model"""
    try:
        model = YOLO('models/yolo/yolov8n.pt')
        print("✅ YOLO model loaded")
        return True
    except Exception as e:
        print(f"❌ YOLO model error: {e}")
        return False

if __name__ == "__main__":
    print("Testing setup...")
    camera_ok = test_camera()
    yolo_ok = test_yolo()
    
    if camera_ok and yolo_ok:
        print("\n🎉 Setup successful! Ready to build your tracking system.")
    else:
        print("\n⚠️  Some issues detected. Check the errors above.")
