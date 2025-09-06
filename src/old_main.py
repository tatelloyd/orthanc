#!/usr/bin/env python3
"""
Basic Object Detection System
Phase 0: Camera + Detection Only (No hardware yet)
"""

import cv2
import numpy as np
from ultralytics import YOLO
import time
from pathlib import Path

class SimpleDetector:
    def __init__(self, camera_index=0, model_path='yolov8n.pt'):
        """Initialize the detector with camera and YOLO model"""
        
        # Camera setup
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            # Try different camera indices if 0 doesn't work
            for i in range(1, 4):
                print(f"Trying camera index {i}...")
                self.cap = cv2.VideoCapture(i)
                if self.cap.isOpened():
                    print(f"✅ Camera found at index {i}")
                    break
            else:
                raise RuntimeError("❌ No camera found. Check USB connection.")
        
        # Set camera properties for better performance on Pi
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer for lower latency
        
        # Load YOLO model
        print(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        
        # Performance tracking
        self.fps = 0
        self.frame_count = 0
        self.start_time = time.time()
        
        # Detection settings
        self.confidence_threshold = 0.5
        self.target_classes = ['person', 'car', 'truck', 'bus', 'bicycle', 'motorcycle']
        
        print("✅ Detector initialized successfully!")
        
    def detect_objects(self, frame):
        """Run YOLO detection on a frame"""
        # Run inference
        results = self.model(frame, conf=self.confidence_threshold, verbose=False)
        
        detections = []
        
        # Process results
        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # Get class and confidence
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # Get class name
                    class_name = self.model.names[cls]
                    
                    # Store detection
                    detection = {
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'class': class_name,
                        'confidence': conf,
                        'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                    }
                    detections.append(detection)
        
        return detections
    
    def draw_detections(self, frame, detections):
        """Draw bounding boxes and labels on frame"""
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            class_name = det['class']
            conf = det['confidence']
            center = det['center']
            
            # Choose color based on class (optional filtering for target classes)
            if class_name in self.target_classes:
                color = (0, 255, 0)  # Green for targets
            else:
                color = (255, 0, 0)  # Blue for others
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw center point
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            
            # Draw label with confidence
            label = f"{class_name}: {conf:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            
            # Background for label
            cv2.rectangle(frame, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), color, -1)
            
            # Text label
            cv2.putText(frame, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Print detection to console
            print(f"Detected: {class_name} at ({center[0]}, {center[1]}) - Confidence: {conf:.2f}")
        
        return frame
    
    def update_fps(self):
        """Calculate and update FPS"""
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        
        if elapsed > 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.start_time = time.time()
        
        return self.fps
    
    def draw_info(self, frame):
        """Draw FPS and other info on frame"""
        # FPS counter
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Instructions
        cv2.putText(frame, "Press 'q' to quit, 's' to save image", (10, 460),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame
    
    def run(self):
        """Main detection loop"""
        print("\n🚀 Starting detection system...")
        print("Press 'q' to quit, 's' to save current frame")
        
        # Create data directory if it doesn't exist
        Path("data/images").mkdir(parents=True, exist_ok=True)
        
        while True:
            # Read frame
            ret, frame = self.cap.read()
            if not ret:
                print("❌ Failed to read frame")
                break
            
            # Run detection
            detections = self.detect_objects(frame)
            
            # Draw results
            frame = self.draw_detections(frame, detections)
            
            # Update and draw FPS
            self.update_fps()
            frame = self.draw_info(frame)
            
            # Display frame
            cv2.imshow('Object Detection System', frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\n👋 Shutting down...")
                break
            elif key == ord('s'):
                # Save current frame
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"data/images/detection_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"💾 Saved image: {filename}")
        
        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()
        print("✅ System shutdown complete")

def test_camera_connection():
    """Quick test to find and verify camera"""
    print("🔍 Testing camera connection...")
    
    for i in range(4):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"✅ Camera found at index {i}")
                print(f"   Resolution: {frame.shape[1]}x{frame.shape[0]}")
                cap.release()
                return i
            cap.release()
    
    print("❌ No camera found. Check USB connection.")
    return None

if __name__ == "__main__":
    # First, test camera
    camera_index = test_camera_connection()
    
    if camera_index is not None:
        try:
            # Initialize detector
            detector = SimpleDetector(camera_index=camera_index)
            
            # Run detection system
            detector.run()
            
        except Exception as e:
            print(f"❌ Error: {e}")
            import traceback
            traceback.print_exc()
    else:
        print("\n⚠️  Please connect your Logitech webcam and try again.")
        print("Tips:")
        print("  - Check USB connection")
        print("  - Try different USB ports")
        print("  - Run 'ls /dev/video*' to see available cameras")
        print("  - You may need to run with 'sudo' for camera access")
