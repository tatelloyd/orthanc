#!/usr/bin/env python3
"""
YOLO Object Detection - Simple Test Mode
Detects objects and prints them to the screen
"""

import cv2
import sys
import time
from ultralytics import YOLO

def main():
    print("\n=== YOLO Detection Test Mode ===")
    print("Detecting objects for 10 seconds...\n")
    
    # Load YOLO model
    print("Loading YOLOv8n model...")
    try:
        model = YOLO('yolov8n.pt')
        print("✅ Model loaded successfully!\n")
    except Exception as e:
        print(f"❌ Failed to load YOLO model: {e}")
        return 1
    
    # Open camera
    print("Opening camera...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ Cannot open camera")
        return 1
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ Camera opened: {width}x{height}\n")
    
    # Detection settings
    detection_duration = 10.0  # Run for 10 seconds
    target_fps = 5  # Check for objects 5 times per second
    frame_delay = 1.0 / target_fps
    
    print(f"🎯 Running detection at {target_fps} Hz for {detection_duration}s")
    print("=" * 60)
    
    start_time = time.time()
    last_detection_time = start_time
    frame_count = 0
    
    try:
        while time.time() - start_time < detection_duration:
            current_time = time.time()
            
            # Throttle detection rate
            if current_time - last_detection_time < frame_delay:
                time.sleep(0.01)
                continue
            
            last_detection_time = current_time
            
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("⚠️  Failed to grab frame")
                continue
            
            # FLIP IMAGE since camera is mounted upside down
            frame = cv2.flip(frame, -1)  # -1 flips both horizontal and vertical
            
            # Run YOLO detection
            results = model(frame, verbose=False)
            
            # Process and print detections
            detections = []
            for box in results[0].boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls]
                
                # Only include high-confidence detections
                if conf > 0.5:
                    # Get bounding box center
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    center_x = ((x1 + x2) / 2) / width
                    center_y = ((y1 + y2) / 2) / height
                    
                    detections.append({
                        'label': label,
                        'confidence': conf,
                        'x': center_x,
                        'y': center_y
                    })
            
            # Print results
            frame_count += 1
            elapsed = time.time() - start_time
            
            if detections:
                print(f"[{elapsed:.1f}s] Frame {frame_count}: Found {len(detections)} object(s)")
                for det in detections:
                    print(f"  → {det['label']} "
                          f"(confidence: {det['confidence']:.2f}, "
                          f"position: x={det['x']:.2f}, y={det['y']:.2f})")
            else:
                print(f"[{elapsed:.1f}s] Frame {frame_count}: No objects detected")
    
    except KeyboardInterrupt:
        print("\n\n🛑 Detection stopped by user")
    finally:
        cap.release()
    
    print("\n" + "=" * 60)
    print(f"✅ Detection test completed! Processed {frame_count} frames")
    print()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())