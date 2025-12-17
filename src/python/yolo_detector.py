#!/usr/bin/env python3
"""
YOLO Object Detection - Continuous Mode with Improved Reliability
"""

import cv2
import sys
import time
import json
import fcntl
from ultralytics import YOLO

def main():
    print("\n=== YOLO Detection Service ===")
    
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
    
    # Set camera to lower resolution for faster processing
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ Camera opened: {width}x{height}\n")
    
    # Detection settings
    target_fps = 10
    frame_delay = 1.0 / target_fps
    
    print(f"🎯 Detection service running...")
    print("=" * 60)
    
    start_time = time.time()
    last_detection_time = start_time
    frame_count = 0
    
    # Temporal smoothing - remember last few detections
    last_person_position = None
    frames_since_person = 0
    
    try:
        while True:
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
            frame = cv2.flip(frame, -1)
            
            # Run YOLO detection with LOWER confidence threshold for better detection
            results = model(frame, verbose=False, conf=0.25, imgsz=320)  # Lowered from 0.35
            
            # Process detections
            detections = []
            person_found = False
            
            for box in results[0].boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls]
                
                # Get bounding box center
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                center_x = ((x1 + x2) / 2) / width
                center_y = ((y1 + y2) / 2) / height
                
                # CRITICAL FIX: Reject detections at edges (likely false positives)
                # Camera is upside down, so y < 0.15 means top of ACTUAL view (person's feet/floor)
                if center_y < 0.15 or center_y > 0.85:
                    continue  # Skip edge detections
                if center_x < 0.05 or center_x > 0.95:
                    continue  # Skip edge detections
                
                detection = {
                    'label': label,
                    'confidence': float(conf),
                    'x': float(center_x),
                    'y': float(center_y)
                }
                
                detections.append(detection)
                
                if label == 'person':
                    person_found = True
                    last_person_position = {'x': center_x, 'y': center_y}
                    frames_since_person = 0
                    
                    # Boost confidence for people near center (more likely to be the target)
                    distance_from_center = ((center_x - 0.5)**2 + (center_y - 0.5)**2)**0.5
                    if distance_from_center < 0.2:  # Within 20% of center
                        detection['confidence'] = min(1.0, conf * 1.2)  # Boost confidence
            
            # If no person found but we had one recently, use interpolation
            if not person_found and last_person_position and frames_since_person < 5:
                # Add the last known position with a flag
                detections.append({
                    'label': 'person',
                    'confidence': 0.4,  # Lower confidence to indicate interpolation
                    'x': last_person_position['x'],
                    'y': last_person_position['y'],
                    'interpolated': True
                })
                person_found = True
                frames_since_person += 1
            elif not person_found:
                frames_since_person += 1
            
            # Print compact summary
            frame_count += 1
            elapsed = time.time() - start_time
            
            person_count = sum(1 for d in detections if d['label'] == 'person')
            other_count = len(detections) - person_count
            
            if person_count > 0:
                status = f"👤 {person_count} person(s)"
                if other_count > 0:
                    status += f" + {other_count} other"
            elif other_count > 0:
                status = f"{other_count} objects (no people)"
            else:
                status = "0 objects (no people)"
            
            print(f"[{elapsed:6.1f}s] Frame {frame_count:4d}: {status}")
            
            # Write to JSON
            detection_data = {
                'timestamp': time.time(),
                'detections': detections
            }
            
            try:
                with open('detections.json', 'w') as f:
                    fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                    json.dump(detection_data, f)
                    f.flush()
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
            except Exception as e:
                print(f"⚠️  Failed to write JSON: {e}")
    
    except KeyboardInterrupt:
        print("\n\n🛑 Detection stopped by user")
    finally:
        cap.release()
    
    print("\n" + "=" * 60)
    print(f"✅ Detection service completed! Processed {frame_count} frames")
    print()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())