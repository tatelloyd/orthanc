#!/usr/bin/env python3
"""
YOLO Object Detection - Improved Stability and False Positive Rejection
"""

import cv2
import sys
import time
import json
import fcntl
from collections import deque
from ultralytics import YOLO

class StableTracker:
    """Tracks detections over time to filter out false positives"""
    def __init__(self, buffer_size=3, position_threshold=0.15):
        self.detections = deque(maxlen=buffer_size)
        self.position_threshold = position_threshold
        
    def add_detection(self, detection):
        """Add a detection and return stable position if confident"""
        self.detections.append(detection)
        
        # Need at least 2 recent detections to be confident
        if len(self.detections) < 2:
            return None
            
        # Check if recent detections are consistent (close together)
        recent = list(self.detections)
        avg_x = sum(d['x'] for d in recent) / len(recent)
        avg_y = sum(d['y'] for d in recent) / len(recent)
        
        # Check variance - reject if detections are jumping around
        max_dist = max(
            ((d['x'] - avg_x)**2 + (d['y'] - avg_y)**2)**0.5 
            for d in recent
        )
        
        if max_dist > self.position_threshold:
            return None  # Too much variance, not stable
            
        # Return averaged position
        return {
            'label': 'person',
            'confidence': sum(d['confidence'] for d in recent) / len(recent),
            'x': avg_x,
            'y': avg_y,
            'stable': True
        }
    
    def clear(self):
        self.detections.clear()

def is_valid_detection(center_x, center_y, conf):
    """Strict validation for person detections"""
    
    # CRITICAL: Much stricter edge rejection
    # Camera is upside down, so y < 0.2 is actually TOP of real view (ceiling/floor)
    # Reject anything in top 25% or bottom 25% of frame
    if center_y < 0.25 or center_y > 0.75:
        return False
    
    # Reject edges on x-axis too
    if center_x < 0.1 or center_x > 0.9:
        return False
    
    # Require minimum confidence
    if conf < 0.3:
        return False
        
    return True

def main():
    print("\n=== YOLO Detection Service (Improved Stability) ===")
    
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
    
    # Stable tracking
    tracker = StableTracker(buffer_size=3, position_threshold=0.12)
    
    print(f"🎯 Detection service running...")
    print("=" * 60)
    
    start_time = time.time()
    last_detection_time = start_time
    frame_count = 0
    
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
            
            # Run YOLO detection with moderate confidence
            results = model(frame, verbose=False, conf=0.3, imgsz=320)
            
            # Process detections
            raw_detections = []
            person_candidates = []
            
            for box in results[0].boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls]
                
                # Get bounding box center
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                center_x = ((x1 + x2) / 2) / width
                center_y = ((y1 + y2) / 2) / height
                
                if label == 'person':
                    # Apply strict validation
                    if not is_valid_detection(center_x, center_y, conf):
                        print(f"   ⚠️ Rejected person at ({center_x:.3f}, {center_y:.3f}) conf={conf:.2f} - edge/low conf")
                        continue
                    
                    person_candidates.append({
                        'label': label,
                        'confidence': float(conf),
                        'x': float(center_x),
                        'y': float(center_y)
                    })
                else:
                    # Track other objects for debugging
                    raw_detections.append({
                        'label': label,
                        'confidence': float(conf),
                        'x': float(center_x),
                        'y': float(center_y)
                    })
            
            # Select best person candidate (closest to center, high confidence)
            detections = []
            stable_person = None
            
            if person_candidates:
                # Prefer detections near center of frame
                best = max(person_candidates, key=lambda d: 
                    d['confidence'] / (1.0 + ((d['x']-0.5)**2 + (d['y']-0.5)**2))
                )
                
                # Add to tracker for temporal filtering
                stable_person = tracker.add_detection(best)
                
                if stable_person:
                    detections.append(stable_person)
                    print(f"   ✅ Stable person at ({stable_person['x']:.3f}, {stable_person['y']:.3f}) conf={stable_person['confidence']:.2f}")
                else:
                    # Not stable yet, but still output best candidate
                    detections.append(best)
                    print(f"   ⏳ Tracking candidate at ({best['x']:.3f}, {best['y']:.3f}) conf={best['confidence']:.2f}")
            else:
                tracker.clear()
            
            # Print compact summary
            frame_count += 1
            elapsed = time.time() - start_time
            
            person_count = len(detections)
            other_count = len(raw_detections)
            
            if person_count > 0:
                status = f"👤 {person_count} person {'(stable)' if stable_person else '(tracking)'}"
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
                'detections': detections  # Only stable/best person
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