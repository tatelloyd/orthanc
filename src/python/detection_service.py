#!/usr/bin/env python3
"""
Detection Service - Continuous YOLO detection with IPC output
Writes detection results to shared file/pipe for C++ tracker
"""

import cv2
import sys
import time
import json
import os
import argparse
from pathlib import Path
from ultralytics import YOLO

class DetectionService:
    def __init__(self, output_file='detections.json', target_fps=10):
        self.output_file = Path(output_file)
        self.target_fps = target_fps
        self.frame_delay = 1.0 / target_fps
        
        print("\n=== YOLO Detection Service ===")
        print(f"Output: {self.output_file}")
        print(f"Detection rate: {target_fps} Hz\n")
        
        # Load YOLO model
        print("Loading YOLOv8n model...")
        try:
            self.model = YOLO('yolov8n.pt')
            print("✅ Model loaded\n")
        except Exception as e:
            print(f"❌ Failed to load YOLO: {e}")
            sys.exit(1)
        
        # Open camera
        print("Opening camera...")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("❌ Cannot open camera")
            sys.exit(1)
        
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"✅ Camera: {width}x{height}\n")
        
        self.width = width
        self.height = height
        
    def should_keep_detection(self, conf, x, y):
        """Adaptive confidence threshold based on position in frame"""
        import math
        dist_from_center = math.sqrt((x - 0.5)**2 + (y - 0.5)**2)
        
        # Center region (dist < 0.3): require 0.6 confidence
        # Mid region (0.3-0.4): require 0.5 confidence  
        # Edge region (dist > 0.4): accept 0.45 confidence
        if dist_from_center < 0.3:
            return conf > 0.6
        elif dist_from_center < 0.4:
            return conf > 0.5
        else:
            return conf > 0.45
    
    def detect_frame(self):
        """Capture and process one frame"""
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        # Flip for upside-down camera
        frame = cv2.flip(frame, -1)
        
        # Run detection
        results = self.model(frame, verbose=False)
        
        # Extract detections
        detections = []
        for box in results[0].boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = self.model.names[cls]
            
            # Calculate position first - convert numpy arrays to float immediately
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].cpu().numpy()]
            center_x = (x1 + x2) / 2 / self.width
            center_y = (y1 + y2) / 2 / self.height
            
            # Use adaptive threshold based on position
            if self.should_keep_detection(conf, center_x, center_y):
                detections.append({
                    'label': label,
                    'confidence': float(round(conf, 3)),
                    'x': float(round(center_x, 3)),
                    'y': float(round(center_y, 3)),
                    'timestamp_ms': int(time.time() * 1000)
                })
        
        return detections
    
    def write_detections(self, detections):
        """Write detections to JSON file atomically"""
        try:
            with open(self.output_file, 'w') as f:
                json.dump(detections, f)
                f.flush()
                os.fsync(f.fileno())
        except Exception as e:
            print(f"⚠️  Write error: {e}")
    
    def run(self, duration=None):
        """Main detection loop"""
        print("🎯 Detection service running...")
        print("=" * 60)
        
        start_time = time.time()
        last_detection = start_time
        frame_count = 0
        
        try:
            while True:
                current_time = time.time()
                
                # Check duration limit
                if duration and (current_time - start_time) >= duration:
                    break
                
                # Throttle detection rate
                if current_time - last_detection < self.frame_delay:
                    time.sleep(0.01)
                    continue
                
                last_detection = current_time
                
                # Process frame
                detections = self.detect_frame()
                if detections is None:
                    print("⚠️  Frame capture failed")
                    continue
                
                # Write results
                self.write_detections(detections)
                
                # Status update
                frame_count += 1
                elapsed = current_time - start_time
                
                people_count = sum(1 for d in detections if d['label'] == 'person')
                if people_count > 0:
                    print(f"[{elapsed:6.1f}s] Frame {frame_count:4d}: "
                          f"👤 {people_count} person(s) + {len(detections)-people_count} other")
                else:
                    print(f"[{elapsed:6.1f}s] Frame {frame_count:4d}: "
                          f"{len(detections)} objects (no people)")
                
        except KeyboardInterrupt:
            print("\n\n🛑 Detection stopped by user")
        finally:
            self.cleanup()
        
        print(f"\n✅ Processed {frame_count} frames in {time.time()-start_time:.1f}s")
    
    def cleanup(self):
        """Release resources"""
        self.cap.release()
        # Clear output file
        self.write_detections([])
        print("🧹 Resources released")

def main():
    parser = argparse.ArgumentParser(description='YOLO Detection Service')
    parser.add_argument('--output', default='detections.json',
                       help='Output JSON file (default: detections.json)')
    parser.add_argument('--fps', type=int, default=10,
                       help='Detection rate in Hz (default: 10)')
    parser.add_argument('--duration', type=float, default=None,
                       help='Run duration in seconds (default: infinite)')
    
    args = parser.parse_args()
    
    service = DetectionService(output_file=args.output, target_fps=args.fps)
    service.run(duration=args.duration)
    
    return 0

if __name__ == "__main__":
    sys.exit(main())