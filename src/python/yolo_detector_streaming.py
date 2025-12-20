#!/usr/bin/env python3
"""
YOLO Object Detection with Web Streaming for Remote Debugging
Run on Pi, view on Mac at http://<pi-ip>:5000
"""

import cv2
import sys
import time
import json
import fcntl
import threading
from collections import deque
from flask import Flask, Response
from ultralytics import YOLO

# Configuration
STREAM = True  # Set to False to disable streaming (save resources)
STREAM_PORT = 5000

app = Flask(__name__)

class StableTracker:
    """Tracks detections over time to filter out false positives"""
    def __init__(self, buffer_size=3, position_threshold=0.15):
        self.detections = deque(maxlen=buffer_size)
        self.position_threshold = position_threshold
        
    def add_detection(self, detection):
        """Add a detection and return stable position if confident"""
        self.detections.append(detection)
        
        # Return none if there aren't consistent detections.
        if len(self.detections) < 2:
            return None
            
        # Return average coordinates of detections.
        recent = list(self.detections)
        avg_x = sum(d['x'] for d in recent) / len(recent)
        avg_y = sum(d['y'] for d in recent) / len(recent)
        

        # Return no detections if the max distance of the distances
        # is greater than the allowed threshold.
        max_dist = max(
            ((d['x'] - avg_x)**2 + (d['y'] - avg_y)**2)**0.5 
            for d in recent
        )
        
        if max_dist > self.position_threshold:
            return None
            
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
    if center_y < 0.25 or center_y > 0.75:
        return False
    if center_x < 0.1 or center_x > 0.9:
        return False
    if conf < 0.3:
        return False
    return True

# Global variables for frame sharing
latest_frame = None
frame_lock = threading.Lock()
detection_info = {}

def annotate_frame(frame, results, model, width, height, person_candidates, stable_person, tracker_state):
    """Draw all camera information on frame"""
    annotated = frame.copy()
    
    # Draw all YOLO detections
    for box in results[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        label = model.names[cls]
        
        center_x = ((x1 + x2) / 2) / width
        center_y = ((y1 + y2) / 2) / height
        
        # Color coding
        if label == 'person':
            is_valid = is_valid_detection(center_x, center_y, conf)
            color = (0, 255, 0) if is_valid else (0, 0, 255)  # Green if valid, Red if rejected
            thickness = 3 if is_valid else 2
        else:
            color = (0, 255, 255)  # Yellow for other objects
            thickness = 1
        
        # Draw bounding box
        cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
        
        # Draw label
        label_text = f"{label} {conf:.2f}"
        cv2.putText(annotated, label_text, (int(x1), int(y1)-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    # Draw center crosshair (turret aim point)
    center_x = width // 2
    center_y = height // 2
    cv2.line(annotated, (center_x - 30, center_y), (center_x + 30, center_y), (255, 0, 0), 2)
    cv2.line(annotated, (center_x, center_y - 30), (center_x, center_y + 30), (255, 0, 0), 2)
    cv2.circle(annotated, (center_x, center_y), 5, (255, 0, 0), -1)
    
    # Draw deadband zone (where tracking stops adjusting)
    deadband = 0.1
    deadband_x = int(deadband * width)
    deadband_y = int(deadband * height)
    cv2.rectangle(annotated, 
                  (center_x - deadband_x, center_y - deadband_y),
                  (center_x + deadband_x, center_y + deadband_y),
                  (128, 128, 128), 2)
    
    # Draw valid detection zone (edge rejection boundaries)
    valid_top = int(0.25 * height)
    valid_bottom = int(0.75 * height)
    valid_left = int(0.1 * width)
    valid_right = int(0.9 * width)
    cv2.rectangle(annotated, (valid_left, valid_top), (valid_right, valid_bottom), 
                  (255, 128, 0), 1)  # Orange boundary
    
    # Status text overlay
    status_y = 30
    if stable_person:
        status = f"TRACKING: Stable person at ({stable_person['x']:.2f}, {stable_person['y']:.2f})"
        color = (0, 255, 0)
    elif person_candidates:
        status = f"SEARCHING: {len(person_candidates)} candidate(s) (not stable yet)"
        color = (0, 255, 255)
    else:
        status = "SCANNING: No person detected"
        color = (0, 0, 255)
    
    cv2.putText(annotated, status, (10, status_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    
    # Detection count
    cv2.putText(annotated, f"Objects: {len(results[0].boxes)}", (10, status_y + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Frame rate
    cv2.putText(annotated, f"FPS: {tracker_state.get('fps', 0):.1f}", (10, status_y + 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    return annotated

def generate_frames():
    """Generator for Flask streaming"""
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.1)
                continue
            frame = latest_frame.copy()
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            continue
            
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    """Serve simple HTML page"""
    return """
    <html>
        <head>
            <title>YOLO Tracker </title>
            <style>
                body { margin: 0; padding: 20px; background: #1a1a1a; text-align: center; }
                h1 { color: #fff; font-family: Arial; }
                img { max-width: 95vw; max-height: 85vh; border: 2px solid #444; }
                .info { color: #aaa; font-family: monospace; margin-top: 10px; }
            </style>
        </head>
        <body>
            <h1>🎯 YOLO Tracker Stream</h1>
            <img src="/video_feed" />
            <div class="info">
                <p>🟢 Green box = Valid person detection | 🔴 Red box = Rejected detection</p>
                <p>🔵 Blue crosshair = Turret aim point | ⬜ Gray box = Deadband (no adjustment)</p>
                <p>🟧 Orange box = Valid detection zone (edges rejected)</p>
            </div>
        </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def detection_loop():
    """Main detection loop (runs in separate thread)"""
    global latest_frame
    
    print("\n=== YOLO Detection Service (Web Streaming) ===")
    
    # Load YOLO model
    print("Loading YOLOv8n model...")
    try:
        model = YOLO('yolov8n.pt')
        print("✅ Model loaded successfully!\n")
    except Exception as e:
        print(f"❌ Failed to load YOLO model: {e}")
        return
    
    # Open camera
    print("Opening camera...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ Cannot open camera")
        return
    
    # Basic camera setup.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ Camera opened: {width}x{height}\n")
    
    target_fps = 15
    frame_delay = 1.0 / target_fps
    tracker = StableTracker(buffer_size=3, position_threshold=0.12)
    tracker_state = {'fps': 0}
    
    # If idicated, stream what camera sees to specified IP address and port.
    print(f"🎯 Detection service running...")
    if STREAM:
        import socket
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)
        print(f"🌐 View stream at: http://{local_ip}:{STREAM_PORT}")
    print("=" * 60)
    
    start_time = time.time()
    last_detection_time = start_time
    frame_count = 0
    fps_start = time.time()
    fps_frames = 0
    
    try:
        while True:
            current_time = time.time()
            
            if current_time - last_detection_time < frame_delay:
                time.sleep(0.01)
                continue
            
            last_detection_time = current_time
            
            ret, frame = cap.read()
            if not ret:
                print("⚠️  Failed to grab frame")
                continue
            
            # FLIP IMAGE since camera is mounted upside down
            frame = cv2.flip(frame, -1)
            
            # Run YOLO detection
            results = model(frame, verbose=False, conf=0.4, imgsz=160)
            
            # Process detections
            person_candidates = []
            
            # Dissect the reults for each box to:
            # 1) Determine what it is.
            # 2) Determine the confidence of the classification.
            for box in results[0].boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls]
                
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                center_x = ((x1 + x2) / 2) / width
                center_y = ((y1 + y2) / 2) / height
                
                if label == 'person':
                    if not is_valid_detection(center_x, center_y, conf):
                        print(f"   ⚠️ Rejected person at ({center_x:.3f}, {center_y:.3f}) conf={conf:.2f}")
                        continue
                    
                    person_candidates.append({
                        'label': label,
                        'confidence': float(conf),
                        'x': float(center_x),
                        'y': float(center_y)
                    })
            
            # Select best person candidate
            detections = []
            stable_person = None
            
            if person_candidates:
                best = max(person_candidates, key=lambda d: 
                    d['confidence'] / (1.0 + ((d['x']-0.5)**2 + (d['y']-0.5)**2))
                )
                
                stable_person = tracker.add_detection(best)
                
                if stable_person:
                    detections.append(stable_person)
                    print(f"   ✅ Stable person at ({stable_person['x']:.3f}, {stable_person['y']:.3f})")
                else:
                    detections.append(best)
                    print(f"   ⏳ Tracking candidate at ({best['x']:.3f}, {best['y']:.3f})")
            else:
                tracker.clear()
            
            # Calculate FPS
            fps_frames += 1
            if current_time - fps_start > 1.0:
                tracker_state['fps'] = fps_frames / (current_time - fps_start)
                fps_frames = 0
                fps_start = current_time
            
            # Annotate frame for streaming
            if STREAM:
                annotated_frame = annotate_frame(frame, results, model, width, height, 
                                                person_candidates, stable_person, tracker_state)
                with frame_lock:
                    latest_frame = annotated_frame
            
            # Print status
            frame_count += 1
            elapsed = time.time() - start_time
            person_count = len(detections)
            
            # Output if a person was found or not.
            if person_count > 0:
                status = f"👤 {person_count} person {'(stable)' if stable_person else '(tracking)'}"
            else:
                status = "0 objects (no people)"
            
            print(f"[{elapsed:6.1f}s] Frame {frame_count:4d}: {status}")
            
            # Write to JSON
            detection_data = {
                'timestamp': time.time(),
                'detections': detections
            }
            
            # Lock the file and then reset detections for the next cycle.
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

def main():
    if STREAM:
        # Start detection in background thread
        detection_thread = threading.Thread(target=detection_loop, daemon=True)
        detection_thread.start()
        
        # Start Flask server (blocking)
        print("\n🌐 Starting web server...")
        app.run(host='0.0.0.0', port=STREAM_PORT, threaded=True, debug=False)
    else:
        # Run detection only (no streaming)
        detection_loop()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())