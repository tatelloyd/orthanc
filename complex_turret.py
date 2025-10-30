#!/usr/bin/env python3
"""
Enhanced Turret Controller with AI Agent Support
Run with: python simple_turret.py [manual|ai]
"""

import cv2
import time
import threading
import sys
import os
import asyncio
from ultralytics import YOLO
import RPi.GPIO as GPIO

# Import AI agent (assumes ai_agent.py is in same directory)
try:
    from ai_agent import AITurretAgent
    AI_AVAILABLE = True
except ImportError:
    AI_AVAILABLE = False
    print("Warning: AI agent not available (ai_agent.py not found)")

class SimpleServo:
    def __init__(self, pin=16):
        """Simple servo controller"""
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, 50)  # 50Hz
        self.current_angle = 90  # Start at center
        
        duty = self.angle_to_duty(90)
        self.pwm.start(duty)
        print(f"Servo initialized on GPIO {pin} at center (90 degrees)")
    
    def angle_to_duty(self, angle):
        """Convert angle (0-180) to duty cycle"""
        min_duty = 2.5
        max_duty = 12.5
        return min_duty + (angle / 180.0) * (max_duty - min_duty)
    
    def move_to(self, angle):
        """Move servo to specific angle with holding"""
        angle = max(0, min(180, angle))
        duty = self.angle_to_duty(angle)
        self.pwm.ChangeDutyCycle(duty)
        self.current_angle = angle
        time.sleep(0.5)  # Give time to move
        # Keep duty cycle active for holding torque
        print(f"Servo moved to {angle} degrees")
    
    def cleanup(self):
        """Clean up GPIO"""
        self.pwm.stop()
        GPIO.cleanup()

class SimpleLaser:
    def __init__(self, pin=18):
        """Simple laser controller"""
        self.pin = pin
        self.is_on = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    
    def toggle(self):
        """Toggle laser on/off"""
        self.is_on = not self.is_on
        GPIO.output(self.pin, GPIO.HIGH if self.is_on else GPIO.LOW)
        print(f"Laser: {'ON' if self.is_on else 'OFF'}")
    
    def set_state(self, on):
        """Set laser on or off"""
        self.is_on = on
        GPIO.output(self.pin, GPIO.HIGH if on else GPIO.LOW)

class SimpleDetector:
    def __init__(self, model_path='yolov8n.pt'):
        """Simple object detector"""
        print(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.confidence_threshold = 0.5
        
        # Find camera
        self.camera = None
        for i in range(4):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    self.camera = cap
                    self.camera_index = i
                    print(f"Camera found at index {i}")
                    break
                cap.release()
        
        if not self.camera:
            raise RuntimeError("No camera found")
        
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    def detect_and_print(self):
        """Capture frame, detect objects, and print results"""
        ret, frame = self.camera.read()
        if not ret:
            print("Failed to read frame")
            return []
        
        results = self.model(frame, conf=self.confidence_threshold, verbose=False)
        
        detections = []
        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = self.model.names[cls]
                    
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    detection = {
                        'class': class_name,
                        'confidence': conf,
                        'center': (center_x, center_y),
                        'bbox': [int(x1), int(y1), int(x2), int(y2)]
                    }
                    detections.append(detection)
        
        if detections:
            print(f"\n--- DETECTED OBJECTS ({len(detections)}) ---")
            for i, det in enumerate(detections, 1):
                print(f"{i}. {det['class']} - confidence: {det['confidence']:.2f}")
                print(f"   Center: {det['center']}")
        else:
            print("\n--- NO OBJECTS DETECTED ---")
        
        return detections, frame

    def detect_with_visual(self):
        """Show visual feedback for tracking"""
        ret, frame = self.camera.read()
        if not ret:
            return [], frame
    
        results = self.model(frame, conf=self.confidence_threshold, verbose=False)
    
        # Draw on frame
        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                    # Draw center point
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                
                    # Draw target line (image center)
                    cv2.line(frame, (320, 0), (320, 480), (255, 0, 0), 1)
    
        cv2.imshow('Tracking', frame)
        cv2.waitKey(1)
    
        return detections, frame

    
    def cleanup(self):
        """Release camera"""
        if self.camera:
            self.camera.release()

class SimpleTurret:
    def __init__(self):
        """Initialize simple turret system"""
        print("Initializing Simple Turret System...")
        
        try:
            self.servo = SimpleServo(pin=16)
        except Exception as e:
            print(f"Servo initialization failed: {e}")
            self.servo = None
        
        try:
            self.laser = SimpleLaser(pin=18)
        except Exception as e:
            print(f"Laser initialization failed: {e}")
            self.laser = None
        
        try:
            self.detector = SimpleDetector()
        except Exception as e:
            print(f"Detector initialization failed: {e}")
            self.detector = None
        
        self.detecting = False
        self.detection_thread = None
        
        print("Simple Turret System ready!")
    
    def start_detection(self):
        """Start continuous detection in background"""
        if not self.detector:
            print("No detector available")
            return
        
        if self.detecting:
            print("Detection already running")
            return
        
        self.detecting = True
        self.detection_thread = threading.Thread(target=self._detection_loop)
        self.detection_thread.daemon = True
        self.detection_thread.start()
        print("Started continuous detection")
    
    def stop_detection(self):
        """Stop continuous detection"""
        self.detecting = False
        if self.detection_thread:
            self.detection_thread.join()
        print("Stopped continuous detection")
    
    def _detection_loop(self):
        """Background detection loop"""
        while self.detecting:
            try:
                self.detector.detect_and_print()
                time.sleep(2)
            except Exception as e:
                print(f"Detection error: {e}")
                time.sleep(1)
    
    def manual_control(self):
        """Interactive manual control"""
        print("\n" + "="*40)
        print("SIMPLE TURRET MANUAL CONTROL")
        print("="*40)
        print("Commands:")
        #print("  l       - Toggle laser")
        #print("  d       - Detect objects once")
        print("  s       - Start/stop continuous detection")
        print("  0-180   - Move servo to angle (e.g., '90')")
        #print("  left    - Move servo left (30 degrees)")
        #print("  right   - Move servo right (150 degrees)")
        #print("  center  - Center servo (90 degrees)")
        print("  q       - Quit")
        print("="*40)
        
        try:
            while True:
                cmd = input("\nCommand: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 'l':
                    if self.laser:
                        self.laser.toggle()
                    else:
                        print("Laser not available")
                elif cmd == 'd':
                    if self.detector:
                        self.detector.detect_and_print()
                    else:
                        print("Detector not available")
                elif cmd == 's':
                    if self.detecting:
                        self.stop_detection()
                    else:
                        self.start_detection()
                elif cmd in ['left', 'right', 'center']:
                    if self.servo:
                        angles = {'left': 30, 'right': 150, 'center': 90}
                        self.servo.move_to(angles[cmd])
                    else:
                        print("Servo not available")
                else:
                    try:
                        angle = float(cmd)
                        if 0 <= angle <= 180:
                            if self.servo:
                                self.servo.move_to(angle)
                            else:
                                print("Servo not available")
                        else:
                            print("Angle must be between 0 and 180")
                    except ValueError:
                        print("Unknown command. Type 'q' to quit.")
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.cleanup()
    
    async def ai_control(self):
        """AI-controlled autonomous mode"""
        if not AI_AVAILABLE:
            print("ERROR: AI agent not available. Make sure ai_agent.py is installed.")
            return
        
        print("\n" + "="*40)
        print("AI AUTONOMOUS MODE")
        print("="*40)
        print("The AI will control the turret automatically.")
        print("Press Ctrl+C to stop.")
        print("="*40 + "\n")
        
        # Initialize AI agent
        agent = AITurretAgent(self, mode="claude")
        
        try:
            # Run autonomous mode
            await agent.autonomous_mode()
        except KeyboardInterrupt:
            print("\n\nStopping AI mode...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up all resources"""
        print("\nCleaning up...")
        
        self.stop_detection()
        
        if self.laser:
            self.laser.set_state(False)
        
        if self.servo:
            self.servo.cleanup()
        
        if self.detector:
            self.detector.cleanup()
        
        print("Cleanup complete")

def main():
    """Main function"""
    print("Simple Turret Controller")
    print("=" * 40)
    
    # Check command line arguments
    mode = sys.argv[1] if len(sys.argv) > 1 else "manual"
    
    if mode not in ["manual", "ai"]:
        print("Usage: python simple_turret.py [manual|ai]")
        print("  manual - Manual terminal control (default)")
        print("  ai     - AI autonomous control")
        sys.exit(1)
    
    try:
        turret = SimpleTurret()
        
        if mode == "manual":
            turret.manual_control()
        else:  # ai mode
            asyncio.run(turret.ai_control())
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
