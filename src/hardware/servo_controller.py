#!/usr/bin/env python3
"""
Servo Control Module for Pan-Tilt Mechanism - FIXED VERSION
Handles smooth servo movement with PID control
"""

import RPi.GPIO as GPIO
import time
import numpy as np
from threading import Thread, Lock
from collections import deque

class ServoController:
    def __init__(self, pan_pin=16, tilt_pin=24, pan_reversed=False, tilt_reversed=False, 
                 pan_offset=0, tilt_offset=0):  # Updated to your pins
        """
        Initialize servo controller
        
        Args:
            pan_pin: GPIO pin for pan servo (default 16)
            tilt_pin: GPIO pin for tilt servo (default 24)
            pan_reversed: Reverse pan servo direction
            tilt_reversed: Reverse tilt servo direction
            pan_offset: Offset for pan servo center position (-90 to +90)
            tilt_offset: Offset for tilt servo center position (-90 to +90)
        """
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Servo pins
        self.pan_pin = pan_pin
        self.tilt_pin = tilt_pin
        
        # Servo calibration
        self.pan_reversed = pan_reversed
        self.tilt_reversed = tilt_reversed
        self.pan_offset = pan_offset  # Offset from mechanical center
        self.tilt_offset = tilt_offset
        
        # Setup PWM (50Hz for servos)
        GPIO.setup(self.pan_pin, GPIO.OUT)
        GPIO.setup(self.tilt_pin, GPIO.OUT)
        self.pan_pwm = GPIO.PWM(self.pan_pin, 50)
        self.tilt_pwm = GPIO.PWM(self.tilt_pin, 50)
        
        # Servo limits (in degrees)
        self.pan_min = 0
        self.pan_max = 180
        self.tilt_min = 30  # Don't look too far down
        self.tilt_max = 150  # Don't look too far up
        
        # Current positions
        self.current_pan = 90  # Center
        self.current_tilt = 90  # Center
        
        # Target positions
        self.target_pan = 90
        self.target_tilt = 90
        
        # PID parameters - MUCH MORE CONSERVATIVE for stability
        self.kp = 0.3  # Reduced proportional gain
        self.ki = 0.01  # Much reduced integral gain
        self.kd = 0.05  # Reduced derivative gain
        
        # PID state
        self.pan_error_sum = 0
        self.pan_last_error = 0
        self.tilt_error_sum = 0
        self.tilt_last_error = 0
        
        # Smoothing - MUCH MORE CONSERVATIVE
        self.smoothing_factor = 0.1  # Much slower movement
        self.movement_threshold = 0.1  # Smaller threshold
        
        # Thread control
        self.running = False
        self.thread = None
        self.lock = Lock()
        
        # Movement history for smoothing
        self.pan_history = deque(maxlen=3)  # Reduced history
        self.tilt_history = deque(maxlen=3)
        
        # Start servos at center position with DEBUGGING
        center_duty = self._angle_to_duty(90)
        print(f"🔧 Starting PWM with duty cycle: {center_duty}")
        
        self.pan_pwm.start(center_duty)
        self.tilt_pwm.start(center_duty)
        
        # FORCE initial movement to test
        time.sleep(0.5)
        self._force_test_movement()
        
        print("✅ Servo controller initialized")
        print(f"   Pan servo on GPIO {pan_pin}")
        print(f"   Tilt servo on GPIO {tilt_pin}")
        print(f"   PWM frequency: 50Hz")
        print(f"   Center duty cycle: {center_duty}%")
        
    def _angle_to_duty(self, angle):
        """Convert angle (0-180) to duty cycle with calibration"""
        # Standard servo: 1ms-2ms pulse width = 5%-10% duty cycle
        # Many servos actually need: 1ms-2ms = 5%-10% (NOT 2.5%-12.5%)
        # Some servos are reversed or have different ranges
        
        # CALIBRATION: Adjust these values based on your servo behavior
        min_duty = 5.0   # Duty cycle for 0 degrees
        max_duty = 10.0  # Duty cycle for 180 degrees
        
        return min_duty + (angle / 180) * (max_duty - min_duty)
    
    def _clamp_angle(self, angle, min_angle, max_angle):
        """Clamp angle to valid range"""
        return max(min_angle, min(max_angle, angle))
        """Force test movement to verify servo connection"""
        print("🧪 Testing servo movement...")
        
        # Test pan servo
        print("   Testing pan servo (GPIO {})".format(self.pan_pin))
        for angle in [0, 45, 90, 135, 180, 90]:  # More comprehensive test
            calibrated = self._apply_calibration(angle, is_pan=True)
            duty = self._angle_to_duty(calibrated)
            print(f"      Setting pan to {angle}° → {calibrated}° (duty: {duty:.1f}%)")
            self.pan_pwm.ChangeDutyCycle(duty)
            time.sleep(1.5)  # Longer delay
        
        # Test tilt servo
        print("   Testing tilt servo (GPIO {})".format(self.tilt_pin))
        for angle in [30, 60, 90, 120, 150, 90]:  # Within tilt limits
            calibrated = self._apply_calibration(angle, is_pan=False)
            duty = self._angle_to_duty(calibrated)
            print(f"      Setting tilt to {angle}° → {calibrated}° (duty: {duty:.1f}%)")
            self.tilt_pwm.ChangeDutyCycle(duty)
            time.sleep(1.5)  # Longer delay
        
        print("   Test complete - did servos move?")
    
    def _apply_calibration(self, angle, is_pan=True):
        """Apply calibration (offset and reversal) to angle"""
        if is_pan:
            # Apply offset
            calibrated_angle = angle + self.pan_offset
            # Apply reversal
            if self.pan_reversed:
                calibrated_angle = 180 - calibrated_angle
        else:
            # Apply offset  
            calibrated_angle = angle + self.tilt_offset
            # Apply reversal
            if self.tilt_reversed:
                calibrated_angle = 180 - calibrated_angle
                
        # Clamp to valid range
        return max(0, min(180, calibrated_angle))
    
    def set_position(self, pan=None, tilt=None, immediate=False):
        """
        Set target position for servos
        
        Args:
            pan: Pan angle (0-180) or None to keep current
            tilt: Tilt angle (0-180) or None to keep current
            immediate: If True, move immediately without smoothing
        """
        with self.lock:
            if pan is not None:
                self.target_pan = self._clamp_angle(pan, self.pan_min, self.pan_max)
                print(f"🎯 Pan target set to {self.target_pan}°")
    def _force_test_movement(self):
            
            if tilt is not None:
                self.target_tilt = self._clamp_angle(tilt, self.tilt_min, self.tilt_max)
                print(f"🎯 Tilt target set to {self.target_tilt}°")
                if immediate:
                    self.current_tilt = self.target_tilt
                    duty = self._angle_to_duty(self.current_tilt)
                    self.tilt_pwm.ChangeDutyCycle(duty)
                    print(f"   Tilt duty cycle: {duty:.1f}%")
    
    def track_coordinate(self, x, y, frame_width, frame_height):
        """
        Track a coordinate in the camera frame
        
        Args:
            x, y: Pixel coordinates of target
            frame_width, frame_height: Frame dimensions
        """
        # Convert pixel coordinates to servo angles
        # Map x (0 to frame_width) to pan angle
        # Invert because servo moves opposite to screen coordinates
        pan_angle = 180 - (x / frame_width) * 180
        
        # Map y (0 to frame_height) to tilt angle
        tilt_angle = 180 - (y / frame_height) * 180
        
        # Apply smoothing
        self.pan_history.append(pan_angle)
        self.tilt_history.append(tilt_angle)
        
        if len(self.pan_history) > 1:
            # Use median for noise reduction
            smooth_pan = np.median(self.pan_history)
            smooth_tilt = np.median(self.tilt_history)
            self.set_position(smooth_pan, smooth_tilt)
        else:
            self.set_position(pan_angle, tilt_angle)
    
    def center_position(self):
        """Return servos to center position"""
        self.set_position(90, 90, immediate=True)
        print("🎯 Servos centered")
    
    def sweep_scan(self, speed=1):
        """
        Perform a scanning sweep motion
        
        Args:
            speed: Sweep speed multiplier
        """
        print("🔄 Starting scan sweep...")
        
        # Sweep pattern
        positions = [
            (45, 90), (90, 90), (135, 90),  # Middle sweep
            (135, 60), (90, 60), (45, 60),   # Upper sweep
            (45, 120), (90, 120), (135, 120), # Lower sweep
            (90, 90)  # Return to center
        ]
        
        for pan, tilt in positions:
            if not self.running:
                break
            self.set_position(pan, tilt, immediate=True)
            time.sleep(1.0 / speed)  # Increased delay
    
    def _update_pid(self, current, target, error_sum, last_error, dt=0.05):
        """
        PID controller update
        
        Returns:
            control_signal, error_sum, error
        """
        error = target - current
        error_sum += error * dt
        error_derivative = (error - last_error) / dt
        
        # Anti-windup for integral term
        error_sum = max(-50, min(50, error_sum))
        
        # PID formula
        control = (self.kp * error + 
                  self.ki * error_sum + 
                  self.kd * error_derivative)
        
        return control, error_sum, error
    
    def _control_loop(self):
        """Main control loop for smooth servo movement"""
        dt = 0.05  # 20Hz update rate (increased from 50Hz)
        
        while self.running:
            with self.lock:
                # PID control for pan
                pan_control, self.pan_error_sum, self.pan_last_error = self._update_pid(
                    self.current_pan, self.target_pan, 
                    self.pan_error_sum, self.pan_last_error, dt
                )
                
                # PID control for tilt
                tilt_control, self.tilt_error_sum, self.tilt_last_error = self._update_pid(
                    self.current_tilt, self.target_tilt,
                    self.tilt_error_sum, self.tilt_last_error, dt
                )
                
                # Apply smoothing
                new_pan = self.current_pan + pan_control * self.smoothing_factor
                new_tilt = self.current_tilt + tilt_control * self.smoothing_factor
                
                # Clamp to limits
                new_pan = self._clamp_angle(new_pan, self.pan_min, self.pan_max)
                new_tilt = self._clamp_angle(new_tilt, self.tilt_min, self.tilt_max)
                
                # Only update if movement is significant
                if abs(new_pan - self.current_pan) > self.movement_threshold:
                    self.current_pan = new_pan
                    calibrated_angle = self._apply_calibration(self.current_pan, is_pan=True)
                    duty = self._angle_to_duty(calibrated_angle)
                    self.pan_pwm.ChangeDutyCycle(duty)
                    # Less verbose logging in control loop
                    if abs(pan_control) > 5:  # Only log significant movements
                        print(f"🔄 Pan: {self.current_pan:.1f}° → {calibrated_angle:.1f}°")
                
                if abs(new_tilt - self.current_tilt) > self.movement_threshold:
                    self.current_tilt = new_tilt
                    calibrated_angle = self._apply_calibration(self.current_tilt, is_pan=False)
                    duty = self._angle_to_duty(calibrated_angle)
                    self.tilt_pwm.ChangeDutyCycle(duty)
                    # Less verbose logging in control loop
                    if abs(tilt_control) > 5:  # Only log significant movements
                        print(f"🔄 Tilt: {self.current_tilt:.1f}° → {calibrated_angle:.1f}°")
            
            time.sleep(dt)
    
    def start(self):
        """Start the servo control thread"""
        if not self.running:
            self.running = True
            self.thread = Thread(target=self._control_loop)
            self.thread.daemon = True
            self.thread.start()
            print("✅ Servo control thread started")
    
    def stop(self):
        """Stop the servo control thread"""
        self.running = False
        if self.thread:
            self.thread.join()
        print("⏹️  Servo control thread stopped")
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop()
        self.pan_pwm.stop()
        self.tilt_pwm.stop()
        GPIO.cleanup()
        print("🧹 GPIO cleaned up")
    
    def get_status(self):
        """Get current servo status"""
        return {
            'current_pan': round(self.current_pan, 1),
            'current_tilt': round(self.current_tilt, 1),
            'target_pan': round(self.target_pan, 1),
            'target_tilt': round(self.target_tilt, 1),
            'running': self.running
        }

# SIMPLE TEST FUNCTION
def simple_servo_test():
    """Simple servo test without PID complexity"""
    print("🔧 Simple Servo Test")
    print("===================")
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(16, GPIO.OUT)  # Pan
    GPIO.setup(24, GPIO.OUT)  # Tilt
    
    pan_pwm = GPIO.PWM(16, 50)
    tilt_pwm = GPIO.PWM(24, 50)
    
    pan_pwm.start(7.5)  # Center position
    tilt_pwm.start(7.5)
    
    try:
        print("Testing basic positions...")
        positions = [
            (2.5, "0 degrees"),
            (7.5, "90 degrees"),
            (12.5, "180 degrees"),
            (7.5, "Center")
        ]
        
        for duty, desc in positions:
            print(f"Setting to {desc} (duty: {duty}%)")
            pan_pwm.ChangeDutyCycle(duty)
            time.sleep(2)
            tilt_pwm.ChangeDutyCycle(duty)
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("Test interrupted")
    
    finally:
        pan_pwm.stop()
        tilt_pwm.stop()
        GPIO.cleanup()
        print("Cleanup complete")


class ManualControl:
    """Manual control interface for testing servos"""
    
    def __init__(self, servo_controller):
        self.servo = servo_controller
        self.step_size = 10  # Increased step size
        
    def keyboard_control(self):
        """Control servos with keyboard"""
        import sys
        
        print("\n🎮 Manual Servo Control")
        print("------------------------")
        print("W/S: Tilt up/down")
        print("A/D: Pan left/right") 
        print("C: Center position")
        print("T: Simple test")
        print("Q: Quit")
        print("------------------------\n")
        
        try:
            while True:
                key = input("Enter command: ").lower().strip()
                
                current = self.servo.get_status()
                pan = current['current_pan']
                tilt = current['current_tilt']
                
                if key == 'w':  # Tilt up
                    new_tilt = max(self.servo.tilt_min, tilt - self.step_size)
                    self.servo.set_position(tilt=new_tilt, immediate=True)
                    print(f"↑ Tilt: {new_tilt}°")
                    
                elif key == 's':  # Tilt down
                    new_tilt = min(self.servo.tilt_max, tilt + self.step_size)
                    self.servo.set_position(tilt=new_tilt, immediate=True)
                    print(f"↓ Tilt: {new_tilt}°")
                    
                elif key == 'a':  # Pan left
                    new_pan = max(self.servo.pan_min, pan - self.step_size)
                    self.servo.set_position(pan=new_pan, immediate=True)
                    print(f"← Pan: {new_pan}°")
                    
                elif key == 'd':  # Pan right
                    new_pan = min(self.servo.pan_max, pan + self.step_size)
                    self.servo.set_position(pan=new_pan, immediate=True)
                    print(f"→ Pan: {new_pan}°")
                
                elif key == 'c':  # Center
                    self.servo.center_position()
                    print("🎯 Centered")
                
                elif key == 't':  # Simple test
                    print("Running simple test...")
                    self.servo.stop()
                    self.servo.cleanup()
                    simple_servo_test()
                    return
                
                elif key == 'q':  # Quit
                    print("\n👋 Exiting manual control")
                    break
                
        except KeyboardInterrupt:
            print("\n⏹️  Manual control interrupted")


# CALIBRATION TEST FUNCTION
def calibration_test():
    """Interactive calibration test to find correct servo parameters"""
    print("🔧 Servo Calibration Test")
    print("========================")
    
    # Ask user for calibration parameters
    print("\nFirst, let's test different duty cycle ranges...")
    print("Standard ranges:")
    print("1. Conservative: 5-10% (1ms-2ms pulse)")
    print("2. Extended: 2.5-12.5% (0.5ms-2.5ms pulse)")
    print("3. Custom range")
    
    range_choice = input("Choose range (1, 2, or 3): ")
    
    if range_choice == "1":
        min_duty, max_duty = 5.0, 10.0
    elif range_choice == "2":
        min_duty, max_duty = 2.5, 12.5
    else:
        min_duty = float(input("Enter minimum duty cycle (%): "))
        max_duty = float(input("Enter maximum duty cycle (%): "))
    
    # Test specific servo
    servo_choice = input("\nTest pan (p) or tilt (t) servo? ").lower()
    pin = 16 if servo_choice == 'p' else 24
    servo_name = "Pan" if servo_choice == 'p' else "Tilt"
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)
    
    def angle_to_duty(angle):
        return min_duty + (angle / 180) * (max_duty - min_duty)
    
    try:
        print(f"\nTesting {servo_name} servo on GPIO {pin}")
        print(f"Duty cycle range: {min_duty}% - {max_duty}%")
        print("\nCommands:")
        print("0-180: Set angle")
        print("r: Reverse direction test")
        print("c: Center (90°)")
        print("q: Quit")
        
        pwm.start(angle_to_duty(90))  # Start at center
        current_angle = 90
        reversed_mode = False
        
        while True:
            cmd = input(f"\n{servo_name} at {current_angle}° - Command: ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'c':
                current_angle = 90
                actual_angle = 180 - current_angle if reversed_mode else current_angle
                duty = angle_to_duty(actual_angle)
                pwm.ChangeDutyCycle(duty)
                print(f"Center: {current_angle}° (duty: {duty:.1f}%)")
            elif cmd == 'r':
                reversed_mode = not reversed_mode
                actual_angle = 180 - current_angle if reversed_mode else current_angle
                duty = angle_to_duty(actual_angle)
                pwm.ChangeDutyCycle(duty)
                print(f"Reversed: {'ON' if reversed_mode else 'OFF'}")
                print(f"Angle: {current_angle}° → {actual_angle}° (duty: {duty:.1f}%)")
            else:
                try:
                    angle = float(cmd)
                    if 0 <= angle <= 180:
                        current_angle = angle
                        actual_angle = 180 - current_angle if reversed_mode else current_angle
                        duty = angle_to_duty(actual_angle)
                        pwm.ChangeDutyCycle(duty)
                        print(f"Set to: {current_angle}° → {actual_angle}° (duty: {duty:.1f}%)")
                    else:
                        print("Angle must be 0-180")
                except ValueError:
                    print("Invalid command")
        
        # Final calibration summary
        print(f"\n📋 Calibration Summary for {servo_name} servo:")
        print(f"   GPIO pin: {pin}")
        print(f"   Duty cycle range: {min_duty}% - {max_duty}%")
        print(f"   Reversed: {reversed_mode}")
        print(f"   Use these parameters in your servo controller!")
        
    finally:
        pwm.stop()
        GPIO.cleanup()


# Test script
if __name__ == "__main__":
    print("🔧 Testing Servo Controller...")
    print("Using GPIO pins 16 (pan) and 24 (tilt)")
    
    # Ask user which test to run
    print("\nSelect test:")
    print("1. Full servo controller test")
    print("2. Simple servo test")
    print("3. Calibration test (RECOMMENDED)")
    choice = input("Enter choice (1, 2, or 3): ")
    
    if choice == '2':
        simple_servo_test()
    elif choice == '3':
        calibration_test()
    else:
        # Ask for calibration parameters
        print("\nServo Calibration:")
        pan_reversed = input("Is pan servo reversed? (y/n): ").lower() == 'y'
        tilt_reversed = input("Is tilt servo reversed? (y/n): ").lower() == 'y'
        
        try:
            pan_offset = float(input("Pan offset degrees (-90 to +90, 0 for none): ") or "0")
            tilt_offset = float(input("Tilt offset degrees (-90 to +90, 0 for none): ") or "0")
        except ValueError:
            pan_offset = tilt_offset = 0
            
        try:
            # Initialize servo controller with calibration
            servo = ServoController(pan_pin=16, tilt_pin=24, 
                                  pan_reversed=pan_reversed, tilt_reversed=tilt_reversed,
                                  pan_offset=pan_offset, tilt_offset=tilt_offset)
            
            # Manual control
            manual = ManualControl(servo)
            manual.keyboard_control()
            
        except Exception as e:
            print(f"❌ Error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            servo.cleanup()
            print("✅ Test complete")#!/usr/bin/env python3
"""
Servo Control Module for Pan-Tilt Mechanism - FIXED VERSION
Handles smooth servo movement with PID control
"""

import RPi.GPIO as GPIO
import time
import numpy as np
from threading import Thread, Lock
from collections import deque

class ServoController:
    def __init__(self, pan_pin=16, tilt_pin=24, pan_reversed=False, tilt_reversed=False, 
                 pan_offset=0, tilt_offset=0):  # Updated to your pins
        """
        Initialize servo controller
        
        Args:
            pan_pin: GPIO pin for pan servo (default 16)
            tilt_pin: GPIO pin for tilt servo (default 24)
            pan_reversed: Reverse pan servo direction
            tilt_reversed: Reverse tilt servo direction
            pan_offset: Offset for pan servo center position (-90 to +90)
            tilt_offset: Offset for tilt servo center position (-90 to +90)
        """
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Servo pins
        self.pan_pin = pan_pin
        self.tilt_pin = tilt_pin
        
        # Servo calibration
        self.pan_reversed = pan_reversed
        self.tilt_reversed = tilt_reversed
        self.pan_offset = pan_offset  # Offset from mechanical center
        self.tilt_offset = tilt_offset
        
        # Setup PWM (50Hz for servos)
        GPIO.setup(self.pan_pin, GPIO.OUT)
        GPIO.setup(self.tilt_pin, GPIO.OUT)
        self.pan_pwm = GPIO.PWM(self.pan_pin, 50)
        self.tilt_pwm = GPIO.PWM(self.tilt_pin, 50)
        
        # Servo limits (in degrees)
        self.pan_min = 0
        self.pan_max = 180
        self.tilt_min = 30  # Don't look too far down
        self.tilt_max = 150  # Don't look too far up
        
        # Current positions
        self.current_pan = 90  # Center
        self.current_tilt = 90  # Center
        
        # Target positions
        self.target_pan = 90
        self.target_tilt = 90
        
        # PID parameters - MUCH MORE CONSERVATIVE for stability
        self.kp = 0.3  # Reduced proportional gain
        self.ki = 0.01  # Much reduced integral gain
        self.kd = 0.05  # Reduced derivative gain
        
        # PID state
        self.pan_error_sum = 0
        self.pan_last_error = 0
        self.tilt_error_sum = 0
        self.tilt_last_error = 0
        
        # Smoothing - MUCH MORE CONSERVATIVE
        self.smoothing_factor = 0.1  # Much slower movement
        self.movement_threshold = 0.1  # Smaller threshold
        
        # Thread control
        self.running = False
        self.thread = None
        self.lock = Lock()
        
        # Movement history for smoothing
        self.pan_history = deque(maxlen=3)  # Reduced history
        self.tilt_history = deque(maxlen=3)
        
        # Start servos at center position with DEBUGGING
        center_duty = self._angle_to_duty(90)
        print(f"🔧 Starting PWM with duty cycle: {center_duty}")
        
        self.pan_pwm.start(center_duty)
        self.tilt_pwm.start(center_duty)
        
        # FORCE initial movement to test
        time.sleep(0.5)
        self._force_test_movement()
        
        print("✅ Servo controller initialized")
        print(f"   Pan servo on GPIO {pan_pin}")
        print(f"   Tilt servo on GPIO {tilt_pin}")
        print(f"   PWM frequency: 50Hz")
        print(f"   Center duty cycle: {center_duty}%")
        
    def _angle_to_duty(self, angle):
        """Convert angle (0-180) to duty cycle with calibration"""
        # Standard servo: 1ms-2ms pulse width = 5%-10% duty cycle
        # Many servos actually need: 1ms-2ms = 5%-10% (NOT 2.5%-12.5%)
        # Some servos are reversed or have different ranges
        
        # CALIBRATION: Adjust these values based on your servo behavior
        min_duty = 5.0   # Duty cycle for 0 degrees
        max_duty = 10.0  # Duty cycle for 180 degrees
        
        return min_duty + (angle / 180) * (max_duty - min_duty)
    
    def _clamp_angle(self, angle, min_angle, max_angle):
        """Clamp angle to valid range"""
        return max(min_angle, min(max_angle, angle))
        """Force test movement to verify servo connection"""
        print("🧪 Testing servo movement...")
        
        # Test pan servo
        print("   Testing pan servo (GPIO {})".format(self.pan_pin))
        for angle in [0, 45, 90, 135, 180, 90]:  # More comprehensive test
            calibrated = self._apply_calibration(angle, is_pan=True)
            duty = self._angle_to_duty(calibrated)
            print(f"      Setting pan to {angle}° → {calibrated}° (duty: {duty:.1f}%)")
            self.pan_pwm.ChangeDutyCycle(duty)
            time.sleep(1.5)  # Longer delay
        
        # Test tilt servo
        print("   Testing tilt servo (GPIO {})".format(self.tilt_pin))
        for angle in [30, 60, 90, 120, 150, 90]:  # Within tilt limits
            calibrated = self._apply_calibration(angle, is_pan=False)
            duty = self._angle_to_duty(calibrated)
            print(f"      Setting tilt to {angle}° → {calibrated}° (duty: {duty:.1f}%)")
            self.tilt_pwm.ChangeDutyCycle(duty)
            time.sleep(1.5)  # Longer delay
        
        print("   Test complete - did servos move?")
    
    def _apply_calibration(self, angle, is_pan=True):
        """Apply calibration (offset and reversal) to angle"""
        if is_pan:
            # Apply offset
            calibrated_angle = angle + self.pan_offset
            # Apply reversal
            if self.pan_reversed:
                calibrated_angle = 180 - calibrated_angle
        else:
            # Apply offset  
            calibrated_angle = angle + self.tilt_offset
            # Apply reversal
            if self.tilt_reversed:
                calibrated_angle = 180 - calibrated_angle
                
        # Clamp to valid range
        return max(0, min(180, calibrated_angle))
    
    def set_position(self, pan=None, tilt=None, immediate=False):
        """
        Set target position for servos
        
        Args:
            pan: Pan angle (0-180) or None to keep current
            tilt: Tilt angle (0-180) or None to keep current
            immediate: If True, move immediately without smoothing
        """
        with self.lock:
            if pan is not None:
                self.target_pan = self._clamp_angle(pan, self.pan_min, self.pan_max)
                print(f"🎯 Pan target set to {self.target_pan}°")
    def _force_test_movement(self):
            
            if tilt is not None:
                self.target_tilt = self._clamp_angle(tilt, self.tilt_min, self.tilt_max)
                print(f"🎯 Tilt target set to {self.target_tilt}°")
                if immediate:
                    self.current_tilt = self.target_tilt
                    duty = self._angle_to_duty(self.current_tilt)
                    self.tilt_pwm.ChangeDutyCycle(duty)
                    print(f"   Tilt duty cycle: {duty:.1f}%")
    
    def track_coordinate(self, x, y, frame_width, frame_height):
        """
        Track a coordinate in the camera frame
        
        Args:
            x, y: Pixel coordinates of target
            frame_width, frame_height: Frame dimensions
        """
        # Convert pixel coordinates to servo angles
        # Map x (0 to frame_width) to pan angle
        # Invert because servo moves opposite to screen coordinates
        pan_angle = 180 - (x / frame_width) * 180
        
        # Map y (0 to frame_height) to tilt angle
        tilt_angle = 180 - (y / frame_height) * 180
        
        # Apply smoothing
        self.pan_history.append(pan_angle)
        self.tilt_history.append(tilt_angle)
        
        if len(self.pan_history) > 1:
            # Use median for noise reduction
            smooth_pan = np.median(self.pan_history)
            smooth_tilt = np.median(self.tilt_history)
            self.set_position(smooth_pan, smooth_tilt)
        else:
            self.set_position(pan_angle, tilt_angle)
    
    def center_position(self):
        """Return servos to center position"""
        self.set_position(90, 90, immediate=True)
        print("🎯 Servos centered")
    
    def sweep_scan(self, speed=1):
        """
        Perform a scanning sweep motion
        
        Args:
            speed: Sweep speed multiplier
        """
        print("🔄 Starting scan sweep...")
        
        # Sweep pattern
        positions = [
            (45, 90), (90, 90), (135, 90),  # Middle sweep
            (135, 60), (90, 60), (45, 60),   # Upper sweep
            (45, 120), (90, 120), (135, 120), # Lower sweep
            (90, 90)  # Return to center
        ]
        
        for pan, tilt in positions:
            if not self.running:
                break
            self.set_position(pan, tilt, immediate=True)
            time.sleep(1.0 / speed)  # Increased delay
    
    def _update_pid(self, current, target, error_sum, last_error, dt=0.05):
        """
        PID controller update
        
        Returns:
            control_signal, error_sum, error
        """
        error = target - current
        error_sum += error * dt
        error_derivative = (error - last_error) / dt
        
        # Anti-windup for integral term
        error_sum = max(-50, min(50, error_sum))
        
        # PID formula
        control = (self.kp * error + 
                  self.ki * error_sum + 
                  self.kd * error_derivative)
        
        return control, error_sum, error
    
    def _control_loop(self):
        """Main control loop for smooth servo movement"""
        dt = 0.05  # 20Hz update rate (increased from 50Hz)
        
        while self.running:
            with self.lock:
                # PID control for pan
                pan_control, self.pan_error_sum, self.pan_last_error = self._update_pid(
                    self.current_pan, self.target_pan, 
                    self.pan_error_sum, self.pan_last_error, dt
                )
                
                # PID control for tilt
                tilt_control, self.tilt_error_sum, self.tilt_last_error = self._update_pid(
                    self.current_tilt, self.target_tilt,
                    self.tilt_error_sum, self.tilt_last_error, dt
                )
                
                # Apply smoothing
                new_pan = self.current_pan + pan_control * self.smoothing_factor
                new_tilt = self.current_tilt + tilt_control * self.smoothing_factor
                
                # Clamp to limits
                new_pan = self._clamp_angle(new_pan, self.pan_min, self.pan_max)
                new_tilt = self._clamp_angle(new_tilt, self.tilt_min, self.tilt_max)
                
                # Only update if movement is significant
                if abs(new_pan - self.current_pan) > self.movement_threshold:
                    self.current_pan = new_pan
                    calibrated_angle = self._apply_calibration(self.current_pan, is_pan=True)
                    duty = self._angle_to_duty(calibrated_angle)
                    self.pan_pwm.ChangeDutyCycle(duty)
                    # Less verbose logging in control loop
                    if abs(pan_control) > 5:  # Only log significant movements
                        print(f"🔄 Pan: {self.current_pan:.1f}° → {calibrated_angle:.1f}°")
                
                if abs(new_tilt - self.current_tilt) > self.movement_threshold:
                    self.current_tilt = new_tilt
                    calibrated_angle = self._apply_calibration(self.current_tilt, is_pan=False)
                    duty = self._angle_to_duty(calibrated_angle)
                    self.tilt_pwm.ChangeDutyCycle(duty)
                    # Less verbose logging in control loop
                    if abs(tilt_control) > 5:  # Only log significant movements
                        print(f"🔄 Tilt: {self.current_tilt:.1f}° → {calibrated_angle:.1f}°")
            
            time.sleep(dt)
    
    def start(self):
        """Start the servo control thread"""
        if not self.running:
            self.running = True
            self.thread = Thread(target=self._control_loop)
            self.thread.daemon = True
            self.thread.start()
            print("✅ Servo control thread started")
    
    def stop(self):
        """Stop the servo control thread"""
        self.running = False
        if self.thread:
            self.thread.join()
        print("⏹️  Servo control thread stopped")
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop()
        self.pan_pwm.stop()
        self.tilt_pwm.stop()
        GPIO.cleanup()
        print("🧹 GPIO cleaned up")
    
    def get_status(self):
        """Get current servo status"""
        return {
            'current_pan': round(self.current_pan, 1),
            'current_tilt': round(self.current_tilt, 1),
            'target_pan': round(self.target_pan, 1),
            'target_tilt': round(self.target_tilt, 1),
            'running': self.running
        }

# SIMPLE TEST FUNCTION
def simple_servo_test():
    """Simple servo test without PID complexity"""
    print("🔧 Simple Servo Test")
    print("===================")
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(16, GPIO.OUT)  # Pan
    GPIO.setup(24, GPIO.OUT)  # Tilt
    
    pan_pwm = GPIO.PWM(16, 50)
    tilt_pwm = GPIO.PWM(24, 50)
    
    pan_pwm.start(7.5)  # Center position
    tilt_pwm.start(7.5)
    
    try:
        print("Testing basic positions...")
        positions = [
            (2.5, "0 degrees"),
            (7.5, "90 degrees"),
            (12.5, "180 degrees"),
            (7.5, "Center")
        ]
        
        for duty, desc in positions:
            print(f"Setting to {desc} (duty: {duty}%)")
            pan_pwm.ChangeDutyCycle(duty)
            time.sleep(2)
            tilt_pwm.ChangeDutyCycle(duty)
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("Test interrupted")
    
    finally:
        pan_pwm.stop()
        tilt_pwm.stop()
        GPIO.cleanup()
        print("Cleanup complete")


class ManualControl:
    """Manual control interface for testing servos"""
    
    def __init__(self, servo_controller):
        self.servo = servo_controller
        self.step_size = 10  # Increased step size
        
    def keyboard_control(self):
        """Control servos with keyboard"""
        import sys
        
        print("\n🎮 Manual Servo Control")
        print("------------------------")
        print("W/S: Tilt up/down")
        print("A/D: Pan left/right") 
        print("C: Center position")
        print("T: Simple test")
        print("Q: Quit")
        print("------------------------\n")
        
        try:
            while True:
                key = input("Enter command: ").lower().strip()
                
                current = self.servo.get_status()
                pan = current['current_pan']
                tilt = current['current_tilt']
                
                if key == 'w':  # Tilt up
                    new_tilt = max(self.servo.tilt_min, tilt - self.step_size)
                    self.servo.set_position(tilt=new_tilt, immediate=True)
                    print(f"↑ Tilt: {new_tilt}°")
                    
                elif key == 's':  # Tilt down
                    new_tilt = min(self.servo.tilt_max, tilt + self.step_size)
                    self.servo.set_position(tilt=new_tilt, immediate=True)
                    print(f"↓ Tilt: {new_tilt}°")
                    
                elif key == 'a':  # Pan left
                    new_pan = max(self.servo.pan_min, pan - self.step_size)
                    self.servo.set_position(pan=new_pan, immediate=True)
                    print(f"← Pan: {new_pan}°")
                    
                elif key == 'd':  # Pan right
                    new_pan = min(self.servo.pan_max, pan + self.step_size)
                    self.servo.set_position(pan=new_pan, immediate=True)
                    print(f"→ Pan: {new_pan}°")
                
                elif key == 'c':  # Center
                    self.servo.center_position()
                    print("🎯 Centered")
                
                elif key == 't':  # Simple test
                    print("Running simple test...")
                    self.servo.stop()
                    self.servo.cleanup()
                    simple_servo_test()
                    return
                
                elif key == 'q':  # Quit
                    print("\n👋 Exiting manual control")
                    break
                
        except KeyboardInterrupt:
            print("\n⏹️  Manual control interrupted")


# CALIBRATION TEST FUNCTION
def calibration_test():
    """Interactive calibration test to find correct servo parameters"""
    print("🔧 Servo Calibration Test")
    print("========================")
    
    # Ask user for calibration parameters
    print("\nFirst, let's test different duty cycle ranges...")
    print("Standard ranges:")
    print("1. Conservative: 5-10% (1ms-2ms pulse)")
    print("2. Extended: 2.5-12.5% (0.5ms-2.5ms pulse)")
    print("3. Custom range")
    
    range_choice = input("Choose range (1, 2, or 3): ")
    
    if range_choice == "1":
        min_duty, max_duty = 5.0, 10.0
    elif range_choice == "2":
        min_duty, max_duty = 2.5, 12.5
    else:
        min_duty = float(input("Enter minimum duty cycle (%): "))
        max_duty = float(input("Enter maximum duty cycle (%): "))
    
    # Test specific servo
    servo_choice = input("\nTest pan (p) or tilt (t) servo? ").lower()
    pin = 16 if servo_choice == 'p' else 24
    servo_name = "Pan" if servo_choice == 'p' else "Tilt"
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)
    
    def angle_to_duty(angle):
        return min_duty + (angle / 180) * (max_duty - min_duty)
    
    try:
        print(f"\nTesting {servo_name} servo on GPIO {pin}")
        print(f"Duty cycle range: {min_duty}% - {max_duty}%")
        print("\nCommands:")
        print("0-180: Set angle")
        print("r: Reverse direction test")
        print("c: Center (90°)")
        print("q: Quit")
        
        pwm.start(angle_to_duty(90))  # Start at center
        current_angle = 90
        reversed_mode = False
        
        while True:
            cmd = input(f"\n{servo_name} at {current_angle}° - Command: ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'c':
                current_angle = 90
                actual_angle = 180 - current_angle if reversed_mode else current_angle
                duty = angle_to_duty(actual_angle)
                pwm.ChangeDutyCycle(duty)
                print(f"Center: {current_angle}° (duty: {duty:.1f}%)")
            elif cmd == 'r':
                reversed_mode = not reversed_mode
                actual_angle = 180 - current_angle if reversed_mode else current_angle
                duty = angle_to_duty(actual_angle)
                pwm.ChangeDutyCycle(duty)
                print(f"Reversed: {'ON' if reversed_mode else 'OFF'}")
                print(f"Angle: {current_angle}° → {actual_angle}° (duty: {duty:.1f}%)")
            else:
                try:
                    angle = float(cmd)
                    if 0 <= angle <= 180:
                        current_angle = angle
                        actual_angle = 180 - current_angle if reversed_mode else current_angle
                        duty = angle_to_duty(actual_angle)
                        pwm.ChangeDutyCycle(duty)
                        print(f"Set to: {current_angle}° → {actual_angle}° (duty: {duty:.1f}%)")
                    else:
                        print("Angle must be 0-180")
                except ValueError:
                    print("Invalid command")
        
        # Final calibration summary
        print(f"\n📋 Calibration Summary for {servo_name} servo:")
        print(f"   GPIO pin: {pin}")
        print(f"   Duty cycle range: {min_duty}% - {max_duty}%")
        print(f"   Reversed: {reversed_mode}")
        print(f"   Use these parameters in your servo controller!")
        
    finally:
        pwm.stop()
        GPIO.cleanup()


# Test script
if __name__ == "__main__":
    print("🔧 Testing Servo Controller...")
    print("Using GPIO pins 16 (pan) and 24 (tilt)")
    
    # Ask user which test to run
    print("\nSelect test:")
    print("1. Full servo controller test")
    print("2. Simple servo test")
    print("3. Calibration test (RECOMMENDED)")
    choice = input("Enter choice (1, 2, or 3): ")
    
    if choice == '2':
        simple_servo_test()
    elif choice == '3':
        calibration_test()
    else:
        # Ask for calibration parameters
        print("\nServo Calibration:")
        pan_reversed = input("Is pan servo reversed? (y/n): ").lower() == 'y'
        tilt_reversed = input("Is tilt servo reversed? (y/n): ").lower() == 'y'
        
        try:
            pan_offset = float(input("Pan offset degrees (-90 to +90, 0 for none): ") or "0")
            tilt_offset = float(input("Tilt offset degrees (-90 to +90, 0 for none): ") or "0")
        except ValueError:
            pan_offset = tilt_offset = 0
            
        try:
            # Initialize servo controller with calibration
            servo = ServoController(pan_pin=16, tilt_pin=24, 
                                  pan_reversed=pan_reversed, tilt_reversed=tilt_reversed,
                                  pan_offset=pan_offset, tilt_offset=tilt_offset)
            
            # Manual control
            manual = ManualControl(servo)
            manual.keyboard_control()
            
        except Exception as e:
            print(f"❌ Error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            servo.cleanup()
            print("✅ Test complete")#!/usr/bin/env python3
"""
Servo Control Module for Pan-Tilt Mechanism - FIXED VERSION
Handles smooth servo movement with PID control
"""

import RPi.GPIO as GPIO
import time
import numpy as np
from threading import Thread, Lock
from collections import deque

class ServoController:
    def __init__(self, pan_pin=16, tilt_pin=24, pan_reversed=False, tilt_reversed=False, 
                 pan_offset=0, tilt_offset=0):  # Updated to your pins
        """
        Initialize servo controller
        
        Args:
            pan_pin: GPIO pin for pan servo (default 16)
            tilt_pin: GPIO pin for tilt servo (default 24)
            pan_reversed: Reverse pan servo direction
            tilt_reversed: Reverse tilt servo direction
            pan_offset: Offset for pan servo center position (-90 to +90)
            tilt_offset: Offset for tilt servo center position (-90 to +90)
        """
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Servo pins
        self.pan_pin = pan_pin
        self.tilt_pin = tilt_pin
        
        # Servo calibration
        self.pan_reversed = pan_reversed
        self.tilt_reversed = tilt_reversed
        self.pan_offset = pan_offset  # Offset from mechanical center
        self.tilt_offset = tilt_offset
        
        # Setup PWM (50Hz for servos)
        GPIO.setup(self.pan_pin, GPIO.OUT)
        GPIO.setup(self.tilt_pin, GPIO.OUT)
        self.pan_pwm = GPIO.PWM(self.pan_pin, 50)
        self.tilt_pwm = GPIO.PWM(self.tilt_pin, 50)
        
        # Servo limits (in degrees)
        self.pan_min = 0
        self.pan_max = 180
        self.tilt_min = 30  # Don't look too far down
        self.tilt_max = 150  # Don't look too far up
        
        # Current positions
        self.current_pan = 90  # Center
        self.current_tilt = 90  # Center
        
        # Target positions
        self.target_pan = 90
        self.target_tilt = 90
        
        # PID parameters - MUCH MORE CONSERVATIVE for stability
        self.kp = 0.3  # Reduced proportional gain
        self.ki = 0.01  # Much reduced integral gain
        self.kd = 0.05  # Reduced derivative gain
        
        # PID state
        self.pan_error_sum = 0
        self.pan_last_error = 0
        self.tilt_error_sum = 0
        self.tilt_last_error = 0
        
        # Smoothing - MUCH MORE CONSERVATIVE
        self.smoothing_factor = 0.1  # Much slower movement
        self.movement_threshold = 0.1  # Smaller threshold
        
        # Thread control
        self.running = False
        self.thread = None
        self.lock = Lock()
        
        # Movement history for smoothing
        self.pan_history = deque(maxlen=3)  # Reduced history
        self.tilt_history = deque(maxlen=3)
        
        # Start servos at center position with DEBUGGING
        center_duty = self._angle_to_duty(90)
        print(f"🔧 Starting PWM with duty cycle: {center_duty}")
        
        self.pan_pwm.start(center_duty)
        self.tilt_pwm.start(center_duty)
        
        # FORCE initial movement to test
        time.sleep(0.5)
        self._force_test_movement()
        
        print("✅ Servo controller initialized")
        print(f"   Pan servo on GPIO {pan_pin}")
        print(f"   Tilt servo on GPIO {tilt_pin}")
        print(f"   PWM frequency: 50Hz")
        print(f"   Center duty cycle: {center_duty}%")
        
    def _angle_to_duty(self, angle):
        """Convert angle (0-180) to duty cycle with calibration"""
        # Standard servo: 1ms-2ms pulse width = 5%-10% duty cycle
        # Many servos actually need: 1ms-2ms = 5%-10% (NOT 2.5%-12.5%)
        # Some servos are reversed or have different ranges
        
        # CALIBRATION: Adjust these values based on your servo behavior
        min_duty = 5.0   # Duty cycle for 0 degrees
        max_duty = 10.0  # Duty cycle for 180 degrees
        
        return min_duty + (angle / 180) * (max_duty - min_duty)
    
    def _clamp_angle(self, angle, min_angle, max_angle):
        """Clamp angle to valid range"""
        return max(min_angle, min(max_angle, angle))
        """Force test movement to verify servo connection"""
        print("🧪 Testing servo movement...")
        
        # Test pan servo
        print("   Testing pan servo (GPIO {})".format(self.pan_pin))
        for angle in [0, 45, 90, 135, 180, 90]:  # More comprehensive test
            calibrated = self._apply_calibration(angle, is_pan=True)
            duty = self._angle_to_duty(calibrated)
            print(f"      Setting pan to {angle}° → {calibrated}° (duty: {duty:.1f}%)")
            self.pan_pwm.ChangeDutyCycle(duty)
            time.sleep(1.5)  # Longer delay
        
        # Test tilt servo
        print("   Testing tilt servo (GPIO {})".format(self.tilt_pin))
        for angle in [30, 60, 90, 120, 150, 90]:  # Within tilt limits
            calibrated = self._apply_calibration(angle, is_pan=False)
            duty = self._angle_to_duty(calibrated)
            print(f"      Setting tilt to {angle}° → {calibrated}° (duty: {duty:.1f}%)")
            self.tilt_pwm.ChangeDutyCycle(duty)
            time.sleep(1.5)  # Longer delay
        
        print("   Test complete - did servos move?")
    
    def _apply_calibration(self, angle, is_pan=True):
        """Apply calibration (offset and reversal) to angle"""
        if is_pan:
            # Apply offset
            calibrated_angle = angle + self.pan_offset
            # Apply reversal
            if self.pan_reversed:
                calibrated_angle = 180 - calibrated_angle
        else:
            # Apply offset  
            calibrated_angle = angle + self.tilt_offset
            # Apply reversal
            if self.tilt_reversed:
                calibrated_angle = 180 - calibrated_angle
                
        # Clamp to valid range
        return max(0, min(180, calibrated_angle))
    
    def set_position(self, pan=None, tilt=None, immediate=False):
        """
        Set target position for servos
        
        Args:
            pan: Pan angle (0-180) or None to keep current
            tilt: Tilt angle (0-180) or None to keep current
            immediate: If True, move immediately without smoothing
        """
        with self.lock:
            if pan is not None:
                self.target_pan = self._clamp_angle(pan, self.pan_min, self.pan_max)
                print(f"🎯 Pan target set to {self.target_pan}°")
    def _force_test_movement(self):
            
            if tilt is not None:
                self.target_tilt = self._clamp_angle(tilt, self.tilt_min, self.tilt_max)
                print(f"🎯 Tilt target set to {self.target_tilt}°")
                if immediate:
                    self.current_tilt = self.target_tilt
                    duty = self._angle_to_duty(self.current_tilt)
                    self.tilt_pwm.ChangeDutyCycle(duty)
                    print(f"   Tilt duty cycle: {duty:.1f}%")
    
    def track_coordinate(self, x, y, frame_width, frame_height):
        """
        Track a coordinate in the camera frame
        
        Args:
            x, y: Pixel coordinates of target
            frame_width, frame_height: Frame dimensions
        """
        # Convert pixel coordinates to servo angles
        # Map x (0 to frame_width) to pan angle
        # Invert because servo moves opposite to screen coordinates
        pan_angle = 180 - (x / frame_width) * 180
        
        # Map y (0 to frame_height) to tilt angle
        tilt_angle = 180 - (y / frame_height) * 180
        
        # Apply smoothing
        self.pan_history.append(pan_angle)
        self.tilt_history.append(tilt_angle)
        
        if len(self.pan_history) > 1:
            # Use median for noise reduction
            smooth_pan = np.median(self.pan_history)
            smooth_tilt = np.median(self.tilt_history)
            self.set_position(smooth_pan, smooth_tilt)
        else:
            self.set_position(pan_angle, tilt_angle)
    
    def center_position(self):
        """Return servos to center position"""
        self.set_position(90, 90, immediate=True)
        print("🎯 Servos centered")
    
    def sweep_scan(self, speed=1):
        """
        Perform a scanning sweep motion
        
        Args:
            speed: Sweep speed multiplier
        """
        print("🔄 Starting scan sweep...")
        
        # Sweep pattern
        positions = [
            (45, 90), (90, 90), (135, 90),  # Middle sweep
            (135, 60), (90, 60), (45, 60),   # Upper sweep
            (45, 120), (90, 120), (135, 120), # Lower sweep
            (90, 90)  # Return to center
        ]
        
        for pan, tilt in positions:
            if not self.running:
                break
            self.set_position(pan, tilt, immediate=True)
            time.sleep(1.0 / speed)  # Increased delay
    
    def _update_pid(self, current, target, error_sum, last_error, dt=0.05):
        """
        PID controller update
        
        Returns:
            control_signal, error_sum, error
        """
        error = target - current
        error_sum += error * dt
        error_derivative = (error - last_error) / dt
        
        # Anti-windup for integral term
        error_sum = max(-50, min(50, error_sum))
        
        # PID formula
        control = (self.kp * error + 
                  self.ki * error_sum + 
                  self.kd * error_derivative)
        
        return control, error_sum, error
    
    def _control_loop(self):
        """Main control loop for smooth servo movement"""
        dt = 0.05  # 20Hz update rate (increased from 50Hz)
        
        while self.running:
            with self.lock:
                # PID control for pan
                pan_control, self.pan_error_sum, self.pan_last_error = self._update_pid(
                    self.current_pan, self.target_pan, 
                    self.pan_error_sum, self.pan_last_error, dt
                )
                
                # PID control for tilt
                tilt_control, self.tilt_error_sum, self.tilt_last_error = self._update_pid(
                    self.current_tilt, self.target_tilt,
                    self.tilt_error_sum, self.tilt_last_error, dt
                )
                
                # Apply smoothing
                new_pan = self.current_pan + pan_control * self.smoothing_factor
                new_tilt = self.current_tilt + tilt_control * self.smoothing_factor
                
                # Clamp to limits
                new_pan = self._clamp_angle(new_pan, self.pan_min, self.pan_max)
                new_tilt = self._clamp_angle(new_tilt, self.tilt_min, self.tilt_max)
                
                # Only update if movement is significant
                if abs(new_pan - self.current_pan) > self.movement_threshold:
                    self.current_pan = new_pan
                    calibrated_angle = self._apply_calibration(self.current_pan, is_pan=True)
                    duty = self._angle_to_duty(calibrated_angle)
                    self.pan_pwm.ChangeDutyCycle(duty)
                    # Less verbose logging in control loop
                    if abs(pan_control) > 5:  # Only log significant movements
                        print(f"🔄 Pan: {self.current_pan:.1f}° → {calibrated_angle:.1f}°")
                
                if abs(new_tilt - self.current_tilt) > self.movement_threshold:
                    self.current_tilt = new_tilt
                    calibrated_angle = self._apply_calibration(self.current_tilt, is_pan=False)
                    duty = self._angle_to_duty(calibrated_angle)
                    self.tilt_pwm.ChangeDutyCycle(duty)
                    # Less verbose logging in control loop
                    if abs(tilt_control) > 5:  # Only log significant movements
                        print(f"🔄 Tilt: {self.current_tilt:.1f}° → {calibrated_angle:.1f}°")
            
            time.sleep(dt)
    
    def start(self):
        """Start the servo control thread"""
        if not self.running:
            self.running = True
            self.thread = Thread(target=self._control_loop)
            self.thread.daemon = True
            self.thread.start()
            print("✅ Servo control thread started")
    
    def stop(self):
        """Stop the servo control thread"""
        self.running = False
        if self.thread:
            self.thread.join()
        print("⏹️  Servo control thread stopped")
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop()
        self.pan_pwm.stop()
        self.tilt_pwm.stop()
        GPIO.cleanup()
        print("🧹 GPIO cleaned up")
    
    def get_status(self):
        """Get current servo status"""
        return {
            'current_pan': round(self.current_pan, 1),
            'current_tilt': round(self.current_tilt, 1),
            'target_pan': round(self.target_pan, 1),
            'target_tilt': round(self.target_tilt, 1),
            'running': self.running
        }

# SIMPLE TEST FUNCTION
def simple_servo_test():
    """Simple servo test without PID complexity"""
    print("🔧 Simple Servo Test")
    print("===================")
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(16, GPIO.OUT)  # Pan
    GPIO.setup(24, GPIO.OUT)  # Tilt
    
    pan_pwm = GPIO.PWM(16, 50)
    tilt_pwm = GPIO.PWM(24, 50)
    
    pan_pwm.start(7.5)  # Center position
    tilt_pwm.start(7.5)
    
    try:
        print("Testing basic positions...")
        positions = [
            (2.5, "0 degrees"),
            (7.5, "90 degrees"),
            (12.5, "180 degrees"),
            (7.5, "Center")
        ]
        
        for duty, desc in positions:
            print(f"Setting to {desc} (duty: {duty}%)")
            pan_pwm.ChangeDutyCycle(duty)
            time.sleep(2)
            tilt_pwm.ChangeDutyCycle(duty)
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("Test interrupted")
    
    finally:
        pan_pwm.stop()
        tilt_pwm.stop()
        GPIO.cleanup()
        print("Cleanup complete")


class ManualControl:
    """Manual control interface for testing servos"""
    
    def __init__(self, servo_controller):
        self.servo = servo_controller
        self.step_size = 10  # Increased step size
        
    def keyboard_control(self):
        """Control servos with keyboard"""
        import sys
        
        print("\n🎮 Manual Servo Control")
        print("------------------------")
        print("W/S: Tilt up/down")
        print("A/D: Pan left/right") 
        print("C: Center position")
        print("T: Simple test")
        print("Q: Quit")
        print("------------------------\n")
        
        try:
            while True:
                key = input("Enter command: ").lower().strip()
                
                current = self.servo.get_status()
                pan = current['current_pan']
                tilt = current['current_tilt']
                
                if key == 'w':  # Tilt up
                    new_tilt = max(self.servo.tilt_min, tilt - self.step_size)
                    self.servo.set_position(tilt=new_tilt, immediate=True)
                    print(f"↑ Tilt: {new_tilt}°")
                    
                elif key == 's':  # Tilt down
                    new_tilt = min(self.servo.tilt_max, tilt + self.step_size)
                    self.servo.set_position(tilt=new_tilt, immediate=True)
                    print(f"↓ Tilt: {new_tilt}°")
                    
                elif key == 'a':  # Pan left
                    new_pan = max(self.servo.pan_min, pan - self.step_size)
                    self.servo.set_position(pan=new_pan, immediate=True)
                    print(f"← Pan: {new_pan}°")
                    
                elif key == 'd':  # Pan right
                    new_pan = min(self.servo.pan_max, pan + self.step_size)
                    self.servo.set_position(pan=new_pan, immediate=True)
                    print(f"→ Pan: {new_pan}°")
                
                elif key == 'c':  # Center
                    self.servo.center_position()
                    print("🎯 Centered")
                
                elif key == 't':  # Simple test
                    print("Running simple test...")
                    self.servo.stop()
                    self.servo.cleanup()
                    simple_servo_test()
                    return
                
                elif key == 'q':  # Quit
                    print("\n👋 Exiting manual control")
                    break
                
        except KeyboardInterrupt:
            print("\n⏹️  Manual control interrupted")


# CALIBRATION TEST FUNCTION
def calibration_test():
    """Interactive calibration test to find correct servo parameters"""
    print("🔧 Servo Calibration Test")
    print("========================")
    
    # Ask user for calibration parameters
    print("\nFirst, let's test different duty cycle ranges...")
    print("Standard ranges:")
    print("1. Conservative: 5-10% (1ms-2ms pulse)")
    print("2. Extended: 2.5-12.5% (0.5ms-2.5ms pulse)")
    print("3. Custom range")
    
    range_choice = input("Choose range (1, 2, or 3): ")
    
    if range_choice == "1":
        min_duty, max_duty = 5.0, 10.0
    elif range_choice == "2":
        min_duty, max_duty = 2.5, 12.5
    else:
        min_duty = float(input("Enter minimum duty cycle (%): "))
        max_duty = float(input("Enter maximum duty cycle (%): "))
    
    # Test specific servo
    servo_choice = input("\nTest pan (p) or tilt (t) servo? ").lower()
    pin = 16 if servo_choice == 'p' else 24
    servo_name = "Pan" if servo_choice == 'p' else "Tilt"
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)
    
    def angle_to_duty(angle):
        return min_duty + (angle / 180) * (max_duty - min_duty)
    
    try:
        print(f"\nTesting {servo_name} servo on GPIO {pin}")
        print(f"Duty cycle range: {min_duty}% - {max_duty}%")
        print("\nCommands:")
        print("0-180: Set angle")
        print("r: Reverse direction test")
        print("c: Center (90°)")
        print("q: Quit")
        
        pwm.start(angle_to_duty(90))  # Start at center
        current_angle = 90
        reversed_mode = False
        
        while True:
            cmd = input(f"\n{servo_name} at {current_angle}° - Command: ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'c':
                current_angle = 90
                actual_angle = 180 - current_angle if reversed_mode else current_angle
                duty = angle_to_duty(actual_angle)
                pwm.ChangeDutyCycle(duty)
                print(f"Center: {current_angle}° (duty: {duty:.1f}%)")
            elif cmd == 'r':
                reversed_mode = not reversed_mode
                actual_angle = 180 - current_angle if reversed_mode else current_angle
                duty = angle_to_duty(actual_angle)
                pwm.ChangeDutyCycle(duty)
                print(f"Reversed: {'ON' if reversed_mode else 'OFF'}")
                print(f"Angle: {current_angle}° → {actual_angle}° (duty: {duty:.1f}%)")
            else:
                try:
                    angle = float(cmd)
                    if 0 <= angle <= 180:
                        current_angle = angle
                        actual_angle = 180 - current_angle if reversed_mode else current_angle
                        duty = angle_to_duty(actual_angle)
                        pwm.ChangeDutyCycle(duty)
                        print(f"Set to: {current_angle}° → {actual_angle}° (duty: {duty:.1f}%)")
                    else:
                        print("Angle must be 0-180")
                except ValueError:
                    print("Invalid command")
        
        # Final calibration summary
        print(f"\n📋 Calibration Summary for {servo_name} servo:")
        print(f"   GPIO pin: {pin}")
        print(f"   Duty cycle range: {min_duty}% - {max_duty}%")
        print(f"   Reversed: {reversed_mode}")
        print(f"   Use these parameters in your servo controller!")
        
    finally:
        pwm.stop()
        GPIO.cleanup()


# Test script
if __name__ == "__main__":
    print("🔧 Testing Servo Controller...")
    print("Using GPIO pins 16 (pan) and 24 (tilt)")
    
    # Ask user which test to run
    print("\nSelect test:")
    print("1. Full servo controller test")
    print("2. Simple servo test")
    print("3. Calibration test (RECOMMENDED)")
    choice = input("Enter choice (1, 2, or 3): ")
    
    if choice == '2':
        simple_servo_test()
    elif choice == '3':
        calibration_test()
    else:
        # Ask for calibration parameters
        print("\nServo Calibration:")
        pan_reversed = input("Is pan servo reversed? (y/n): ").lower() == 'y'
        tilt_reversed = input("Is tilt servo reversed? (y/n): ").lower() == 'y'
        
        try:
            pan_offset = float(input("Pan offset degrees (-90 to +90, 0 for none): ") or "0")
            tilt_offset = float(input("Tilt offset degrees (-90 to +90, 0 for none): ") or "0")
        except ValueError:
            pan_offset = tilt_offset = 0
            
        try:
            # Initialize servo controller with calibration
            servo = ServoController(pan_pin=16, tilt_pin=24, 
                                  pan_reversed=pan_reversed, tilt_reversed=tilt_reversed,
                                  pan_offset=pan_offset, tilt_offset=tilt_offset)
            
            # Manual control
            manual = ManualControl(servo)
            manual.keyboard_control()
            
        except Exception as e:
            print(f"❌ Error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            servo.cleanup()
            print("✅ Test complete")