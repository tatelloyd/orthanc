#!/usr/bin/env python3
"""
LED Controller for Visual Feedback
Controls WS2812B LED strip and/or laser pointer
"""

import time
import RPi.GPIO as GPIO
from threading import Thread, Lock
from enum import Enum

# Try to import LED library (might need: sudo pip3 install rpi_ws281x)
try:
    from rpi_ws281x import PixelStrip, Color
    LED_AVAILABLE = True
except ImportError:
    LED_AVAILABLE = False
    print("⚠️  WS2812B library not available. Install with: sudo pip3 install rpi_ws281x")

class SystemState(Enum):
    """System states for LED indication"""
    IDLE = "idle"           # Blue breathing
    SCANNING = "scanning"   # Green sweep
    TRACKING = "tracking"   # Solid green
    ALERT = "alert"        # Red flashing
    WARNING = "warning"    # Orange pulse
    ERROR = "error"        # Red solid

class LEDController:
    def __init__(self, led_pin=21, led_count=12, laser_pin=18):
        """
        Initialize LED controller
        
        Args:
            led_pin: GPIO pin for WS2812B data (default 21)
            led_count: Number of LEDs in strip (default 12)
            laser_pin: GPIO pin for laser control (default 18)
        """
        # GPIO setup for laser
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Laser setup
        self.laser_pin = laser_pin
        GPIO.setup(self.laser_pin, GPIO.OUT)
        GPIO.output(self.laser_pin, GPIO.LOW)  # Start with laser off
        self.laser_on = False
        
        # LED strip setup
        self.led_count = led_count
        self.led_pin = led_pin
        self.strip = None
        
        if LED_AVAILABLE:
            # LED strip configuration
            LED_FREQ_HZ = 800000  # LED signal frequency in hertz
            LED_DMA = 10          # DMA channel to use for generating signal
            LED_BRIGHTNESS = 128  # Set to 0 for darkest and 255 for brightest
            LED_INVERT = False    # True to invert the signal
            LED_CHANNEL = 0       # PWM channel
            
            try:
                self.strip = PixelStrip(
                    led_count, led_pin, LED_FREQ_HZ, LED_DMA,
                    LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL
                )
                self.strip.begin()
                print(f"✅ LED strip initialized ({led_count} LEDs on GPIO {led_pin})")
            except Exception as e:
                print(f"❌ LED strip initialization failed: {e}")
                print("   Try running with sudo for LED access")
                self.strip = None
        
        # State management
        self.current_state = SystemState.IDLE
        self.lock = Lock()
        self.running = False
        self.animation_thread = None
        
        # Animation parameters
        self.animation_speed = 1.0
        self.brightness = 0.5
        
        # Color definitions (RGB)
        self.colors = {
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'blue': (0, 0, 255),
            'orange': (255, 128, 0),
            'yellow': (255, 255, 0),
            'purple': (128, 0, 255),
            'white': (255, 255, 255),
            'off': (0, 0, 0)
        }
        
        print(f"✅ LED controller initialized")
        print(f"   Laser on GPIO {laser_pin}")
    
    def set_laser(self, on):
        """Turn laser on or off"""
        GPIO.output(self.laser_pin, GPIO.HIGH if on else GPIO.LOW)
        self.laser_on = on
        status = "ON" if on else "OFF"
        print(f"🔴 Laser {status}")
    
    def pulse_laser(self, duration=0.5):
        """Pulse laser for specified duration"""
        self.set_laser(True)
        time.sleep(duration)
        self.set_laser(False)
    
    def set_all_leds(self, color):
        """Set all LEDs to the same color"""
        if not self.strip:
            return
        
        r, g, b = color
        r = int(r * self.brightness)
        g = int(g * self.brightness)
        b = int(b * self.brightness)
        
        for i in range(self.led_count):
            self.strip.setPixelColor(i, Color(r, g, b))
        self.strip.show()
    
    def set_led(self, index, color):
        """Set individual LED color"""
        if not self.strip or index >= self.led_count:
            return
        
        r, g, b = color
        r = int(r * self.brightness)
        g = int(g * self.brightness)
        b = int(b * self.brightness)
        
        self.strip.setPixelColor(index, Color(r, g, b))
        self.strip.show()
    
    def clear_leds(self):
        """Turn off all LEDs"""
        if self.strip:
            self.set_all_leds((0, 0, 0))
    
    def set_state(self, state):
        """Change system state and LED pattern"""
        with self.lock:
            self.current_state = state
            print(f"💡 LED State: {state.value}")
            
            # Immediate visual feedback
            if state == SystemState.ALERT:
                self.pulse_laser(0.1)  # Quick laser pulse for alerts
    
    def _breathing_effect(self, color, speed=1.0):
        """Create breathing effect with given color"""
        import math
        phase = 0
        while self.running and self.current_state in [SystemState.IDLE, SystemState.WARNING]:
            brightness = (math.sin(phase) + 1) / 2  # 0 to 1
            r, g, b = color
            self.set_all_leds((r * brightness, g * brightness, b * brightness))
            phase += 0.1 * speed
            time.sleep(0.05)
    
    def _sweep_effect(self, color, speed=1.0):
        """Create sweeping effect"""
        position = 0
        direction = 1
        
        while self.running and self.current_state == SystemState.SCANNING:
            self.clear_leds()
            
            # Create trail effect
            for i in range(3):
                led_pos = position - i * direction
                if 0 <= led_pos < self.led_count:
                    brightness = 1.0 - (i * 0.3)
                    r, g, b = color
                    self.set_led(led_pos, (r * brightness, g * brightness, b * brightness))
            
            position += direction
            if position >= self.led_count - 1 or position <= 0:
                direction *= -1
            
            time.sleep(0.1 / speed)
    
    def _flash_effect(self, color, speed=1.0):
        """Create flashing effect"""
        on = False
        while self.running and self.current_state == SystemState.ALERT:
            self.set_all_leds(color if on else (0, 0, 0))
            on = not on
            time.sleep(0.2 / speed)
    
    def _animation_loop(self):
        """Main animation loop"""
        while self.running:
            with self.lock:
                state = self.current_state
            
            if state == SystemState.IDLE:
                self._breathing_effect(self.colors['blue'], self.animation_speed)
            
            elif state == SystemState.SCANNING:
                self._sweep_effect(self.colors['green'], self.animation_speed)
            
            elif state == SystemState.TRACKING:
                self.set_all_leds(self.colors['green'])
                time.sleep(0.1)
            
            elif state == SystemState.ALERT:
                self._flash_effect(self.colors['red'], self.animation_speed * 2)
            
            elif state == SystemState.WARNING:
                self._breathing_effect(self.colors['orange'], self.animation_speed * 1.5)
            
            elif state == SystemState.ERROR:
                self.set_all_leds(self.colors['red'])
                time.sleep(0.1)
            
            time.sleep(0.01)
    
    def show_detection_confidence(self, confidence):
        """
        Show detection confidence on LED strip
        
        Args:
            confidence: Detection confidence (0.0 to 1.0)
        """
        if not self.strip:
            return
        
        # Map confidence to number of LEDs
        num_leds = int(confidence * self.led_count)
        
        for i in range(self.led_count):
            if i < num_leds:
                # Green to red gradient based on position
                green = int(255 * (1 - i / self.led_count))
                red = int(255 * (i / self.led_count))
                self.set_led(i, (red, green, 0))
            else:
                self.set_led(i, (0, 0, 0))
    
    def indicate_direction(self, pan_error, tilt_error, threshold=10):
        """
        Show tracking direction on LEDs
        
        Args:
            pan_error: Pan tracking error in degrees
            tilt_error: Tilt tracking error in degrees
            threshold: Error threshold for indication
        """
        if not self.strip:
            return
        
        self.clear_leds()
        
        # Map errors to LED positions
        # Assuming circular LED arrangement or strip
        center = self.led_count // 2
        
        if abs(pan_error) > threshold:
            # Show pan direction
            if pan_error > 0:  # Target is to the right
                for i in range(center, min(center + 3, self.led_count)):
                    self.set_led(i, self.colors['yellow'])
            else:  # Target is to the left
                for i in range(max(0, center - 3), center):
                    self.set_led(i, self.colors['yellow'])
        
        if abs(tilt_error) > threshold:
            # Show tilt direction with different color
            if tilt_error > 0:  # Target is up
                self.set_led(0, self.colors['purple'])
            else:  # Target is down
                self.set_led(self.led_count - 1, self.colors['purple'])
    
    def start(self):
        """Start LED animation thread"""
        if not self.running:
            self.running = True
            if self.strip:
                self.animation_thread = Thread(target=self._animation_loop)
                self.animation_thread.daemon = True
                self.animation_thread.start()
                print("✅ LED animation started")
            else:
                print("⚠️  LED strip not available, animations disabled")
    
    def stop(self):
        """Stop LED animations"""
        self.running = False
        if self.animation_thread:
            self.animation_thread.join()
        self.clear_leds()
        self.set_laser(False)
        print("⏹️  LED animations stopped")
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
        GPIO.cleanup()
        print("🧹 LED controller cleaned up")
    
    def test_sequence(self):
        """Run a test sequence to verify all LEDs and laser"""
        print("\n🧪 Running LED test sequence...")
        
        # Test laser
        print("  Testing laser...")
        self.pulse_laser(0.5)
        time.sleep(0.5)
        
        if self.strip:
            # Test each color
            for color_name, color_value in self.colors.items():
                if color_name != 'off':
                    print(f"  Testing {color_name}...")
                    self.set_all_leds(color_value)
                    time.sleep(0.5)
            
            # Test individual LEDs
            print("  Testing individual LEDs...")
            self.clear_leds()
            for i in range(self.led_count):
                self.set_led(i, self.colors['white'])
                time.sleep(0.1)
                self.set_led(i, (0, 0, 0))
            
            # Test patterns
            states = [
                SystemState.IDLE,
                SystemState.SCANNING,
                SystemState.TRACKING,
                SystemState.WARNING,
                SystemState.ALERT,
                SystemState.ERROR
            ]
            
            for state in states:
                print(f"  Testing {state.value} pattern...")
                self.set_state(state)
                time.sleep(2)
        
        self.clear_leds()
        print("✅ LED test complete")


# Test script
if __name__ == "__main__":
    print("🔧 Testing LED Controller...")
    
    try:
        # Initialize LED controller
        leds = LEDController(led_pin=21, led_count=12, laser_pin=18)
        
        # Run test sequence
        leds.test_sequence()
        
        # Start animations
        leds.start()
        
        print("\n🎮 Interactive LED Control")
        print("------------------------")
        print("1-6: Change state")
        print("L: Toggle laser")
        print("T: Test sequence")
        print("Q: Quit")
        print("------------------------\n")
        
        while True:
            cmd = input("Command: ").strip().lower()
            
            if cmd == '1':
                leds.set_state(SystemState.IDLE)
            elif cmd == '2':
                leds.set_state(SystemState.SCANNING)
            elif cmd == '3':
                leds.set_state(SystemState.TRACKING)
            elif cmd == '4':
                leds.set_state(SystemState.WARNING)
            elif cmd == '5':
                leds.set_state(SystemState.ALERT)
            elif cmd == '6':
                leds.set_state(SystemState.ERROR)
            elif cmd == 'l':
                leds.set_laser(not leds.laser_on)
            elif cmd == 't':
                leds.test_sequence()
            elif cmd == 'q':
                break
        
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        leds.cleanup()
        print("✅ Test complete")
