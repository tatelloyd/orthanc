#!/usr/bin/env python3
"""
Servo Diagnostic Tool
Tests servo functionality and diagnoses common failures
"""

import pigpio
import time
import sys

def test_servo(pi, pin, name):
    """Test a single servo on the given pin"""
    print("\n===========================================")
    print(f"Testing {name} servo on GPIO {pin}")
    print("===========================================\n")
    
    # Test 1: Can we set the pin as output?
    print("TEST 1: Setting pin as output... ", end="")
    try:
        pi.set_mode(pin, pigpio.OUTPUT)
        print("✓ OK")
    except Exception as e:
        print(f"❌ FAILED ({e})")
        print("   → Pin may be damaged or in use by another process")
        return False
    
    # Test 2: Send middle position pulse (1500μs)
    print("\nTEST 2: Sending 1500μs pulses (center position)...")
    print("   Watch the servo - it should move to center if it works")
    
    for i in range(50):  # 1 second worth
        pi.set_servo_pulsewidth(pin, 1500)
        time.sleep(0.02)
    
    response = input("   Did the servo move? (y/n): ").strip().lower()
    
    if response != 'y':
        print("\n⚠️  Servo didn't move. Possible causes:")
        print("   1. Servo is physically broken (most common after overdriving)")
        print("   2. Loose/disconnected wire on signal pin")
        print("   3. Servo power supply issue (but tilt works, so unlikely)")
        print("   4. GPIO pin burned out (can happen with voltage spikes)")
        
        # Test 3: Check if we can read back the pin state
        print("\nTEST 3: Checking GPIO pin health...")
        pi.write(pin, 1)
        time.sleep(0.01)
        state = pi.read(pin)
        print(f"   Set HIGH, read back: {state} {'✓' if state == 1 else '❌'}")
        
        pi.write(pin, 0)
        time.sleep(0.01)
        state = pi.read(pin)
        print(f"   Set LOW, read back: {state} {'✓' if state == 0 else '❌'}")
        
        if state != 0:
            print("\n❌ GPIO pin is not responding correctly")
            print("   → The GPIO pin may be damaged")
        else:
            print("\n✓ GPIO pin works for basic I/O")
            print("   → But servo doesn't respond to PWM")
            print("   → Most likely: SERVO IS BROKEN")
        
        pi.set_servo_pulsewidth(pin, 0)  # Turn off servo
        return False
    
    # Test 4: Full range sweep
    print("\nTEST 4: Full range sweep...")
    print("   Moving from 500μs → 2500μs")
    print("   Watch for smooth motion across full range\n")
    
    # Sweep from min to max
    for pulse in range(500, 2501, 100):
        print(f"   Pulse: {pulse}μs", end='\r')
        sys.stdout.flush()
        pi.set_servo_pulsewidth(pin, pulse)
        time.sleep(0.3)
    print()
    
    # Sweep back
    for pulse in range(2500, 499, -100):
        print(f"   Pulse: {pulse}μs", end='\r')
        sys.stdout.flush()
        pi.set_servo_pulsewidth(pin, pulse)
        time.sleep(0.3)
    print("\n")
    
    response = input("Did the servo move smoothly through its full range? (y/n): ").strip().lower()
    
    if response != 'y':
        print("\n⚠️  LIMITED OR NO MOVEMENT:")
        print("   • Grinding noise? → Stripped gears (mechanical failure)")
        print("   • Jittery motion? → Weak power supply or bad servo")
        print("   • No motion at all? → Servo electronics dead")
        servo_ok = False
    else:
        print("\n✓ Servo appears to be working!")
        servo_ok = True
    
    # Return to center
    pi.set_servo_pulsewidth(pin, 1500)
    time.sleep(0.5)
    pi.set_servo_pulsewidth(pin, 0)  # Turn off servo
    
    return servo_ok

def main():
    print("🔧 SERVO DIAGNOSTIC TOOL")
    print("========================\n")
    
    # Initialize pigpio
    pi = pigpio.pi()
    if not pi.connected:
        print("❌ Failed to connect to pigpio daemon")
        print("   Make sure pigpiod is running: sudo pigpiod")
        print("   Or run this script with sudo if pigpiod isn't running")
        return 1
    
    print("✓ Connected to pigpio daemon\n")
    
    # Ask which servo to test
    print("Which servo is broken?")
    print("1. Pan servo (GPIO 17)")
    print("2. Tilt servo (GPIO 27)")
    print("3. Test both")
    
    try:
        choice = int(input("Choice: ").strip())
    except ValueError:
        print("Invalid choice")
        pi.stop()
        return 1
    
    pan_ok = True
    tilt_ok = True
    
    if choice == 1 or choice == 3:
        pan_ok = test_servo(pi, 17, "PAN")
    
    if choice == 2 or choice == 3:
        tilt_ok = test_servo(pi, 27, "TILT")
    
    # Summary
    print("\n===========================================")
    print("DIAGNOSTIC SUMMARY")
    print("===========================================\n")
    
    if not pan_ok:
        print("❌ PAN SERVO FAILED\n")
    if not tilt_ok:
        print("❌ TILT SERVO FAILED\n")
    
    if not (pan_ok and tilt_ok):
        print("TROUBLESHOOTING STEPS:\n")
        print("1. CHECK WIRING:")
        print("   • Signal wire connected to correct GPIO?")
        print("     - Pan: GPIO 17 (usually orange/yellow wire)")
        print("     - Tilt: GPIO 27")
        print("   • Ground connected? (brown/black wire)")
        print("   • Power connected? (red wire to +5V)\n")
        
        print("2. SWAP TEST:")
        print("   • Disconnect working servo")
        print("   • Connect broken servo to working servo's GPIO pin")
        print("   • Run this test again")
        print("   • If it works → GPIO pin is broken")
        print("   • If it doesn't → Servo is broken\n")
        
        print("3. COMMON FAILURE MODES:")
        print("   • Servo driven to limits repeatedly → stripped gears")
        print("   • Voltage spike → burned electronics")
        print("   • Overheating → thermal shutdown (may recover)")
        print("   • Physical impact → broken gears\n")
        
        print("4. IF SERVO IS BROKEN:")
        print("   • SG90 servos cost ~$2-5")
        print("   • MG90S (metal gear) are more durable: ~$5-8")
        print("   • Order spares - they're consumable parts!\n")
        
        print("5. WHAT LIKELY HAPPENED:")
        print("   Your logs show the pan servo hitting 170° limit repeatedly.")
        print("   If your servo's physical range is <170°, it was fighting")
        print("   against mechanical stops, which strips plastic gears.")
        print("   This is the #1 failure mode for cheap servos.\n")
    else:
        print("✓ Both servos working normally!")
    
    pi.stop()
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
