#include "ServoController.hpp"
#include <pigpiod_if2.h>
#include <iostream>
#include <algorithm>

ServoController::ServoController(int pin_num, int pi_handle,
                                 int min_pw, int max_pw, int center_pw)
    : pi(pi_handle), pin(pin_num), min_pw(min_pw), max_pw(max_pw), 
      center_pw(center_pw), current_pw(center_pw)
{
    // Initialize servo to center position
    set_servo_pulsewidth(pi, pin, center_pw);
    current_pw = center_pw;
}

ServoController::~ServoController() {
    // Center servo before shutdown
    set_servo_pulsewidth(pi, pin, center_pw);
    
    // NOTE: We do NOT call pigpio_stop(pi) here!
    // The pi handle is shared across servos and managed by Turret class
}

void ServoController::setAngle(double degrees) {
    int pw = angleToPulseWidth(degrees);
    setPulseWidth(pw);
}

void ServoController::setPulseWidth(int microseconds) {
    int clamped_pw = clampPulseWidth(microseconds);
    set_servo_pulsewidth(pi, pin, clamped_pw);
    current_pw = clamped_pw;
}

void ServoController::center() {
    setPulseWidth(center_pw);
}

double ServoController::getCurrentAngle() const {
    return pulseWidthToAngle(current_pw);
}

// Private helper methods

int ServoController::clampPulseWidth(int pw) const {
    return std::clamp(pw, min_pw, max_pw);
}

int ServoController::angleToPulseWidth(double degrees) const {
    // Map -90 to +90 degrees to min_pw to max_pw
    // Adjust this mapping based on your servo's actual range
    double normalized = degrees / 180.0;  // 0.0 to 1.0
    return static_cast<int>(min_pw + normalized * (max_pw - min_pw));
}

double ServoController::pulseWidthToAngle(int pw) const {
    // Inverse of angleToPulseWidth
    double normalized = static_cast<double>(pw - min_pw) / (max_pw - min_pw);
    return normalized * 180.0;
}