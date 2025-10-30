#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

class ServoController {
public:
    // Constructor: connects to daemon and initializes servo
    ServoController(int pin_num, int pi_handle, 
        int min_pw = 500, int max_pw = 2500, int center_pw = 1500);
    
    // Destructor: centers servo (daemon cleanup handled by Turret)
    ~ServoController();
    
    // Delete copy constructor and assignment (prevent accidental copies)
    ServoController(const ServoController&) = delete;
    ServoController& operator=(const ServoController&) = delete;
    
    // Core control methods
    void setAngle(double degrees);           // -90 to +90 (or your range)
    void setPulseWidth(int microseconds);    // Direct PWM control
    void center();                            // Return to center position
    
    // Getters
    int getCurrentPulseWidth() const { return current_pw; }
    double getCurrentAngle() const;
    
private:
    int pi;              // pigpio daemon handle
    int pin;             // GPIO pin number
    int min_pw;          // Minimum pulse width (microseconds)
    int max_pw;          // Maximum pulse width (microseconds)
    int center_pw;       // Center pulse width (microseconds)
    int current_pw;      // Current pulse width (for tracking)
    
    // Helper functions
    int clampPulseWidth(int pw) const;
    int angleToPulseWidth(double degrees) const;
    double pulseWidthToAngle(int pw) const;
};

#endif // SERVO_CONTROLLER_H