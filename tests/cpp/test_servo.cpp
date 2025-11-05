#include <iostream>
#include <string>
#include <thread>
#include <pigpiod_if2.h>

int main() {
    // Connect to pigpio daemon
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        std::cerr << "Failed to connect to pigpio daemon" << std::endl;
        return 1;
    }
    std::cout << "Connected to daemon" << std::endl;

    // Set both of the pins
    int pan_pin = 17;
    int tilt_pin = 27;
    
    
    // Move servo to different positions
    std::cout << "Moving pan 1000us (minimum)" << std::endl;
    set_servo_pulsewidth(pi, pan_pin, 1000);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Moving tilt to 1000us (minimum)" << std::endl;
    set_servo_pulsewidth(pi, tilt_pin, 1000);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "Moving pan to 1500us (center)" << std::endl;
    set_servo_pulsewidth(pi, pan_pin, 1500);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Moving tilt to  1500us (minimum)" << std::endl;
    set_servo_pulsewidth(pi, tilt_pin, 1500);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "Moving to 2000us (maximum)" << std::endl;
    set_servo_pulsewidth(pi, pan_pin, 2000);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Moving pan 1000us (minimum)" << std::endl;
    set_servo_pulsewidth(pi, tilt_pin, 2000);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "Back to center" << std::endl;
    set_servo_pulsewidth(pi, pan_pin, 1500);
    set_servo_pulsewidth(pi, tilt_pin, 1500);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Stop servos
    set_servo_pulsewidth(pi, pan_pin, 0);
    set_servo_pulsewidth(pi, tilt_pin, 0);
    
    pigpio_stop(pi);
    std::cout << "Done!" << std::endl;
    
    return 0;
}

// Compile: g++ -o test_servo test_servo.cpp -lpigpiod_if2

