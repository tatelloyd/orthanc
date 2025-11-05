// Libraries
#include <iostream>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <algorithm>
#include <pigpiod_if2.h>

// Header Files
#include "../../src/cpp/Turret.hpp"
#include "../../src/cpp/SignalGenerator.hpp"
#include "../../src/cpp/ServoController.hpp"

// Use a volatile atomic flag for safe access across threads/signal handlers
std::atomic<bool> program_running(true);

void signal_handler(int signum) {
    std::cout << "\nSIGINT (" << signum << ") received. Shutting down gracefully..." << std::endl;
    program_running = false; // Set the flag to stop the main loop
}

int main(){ 
     //Use std::chrono to reduce complexity
    using namespace std::chrono;
    
    struct sigaction sa;
    sa.sa_handler = signal_handler; // Set the handler function
    sigemptyset(&sa.sa_mask);        // Clear the signal mask
    sa.sa_flags = 0;                 // No special flags

    // Register the signal handler for SIGINT
    if (sigaction(SIGINT, &sa, nullptr) == -1) {
        std::cerr << "Error registering signal handler" << std::endl;
        return 1;
    }

    std::cout << "Orthanc is moving! Press Ctrl+C to stop." << std::endl;

    // Initialize time variables
    auto start_time = steady_clock::now();
    const double simulation_time = 15.0;

    // Make turret object
    Turret orthanc(17, 27);

    // Make the test signals
    SignalGenerator sig1(700, 6.0, 1500);
    SignalGenerator sig2(700, 3.0, 1500);

    // Loop runs at 50Hz
    const double dt = 0.02;

    // While simulation is occuring
    while (program_running){
        //Calculate the point in time of the simmulation
        auto current_time = steady_clock::now();
        double elapsed_time = duration<double>(current_time - start_time).count();

        // Stop after simulation time
        if (elapsed_time >= simulation_time)
            break;
        
       // Evaluate signal at current time
        double sine_value = sig1.sine(elapsed_time);
        double square_value = sig2.triangle(elapsed_time);
        
        // Send to tower, which will clamp to valid PWM range via
        // The servo controllers
        orthanc.setPanPulseWidth(sine_value);
        orthanc.setTiltPulseWidth(square_value);
        
        // Optional: Log for debugging
        if (static_cast<int>(elapsed_time*10000 ) % 10000 == 0){  // Every second
            std::cout << "t=" << elapsed_time << "s, pan angle=" << orthanc.getPanAngle() << 
            "°, tilt angle=" << orthanc.getTiltAngle() << "°" << std::endl;
        }

        // Sleep to maintain loop rate
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt)*1000));
    }

    return 0;
}

// Compile: g++ -o test_pan_tilt test_pan_tilt_servo.cpp ../../src/cpp/Turret.cpp ../../src/cpp/ServoController.cpp ../../src/cpp/SignalGenerator.cpp -lpigpiod_if2 -std=c++20 -O2