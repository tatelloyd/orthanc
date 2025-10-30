// Libraries
#include <iostream>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <algorithm>
#include <pigpiod_if2.h>

// Header Files
#include "SignalGenerator.hpp"
#include "ServoController.hpp"

// Use a volatile atomic flag for safe access across threads/signal handlers
std::atomic<bool> program_running(true);

void signal_handler(int signum) {
    std::cout << "\nSIGINT (" << signum << ") received. Shutting down gracefully..." << std::endl;
    program_running = false; // Set the flag to stop the main loop
}

int main(){ 
     //Use std::chrono to reduce complexity
    using namespace std::chrono;

    // Connect to pigpio daemon
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        std::cerr << "Failed to connect to pigpio daemon" << std::endl;
        return 1;
    }
    std::cout << "Connected to daemon" << std::endl;
    
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

    ServoController panController(17, pi);
    ServoController tiltController(27, pi, 750, 2500, 1000);

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
        
        // Send to servo (clamp to valid PWM range)
        panController.setPulseWidth(sine_value);
        tiltController.setPulseWidth(square_value);
        
        // Optional: Log for debugging
        if (static_cast<int>(elapsed_time*10000 ) % 10000 == 0){  // Every second
            std::cout << "t=" << elapsed_time << "s, pan angle=" << panController.getCurrentAngle() << 
            "°, tilt angle=" << tiltController.getCurrentAngle() << "°" << std::endl;
        }

        // Sleep to maintain loop rate
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt)*1000));
    }

    // Reset servo to center
    panController.center();
    tiltController.center();
    pigpio_stop(pi);

    return 0;
}

// Compile: g++ -o orthanc tower.cpp ServoController.cpp SignalGenerator.cpp -lpigpiod_if2 -std=c++20