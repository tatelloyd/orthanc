// Libraries
#include <iostream>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <pigpiod_if2.h>

// Header Files
#include "../version/Turret.hpp"
#include "../version/SignalGenerator.hpp"
#include "../version/ServoController.hpp"
#include <sys/wait.h>

// Use a volatile atomic flag for safe access across threads/signal handlers
std::atomic<bool> program_running(true);

void signal_handler(int signum) {
    std::cout << "\nSIGINT (" << signum << ") received. Shutting down gracefully..." << std::endl;
    program_running = false; // Set the flag to stop the main loop
}

// Clear input buffer
void clearInput() {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

// Display menu and get user choice
int getMenuChoice() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║          ORTHANC TURRET CONTROL MENU                   ║\n";
    std::cout << "╠════════════════════════════════════════════════════════╣\n";
    std::cout << "║  1. Pan back and forth for 10 seconds                  ║\n";
    std::cout << "║  2. Tilt back and forth for 10 seconds                 ║\n";
    std::cout << "║  3. Run YOLO detection (print objects to screen)       ║\n";
    std::cout << "║  4. Set pan angle (0-180 degrees)                      ║\n";
    std::cout << "║  5. Set tilt angle (0-180 degrees)                     ║\n";
    std::cout << "║  6. Center turret (90°, 90°)                           ║\n";
    std::cout << "║  7. Live detection while moving                        ║\n";
    std::cout << "║  0. Exit                                               ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n";
    std::cout << "Enter choice: ";
    
    int choice;
    std::cin >> choice;
    
    if (std::cin.fail()) {
        clearInput();
        return -1;
    }
    
    clearInput();
    return choice;
}

// Template function for servo testing (pan or tilt)
template<typename SetPulseFunc, typename GetAngleFunc>
void testServo(const std::string& servoName, Turret& turret, 
               SetPulseFunc setPulse, GetAngleFunc getAngle, double endPulse) {
    using namespace std::chrono;
    
    std::cout << "\n🔄 Testing " << servoName << " servo (10 seconds)...\n";
    std::cout << "Moving from 0° to 180° and back\n";
    
    auto start_time = steady_clock::now();
    const double test_duration = 10.0;
    const double dt = 0.02;  // 50Hz
    
    // Create a sine wave signal (0-180 degrees over 10 seconds)
    SignalGenerator signal(900, 2.0, 1500);  // Full range, 10s period
    
    while (std::chrono::duration_cast<std::chrono::duration<double>>(
            steady_clock::now() - start_time).count() < test_duration) {
        
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            steady_clock::now() - start_time).count();
        
        double pulse = signal.sine(elapsed);
        (turret.*setPulse)(pulse);  // Call member function through pointer
        
        // Print status every 0.5s
        if (static_cast<int>(elapsed * 10) % 5 == 0) {
            std::cout << "  t=" << elapsed << "s, " << servoName 
                      << " angle: " << (turret.*getAngle)() << "°\n";
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
    
    // Return to center
    (turret.*setPulse)(endPulse);
    std::cout << "✅ " << servoName << " test complete. Returned to center.\n";
}

// Test function 1: Pan back and forth
void testPan(Turret& turret) {
    testServo("PAN", turret, &Turret::setPanPulseWidth, &Turret::getPanAngle, 1500);
}

// Test function 2: Tilt back and forth
void testTilt(Turret& turret) {
    testServo("TILT", turret, &Turret::setTiltPulseWidth, &Turret::getTiltAngle, 1000);
}

// Test function 3: Run YOLO detection
void testYOLO() {
    std::cout << "\n🎯 Starting YOLO detection...\n";
    std::cout << "Running Python detection script for 10 seconds\n";
    std::cout << "Press Ctrl+C in the detection window if you want to stop early\n\n";
    
    // Call the Python script
    int result = system("python3 scripts/yolo_detector.py");
    
    if (result == 0) {
        std::cout << "\n✅ YOLO detection completed successfully\n";
    } else {
        std::cout << "\n⚠️  YOLO detection exited with code " << result << "\n";
    }
}

template<typename SetAngleFunc, typename GetAngleFunc>
void setAngle(const std::string& servoName, Turret& turret, SetAngleFunc setAngle, GetAngleFunc getAngle) {
    std::cout << "\n📐 Enter " << servoName <<" angle (0-180 degrees): ";
    double angle;
    std::cin >> angle;
    
    if (std::cin.fail()) {
        clearInput();
        std::cout << "❌ Invalid input\n";
        return;
    }
    clearInput();
    
    if (angle < 0 || angle > 180) {
        std::cout << "❌ Angle must be between 0 and 180 degrees\n";
        return;
    }
    
    (turret.*setAngle)(angle);
    std::cout << "✅ " << servoName << " set to " << (turret.*getAngle)() << "° (PWM: " 
              << (600 + angle * 10) << "μs)\n";
}

// Test function 4: Set pan angle
void setPanAngle(Turret& turret) {
   setAngle("PAN", turret, &Turret::setPanAngle, &Turret::getPanAngle);
}


// Test function 5: Set tilt angle
void setTiltAngle(Turret& turret) {
    setAngle("Tilt", turret, &Turret::setTiltAngle, &Turret::getTiltAngle);
}

// Test function 6: Center the turret
void centerTurret(Turret& turret) {
    std::cout << "\n🎯 Centering turret...\n";
    turret.centerAll();
    std::cout << "✅ Turret centered at (90°, 90°)\n";
}

// Test function 7: YOLO detection while moving servos
void testYOLOWithMovement(Turret& turret) {
    using namespace std::chrono;
    
    std::cout << "\n🎯🔄 Starting YOLO detection with servo movement...\n";
    std::cout << "The turret will scan while detecting objects for 20 seconds\n";
    std::cout << "This demonstrates simultaneous camera detection and servo control!\n\n";
    
    // Fork a child process to run YOLO detector
    pid_t pid = fork();
    
    if (pid < 0) {
        std::cerr << "❌ Failed to fork process\n";
        return;
    }
    
    if (pid == 0) {
        // Child process: run YOLO detector
        // Redirect to avoid mixing output too much
        execlp("python3", "python3", "scripts/yolo_detector.py", nullptr);
        exit(1);  // If exec fails
    }
    
    // Parent process: move servos while detection runs
    std::cout << "🤖 Servo scanning pattern starting...\n\n";
    
    auto start_time = steady_clock::now();
    const double scan_duration = 20.0;  // Run for 20 seconds (2x YOLO duration)
    const double dt = 0.02;
    
    // Create scanning patterns
    SignalGenerator panSignal(700, 2.0, 1500);    // Slow pan sweep
    SignalGenerator tiltSignal(400, 2.0, 1300);   // Smaller tilt range
    
    while (std::chrono::duration_cast<std::chrono::duration<double>>(
            steady_clock::now() - start_time).count() < scan_duration) {
        
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            steady_clock::now() - start_time).count();
        
        // Move both servos in a scanning pattern
        double pan_pulse = panSignal.sine(elapsed);
        double tilt_pulse = tiltSignal.triangle(elapsed);
        
        turret.setPanPulseWidth(pan_pulse);
        turret.setTiltPulseWidth(tilt_pulse);
        
        // Print status every 2 seconds
        if (static_cast<int>(elapsed * 10) % 20 == 0) {
            std::cout << "  [Servo] t=" << elapsed << "s, Pan: " 
                      << turret.getPanAngle() << "°, Tilt: " 
                      << turret.getTiltAngle() << "°\n";
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
    
    // Wait for YOLO process to finish (it should be done by now)
    int status;
    waitpid(pid, &status, 0);
    
    // Return to center
    centerTurret(turret);
    
    std::cout << "\n✅ Simultaneous detection and movement test complete!\n";
    std::cout << "   Servos returned to center position.\n";
}

int main(){ 
    // Set up signal handler    
    struct sigaction sa;
    sa.sa_handler = signal_handler; // Set the handler function
    sigemptyset(&sa.sa_mask);        // Clear the signal mask
    sa.sa_flags = 0;                 // No special flags

    // Register the signal handler for SIGINT
    if (sigaction(SIGINT, &sa, nullptr) == -1) {
        std::cerr << "Error registering signal handler" << std::endl;
        return 1;
    }

    std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
    std::cout << "║              ORTHANC TURRET SYSTEM v1.0                ║\n";
    std::cout << "║              Initializing...                           ║\n";
    std::cout << "╚════════════════════════════════════════════════════════╝\n";

    // Make turret object
    Turret orthanc(17, 27);

    // Center the turret initially
    std::cout << "\nCentering turret to default position (90°, 90°)...\n";
    orthanc.setPanAngle(90);
    orthanc.setTiltAngle(45);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "✅ Turret initialized and ready\n";

   // Main menu loop
    while (program_running) {
        int choice = getMenuChoice();
        
        switch (choice) {
            case 1:
                testPan(orthanc);
                break;
                
            case 2:
                testTilt(orthanc);
                break;
                
            case 3:
                testYOLO();
                break;
                
            case 4:
                setPanAngle(orthanc);
                break;
                
            case 5:
                setTiltAngle(orthanc);
                break;
                
            case 6:
                centerTurret(orthanc);
                break;

            case 7:
                testYOLOWithMovement(orthanc);
                break;

            case 0:
                std::cout << "\n👋 Exiting...\n";
                program_running = false;
                break;
                
            default:
                std::cout << "❌ Invalid choice. Please enter 0-6.\n";
                break;
        }
    }

    // Return to center on exit
    std::cout << "\n🔄 Returning to center position before shutdown...\n";
    centerTurret(orthanc);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "✅ Shutdown complete. Goodbye!\n\n";

    return 0;
}

// Compile: g++ -o orthanc main.cpp ../version/Turret.cpp ../version/ServoController.cpp ../version/SignalGenerator.cpp -lpigpiod_if2 -std=c++20 -O2