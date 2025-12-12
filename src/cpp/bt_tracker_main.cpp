#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "bt_nodes.hpp"
#include "Turret.hpp"

std::atomic<bool> running(true);

void signal_handler(int signum) {
    std::cout << "\n🛑 Shutting down..." << std::endl;
    running = false;
}

int main() {
    // Signal handler
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    
    // Initialize turret
    std::cout << "🤖 Initializing turret...\n";
    Turret turret(17, 27);  // Your GPIO pins
    turret.setPanAngle(90);
    turret.setTiltAngle(30);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "✅ Turret centered at (90, 90)\n\n";

    // Set turret in shared state
    TrackerState::get().turret = &turret;
    
    // Create factory and register nodes
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<HasPersonDetection>("HasPersonDetection");
    factory.registerNodeType<ProportionalTrackAction>("ProportionalTrack");
    factory.registerNodeType<SimpleScanAction>("SimpleScan");
    
    // Load tree from XML
    std::cout << "Loading tree from: trees/basic_track.xml\n";
    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile("trees/basic_track.xml");
    } catch (const std::exception& e) {
        std::cerr << "❌ Failed to load tree: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "✅ Tree loaded successfully\n\n";
    
    // Tick loop (20 Hz)
    std::cout << "🔄 Starting tick loop (Ctrl+C to stop)...\n\n";
    int tick_count = 0;
    

    /*// In bt_tracker_main.cpp, before the tick loop:
    std::cout << "\n🧪 DIRECTION TEST:\n";
    std::cout << "Current pan: " << turret.getPanAngle() << "°\n";
    std::cout << "Moving to 45°...\n";
    turret.setPanAngle(45);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Moving to 135°...\n";
    turret.setPanAngle(135);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Does 45° point LEFT and 135° point RIGHT? (y/n): ";
    std::string response;
    std::cin >> response;
    if (response != "y") {
        std::cout << "❌ Pan direction is inverted!\n";
        return 1;
    }
    std::cout << "✅ Pan direction confirmed\n\n";*/

    /*std::cout << "\n🧪 TILT TEST (camera is upside down):\n";
    turret.setTiltAngle(45);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Is camera pointing DOWN? (y/n): ";
    std::cin >> response;
    if (response != "y") {
        std::cout << "❌ Tilt direction needs inversion!\n";
        return 1;
    }*/


    while (running) {
        BT::NodeStatus status = tree.tickOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Cleanup
    std::cout << "\n🔄 Returning to center...\n";
    turret.centerAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "\n✅ Shutdown complete\n";
    return 0;
}

