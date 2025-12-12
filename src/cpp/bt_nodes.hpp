#ifndef BT_NODES_HPP
#define BT_NODES_HPP
#include <behaviortree_cpp/behavior_tree.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <nlohmann/json.hpp>
#include <sys/file.h>

#include "Turret.hpp"

using json = nlohmann::json;

// Shared state for nodes
struct TrackerState {
    std::string detection_file = "detections.json";
    std::vector<json> current_detections;
    json target_person;
    bool has_person = false;
    double smoothed_x = 0.5;
    double smoothed_y = 0.5;

    // Loss recovery
    int frames_since_detection = 0;
    std::chrono::steady_clock::time_point last_detection_time;
    const double loss_timeout_seconds = 2.0;  // Wait 2s before giving up

    // Tracking state
    int consecutive_centered_frames = 0;
    const int frames_needed_to_lock = 5;  // 0.5s at 10Hz
    

    Turret* turret = nullptr;
    
    static TrackerState& get() {
        static TrackerState instance;
        return instance;
    }
};

// Condition: Check if we have a person in detections OR recently had one
class HasPersonDetection : public BT::ConditionNode {
public:
    HasPersonDetection(const std::string& name) 
        : BT::ConditionNode(name, {}) {}
    
    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();
        state.has_person = false;
        state.current_detections.clear();

        std::cout << "🔍 Reading: " << state.detection_file << std::endl;
        
        // Open with file descriptor for locking
        int fd = open(state.detection_file.c_str(), O_RDONLY);
        if (fd < 0) {
            std::cout << "⚠️  Cannot open " << state.detection_file << std::endl;
            state.frames_since_detection++;
            return BT::NodeStatus::FAILURE;
        }
        
        // Acquire shared lock
        if (flock(fd, LOCK_SH) != 0) {
            close(fd);
            std::cout << "⚠️  Cannot lock file" << std::endl;
            state.frames_since_detection++;
            return BT::NodeStatus::FAILURE;
        }
        
        std::ifstream file(state.detection_file);
        
        try {
            json data;
            file >> data;
            
            flock(fd, LOCK_UN);
            close(fd);

            // Check timestamp
            if (data.contains("timestamp")) {
                double timestamp = data["timestamp"];
                auto now = std::chrono::system_clock::now();
                double current_time = std::chrono::duration<double>(
                    now.time_since_epoch()).count();
                
                double age = current_time - timestamp;
                
                if (age > 0.5) {
                    std::cout << "⚠️  Stale detection (" << age << "s old)" << std::endl;
                    state.frames_since_detection++;
                    return BT::NodeStatus::FAILURE;
                }
                std::cout << "   Detection age: " << (age * 1000) << "ms" << std::endl;
            }
            
            // Get detections array
            json detections_array = data.contains("detections") ? 
                                   data["detections"] : data;
            
            std::cout << "   Parsed " << detections_array.size() << " detections" << std::endl;
            
            if (!detections_array.is_array()) {
                std::cout << "   Not an array!" << std::endl;
                //state.frames_since_detection++;
                return BT::NodeStatus::FAILURE;
            }
            
            // Find person closest to center
            double best_dist = 999.0;
            for (const auto& det : detections_array) {
                state.current_detections.push_back(det);
                
                if (det.value("label", "") == "person") {
                    double x = det.value("x", 0.5);
                    double y = det.value("y", 0.5);
                    double dist = std::sqrt(std::pow(x - 0.5, 2) + std::pow(y - 0.5, 2));
                    
                    if (dist < best_dist) {
                        best_dist = dist;
                        state.target_person = det;
                        state.has_person = true;
                    }
                }
            }
            
        } catch (const std::exception& e) {
            flock(fd, LOCK_UN);
            close(fd);
            std::cout << "⚠️  JSON parse error: " << e.what() << std::endl;
            state.frames_since_detection++;
            return BT::NodeStatus::FAILURE;
        }
        
        if (state.has_person) {
            double x = state.target_person.value("x", 0.0);
            double y = state.target_person.value("y", 0.0);
            
            // Update last known position
            //state.last_known_x = x;
            //state.last_known_y = y;
            //state.last_detection_time = std::chrono::steady_clock::now();
            //state.frames_since_detection = 0;

            // Reset smoothing to actual position (crucial!)
            state.smoothed_x = x;
            state.smoothed_y = y;
            
            std::cout << "👤 Person detected at (" << x << ", " << y << ")" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        
        // No person found in current frame
        std::cout << "❌ No person detected" << std::endl;
        state.frames_since_detection++;
        return BT::NodeStatus::FAILURE;
        //return checkLossRecovery();
    }
};

class ProportionalTrackAction : public BT::SyncActionNode {
public:
    ProportionalTrackAction(const std::string& name)
        : BT::SyncActionNode(name, {}) {}
    
    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();
        
        if (!state.has_person || !state.turret) {
            std::cout << "⚠️  No target to track" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        
        // Light smoothing - only apply to REAL detections
        double x = state.target_person.value("x", 0.5);
        double y = state.target_person.value("y", 0.5);
        
        const double alpha = 0.4;  // More responsive
        state.smoothed_x = alpha * x + (1.0 - alpha) * state.smoothed_x;
        state.smoothed_y = alpha * y + (1.0 - alpha) * state.smoothed_y;

        double x_error = -(state.smoothed_x - 0.5);  // Flip for servo direction
        double y_error = -(state.smoothed_y - 0.5);

        // Small, consistent deadband
        const double deadband = 0.025;  // 2.5%
        
        // Check if centered BEFORE applying deadband
        bool is_centered = (std::abs(x_error) < deadband && std::abs(y_error) < deadband);
        
        if (is_centered) {
            state.consecutive_centered_frames++;
            if (state.consecutive_centered_frames >= 5) {  // Reduced from 10
                std::cout << "🔒 CENTERED (hold " << state.consecutive_centered_frames << " frames)\n";
                return BT::NodeStatus::SUCCESS;
            }
        } else {
            state.consecutive_centered_frames = 0;
        }

        // Apply deadband to prevent tiny jitters
        if (std::abs(x_error) < deadband) x_error = 0;
        if (std::abs(y_error) < deadband) y_error = 0;

        // If no movement needed, we're done
        if (x_error == 0 && y_error == 0) {
            return BT::NodeStatus::SUCCESS;
        }

        // Adaptive gains - faster for large errors
        double pan_gain = std::abs(x_error) > 0.1 ? 20.0 : 10.0;
        double tilt_gain = std::abs(y_error) > 0.1 ? 20.0 : 10.0;
        
        double pan_adj = std::clamp(x_error * pan_gain, -15.0, 15.0);
        double tilt_adj = std::clamp(y_error * tilt_gain, -15.0, 15.0);

        double current_pan = state.turret->getPanAngle();
        double current_tilt = state.turret->getTiltAngle();

        double new_pan = std::clamp(current_pan + pan_adj, 10.0, 170.0);
        double new_tilt = std::clamp(current_tilt + tilt_adj, 10.0, 170.0);

        if (new_pan <= 10.0 || new_pan >= 170.0) {
            std::cout << "⚠️  Pan servo at limit!\n";
        }

        state.turret->setPanAngle(new_pan);
        state.turret->setTiltAngle(new_tilt);

        // REDUCED sleep - 10Hz detection means 100ms updates available
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::cout << "🎯 Track: err(" << x_error << ", " << y_error << ") "
                  << "→ pan " << current_pan << "→" << new_pan << "°\n";
        
        return BT::NodeStatus::SUCCESS;
    }    
};

// Action: Simple scanning pattern when no target
class SimpleScanAction : public BT::SyncActionNode {
private:
    std::chrono::steady_clock::time_point last_move_time;
    double move_interval_sec = 1.0; 
    bool scanning_right = true; 
public:
    SimpleScanAction(const std::string& name)
        : BT::SyncActionNode(name, {}),
          last_move_time(std::chrono::steady_clock::now()),
          scanning_right(true) {}
    
    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();
        
        if (!state.turret) {
            return BT::NodeStatus::FAILURE;
        }

        // Check if enough time has passed
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_move_time).count();
        
        if (elapsed < move_interval_sec) {
            return BT::NodeStatus::SUCCESS;
        }
        
        // Time to move
        last_move_time = now;
        
        double current_pan = state.turret->getPanAngle();
        double pan_adj = 0.0;
        
        // Simple back-and-forth scan
        if (current_pan >= 170.0) {
            scanning_right = false;
        } else if (current_pan <= 10.0) {
            scanning_right = true;
        } 
        
        if (scanning_right) {
            pan_adj = 20.0;
            std::cout << "🔍 SCAN: → right" << std::endl;
        } else {
            pan_adj = -20.0;
            std::cout << "🔍 SCAN: ← left" << std::endl;
        }
        
        double new_pan = std::clamp(current_pan + pan_adj, 10.0, 170.0);
        state.turret->setPanAngle(new_pan);
        
        //std::cout << "         Pan: " << current_pan << "° → " << new_pan << "°" << std::endl;
        
        std::cout << new_pan << "°)" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

#endif //BT_NODES_HPP