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
    const double loss_timeout_seconds = 2.0;

    // Tracking state
    int consecutive_centered_frames = 0;
    const int frames_needed_to_lock = 5;
    
    // Recovery mode
    bool in_recovery_mode = false;
    double last_known_x = 0.5;
    double last_known_y = 0.5;

    Turret* turret = nullptr;
    
    static TrackerState& get() {
        static TrackerState instance;
        return instance;
    }
};

// Condition: Check if we have a person in detections
class HasPersonDetection : public BT::ConditionNode {
public:
    HasPersonDetection(const std::string& name) 
        : BT::ConditionNode(name, {}) {}
    
    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();
        state.has_person = false;
        state.current_detections.clear();

        std::cout << "🔍 Reading: " << state.detection_file << std::endl;
        
        int fd = open(state.detection_file.c_str(), O_RDONLY);
        if (fd < 0) {
            std::cout << "⚠️  Cannot open " << state.detection_file << std::endl;
            state.frames_since_detection++;
            return BT::NodeStatus::FAILURE;
        }
        
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
            
            json detections_array = data.contains("detections") ? 
                                   data["detections"] : data;
            
            std::cout << "   Parsed " << detections_array.size() << " detections" << std::endl;
            
            if (!detections_array.is_array()) {
                std::cout << "   Not an array!" << std::endl;
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
            
            // Store last known position for recovery
            state.last_known_x = x;
            state.last_known_y = y;
            state.frames_since_detection = 0;
            state.in_recovery_mode = false;

            // Reset smoothing to actual position
            state.smoothed_x = x;
            state.smoothed_y = y;
            
            std::cout << "👤 Person detected at (" << x << ", " << y << ")" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        
        // No person found - enter recovery mode
        std::cout << "❌ No person detected (frames lost: " << state.frames_since_detection << ")" << std::endl;
        state.frames_since_detection++;
        
        // Allow brief recovery period
        if (state.frames_since_detection <= 5) {
            std::cout << "🔄 Recovery mode - holding position" << std::endl;
            state.in_recovery_mode = true;
            return BT::NodeStatus::SUCCESS;  // Pretend we have person for recovery tracking
        }
        
        return BT::NodeStatus::FAILURE;
    }
};

class ProportionalTrackAction : public BT::SyncActionNode {
public:
    ProportionalTrackAction(const std::string& name)
        : BT::SyncActionNode(name, {}) {}
    
    BT::NodeStatus tick() override {
        auto& state = TrackerState::get();
        
        if (!state.turret) {
            std::cout << "⚠️  No turret available" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        
        // In recovery mode, use last known position
        double target_x, target_y;
        if (state.in_recovery_mode) {
            target_x = state.last_known_x;
            target_y = state.last_known_y;
            std::cout << "🔄 Using last known position: (" << target_x << ", " << target_y << ")" << std::endl;
        } else if (!state.has_person) {
            std::cout << "⚠️  No target to track" << std::endl;
            return BT::NodeStatus::FAILURE;
        } else {
            target_x = state.target_person.value("x", 0.5);
            target_y = state.target_person.value("y", 0.5);
        }
        
        // Light smoothing
        const double alpha = 0.4;
        state.smoothed_x = alpha * target_x + (1.0 - alpha) * state.smoothed_x;
        state.smoothed_y = alpha * target_y + (1.0 - alpha) * state.smoothed_y;

        // CORRECTED: Remove the negative sign - error should point toward target
        double x_error = (state.smoothed_x - 0.5);  // Positive = target is right, move right
        double y_error = -(state.smoothed_y - 0.5);  // Positive = target is down, move down

        const double deadband = 0.025;
        
        bool is_centered = (std::abs(x_error) < deadband && std::abs(y_error) < deadband);
        
        if (is_centered && !state.in_recovery_mode) {
            state.consecutive_centered_frames++;
            if (state.consecutive_centered_frames >= state.frames_needed_to_lock) {
                std::cout << "🔒 CENTERED (hold " << state.consecutive_centered_frames << " frames)\n";
                return BT::NodeStatus::SUCCESS;
            }
        } else {
            state.consecutive_centered_frames = 0;
        }

        // Apply deadband
        if (std::abs(x_error) < deadband) x_error = 0;
        if (std::abs(y_error) < deadband) y_error = 0;

        if (x_error == 0 && y_error == 0) {
            return BT::NodeStatus::SUCCESS;
        }

        // REDUCED gains for smoother tracking
        double pan_gain = std::abs(x_error) > 0.15 ? 15.0 : 8.0;   // Reduced from 20/10
        double tilt_gain = std::abs(y_error) > 0.15 ? 15.0 : 8.0;
        
        double pan_adj = std::clamp(x_error * pan_gain, -10.0, 10.0);   // Reduced max from 15
        double tilt_adj = std::clamp(y_error * tilt_gain, -10.0, 10.0);

        double current_pan = state.turret->getPanAngle();
        double current_tilt = state.turret->getTiltAngle();

        double new_pan = std::clamp(current_pan + pan_adj, 10.0, 170.0);
        double new_tilt = std::clamp(current_tilt + tilt_adj, 10.0, 170.0);

        if (new_pan <= 10.0 || new_pan >= 170.0) {
            std::cout << "⚠️  Pan servo at limit!\n";
        }

        state.turret->setPanAngle(new_pan);
        state.turret->setTiltAngle(new_tilt);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::cout << "🎯 Track: err(" << x_error << ", " << y_error << ") "
                  << "→ pan " << current_pan << "→" << new_pan << "°"
                  << " | tilt " << current_tilt << "→" << new_tilt << "°\n";
        
        return BT::NodeStatus::SUCCESS;
    }    
};

// Action: Gentle scanning when target is lost
class SimpleScanAction : public BT::SyncActionNode {
private:
    std::chrono::steady_clock::time_point last_move_time;
    double move_interval_sec = 1.5;  // Slower scanning
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

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_move_time).count();
        
        if (elapsed < move_interval_sec) {
            return BT::NodeStatus::SUCCESS;
        }
        
        last_move_time = now;
        
        double current_pan = state.turret->getPanAngle();
        double pan_adj = 0.0;
        
        // Gentler scanning - smaller steps
        if (current_pan >= 160.0) {
            scanning_right = false;
        } else if (current_pan <= 20.0) {
            scanning_right = true;
        } 
        
        if (scanning_right) {
            pan_adj = 10.0;  // Reduced from 20
            std::cout << "🔍 SCAN: → right (+10°)" << std::endl;
        } else {
            pan_adj = -10.0;
            std::cout << "🔍 SCAN: ← left (-10°)" << std::endl;
        }
        
        double new_pan = std::clamp(current_pan + pan_adj, 10.0, 170.0);
        state.turret->setPanAngle(new_pan);
        
        std::cout << "         Pan: " << current_pan << "° → " << new_pan << "°" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

#endif //BT_NODES_HPP