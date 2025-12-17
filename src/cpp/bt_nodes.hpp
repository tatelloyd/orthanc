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

struct TrackerState {
    std::string detection_file = "detections.json";
    std::vector<json> current_detections;
    json target_person;
    bool has_person = false;
    double smoothed_x = 0.5;
    double smoothed_y = 0.5;

    int frames_since_detection = 0;
    std::chrono::steady_clock::time_point last_detection_time;
    const double loss_timeout_seconds = 2.0;

    int consecutive_centered_frames = 0;
    const int frames_needed_to_lock = 5;
    
    bool in_recovery_mode = false;
    double last_known_x = 0.5;
    double last_known_y = 0.5;

    Turret* turret = nullptr;
    
    static TrackerState& get() {
        static TrackerState instance;
        return instance;
    }
};

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
            
            state.last_known_x = x;
            state.last_known_y = y;
            state.frames_since_detection = 0;
            state.in_recovery_mode = false;

            state.smoothed_x = x;
            state.smoothed_y = y;
            
            std::cout << "👤 Person detected at (" << x << ", " << y << ")" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        
        std::cout << "❌ No person detected (frames lost: " << state.frames_since_detection << ")" << std::endl;
        state.frames_since_detection++;
        
        if (state.frames_since_detection <= 10) {  // Increased from 3 - hold position longer
            std::cout << "🔄 Recovery mode - holding position" << std::endl;
            state.in_recovery_mode = true;
            return BT::NodeStatus::SUCCESS;
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
            
            // CRITICAL: Reject detections that jump too far (servo lag artifact)
            double jump_dist = std::sqrt(
                std::pow(target_x - state.smoothed_x, 2) + 
                std::pow(target_y - state.smoothed_y, 2)
            );
            
            if (jump_dist > 0.3) {  // If detection jumps >30% of frame
                std::cout << "⚠️  Detection jump too large (" << jump_dist << "), using smoothed position" << std::endl;
                target_x = state.smoothed_x;
                target_y = state.smoothed_y;
            }
        }
        
        // MUCH heavier smoothing to compensate for jerky YOLO detections
        const double alpha = 0.2;  // Reduced from 0.5 for more stability
        state.smoothed_x = alpha * target_x + (1.0 - alpha) * state.smoothed_x;
        state.smoothed_y = alpha * target_y + (1.0 - alpha) * state.smoothed_y;

        // FIXED: Inverted x-axis
        // When person is at x=0.9 (right side of frame), we need to pan LEFT (decrease angle)
        // When person is at x=0.1 (left side of frame), we need to pan RIGHT (increase angle)
        double x_error = -(state.smoothed_x - 0.5);  // INVERTED
        double y_error = -(state.smoothed_y - 0.5);  // Inverted for upside-down camera

        const double deadband = 0.08;  // Much larger to avoid oscillation
        
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

        if (std::abs(x_error) < deadband) x_error = 0;
        if (std::abs(y_error) < deadband) y_error = 0;

        if (x_error == 0 && y_error == 0) {
            return BT::NodeStatus::SUCCESS;
        }

        // Even gentler gains to prevent overcorrection
        // Use VERY small gains since we're waiting longer for servos
        double pan_gain = std::abs(x_error) > 0.2 ? 12.0 :   // Reduced from 15
                         (std::abs(x_error) > 0.1 ? 6.0 :    // Reduced from 8
                          3.0);                               // Reduced from 4
        double tilt_gain = std::abs(y_error) > 0.2 ? 12.0 : 
                          (std::abs(y_error) > 0.1 ? 6.0 : 
                           3.0);
        
        double pan_adj = std::clamp(x_error * pan_gain, -4.0, 4.0);   // Even smaller max step
        double tilt_adj = std::clamp(y_error * tilt_gain, -4.0, 4.0); // Even smaller max step

        double current_pan = state.turret->getPanAngle();
        double current_tilt = state.turret->getTiltAngle();

        double new_pan = std::clamp(current_pan + pan_adj, 10.0, 170.0);
        double new_tilt = std::clamp(current_tilt + tilt_adj, 10.0, 170.0);

        if (new_pan <= 10.0 || new_pan >= 170.0) {
            std::cout << "⚠️  Pan servo at limit! Person may be out of range.\n";
        }

        state.turret->setPanAngle(new_pan);
        state.turret->setTiltAngle(new_tilt);

        // CRITICAL: Wait for servos to actually reach position before next detection
        // Scale wait time based on how much we moved - bigger moves need more settling time
        double total_movement = std::abs(pan_adj) + std::abs(tilt_adj);
        int wait_ms = std::min(300, static_cast<int>(100 + total_movement * 20));
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
        
        std::cout << "🎯 Track: target(" << target_x << ", " << target_y << ") "
                  << "err(" << x_error << ", " << y_error << ") "
                  << "→ pan " << current_pan << "→" << new_pan << "°\n";
        
        return BT::NodeStatus::SUCCESS;
    }    
};

class SimpleScanAction : public BT::SyncActionNode {
private:
    std::chrono::steady_clock::time_point last_move_time;
    double move_interval_sec = 2.0;
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
        
        if (current_pan >= 150.0) {
            scanning_right = false;
        } else if (current_pan <= 30.0) {
            scanning_right = true;
        } 
        
        if (scanning_right) {
            pan_adj = 15.0;
            std::cout << "🔍 SCAN: → right (+15°)" << std::endl;
        } else {
            pan_adj = -15.0;
            std::cout << "🔍 SCAN: ← left (-15°)" << std::endl;
        }
        
        double new_pan = std::clamp(current_pan + pan_adj, 10.0, 170.0);
        state.turret->setPanAngle(new_pan);
        
        std::cout << "         Pan: " << current_pan << "° → " << new_pan << "°" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

#endif //BT_NODES_HPP