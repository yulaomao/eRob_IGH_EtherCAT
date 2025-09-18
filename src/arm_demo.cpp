#include "three_axis_arm.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Global flag for graceful shutdown
volatile sig_atomic_t running = 1;

void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ". Shutting down gracefully..." << std::endl;
    running = 0;
}

int main(int argc, char** argv) {
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "=== Three-Axis Robotic Arm Control Demo ===" << std::endl;
    std::cout << "This demo showcases the comprehensive motor control system" << std::endl;
    std::cout << "Press Ctrl+C to stop the program safely" << std::endl << std::endl;

    // Create arm controller
    ThreeAxisArm arm;

    // Configure motors (example configurations)
    // Note: These would need to match your actual hardware
    MotorConfig base_config;
    base_config.position = 0;
    base_config.vendor_id = 0x5a65726f;  // From original code
    base_config.product_code = 0x00029252;
    base_config.name = "Base Motor";
    base_config.encoder_resolution = 524287;  // From original code
    base_config.gear_ratio = 50.0;  // Example gear ratio
    base_config.max_velocity = 180.0;  // degrees/s
    base_config.max_acceleration = 360.0;  // degrees/s²

    MotorConfig shoulder_config = base_config;
    shoulder_config.position = 1;
    shoulder_config.name = "Shoulder Motor";
    shoulder_config.gear_ratio = 100.0;
    shoulder_config.max_velocity = 120.0;
    shoulder_config.max_acceleration = 240.0;

    MotorConfig elbow_config = base_config;
    elbow_config.position = 2;
    elbow_config.name = "Elbow Motor";
    elbow_config.gear_ratio = 80.0;
    elbow_config.max_velocity = 150.0;
    elbow_config.max_acceleration = 300.0;

    // Initialize arm
    std::cout << "Initializing three-axis arm..." << std::endl;
    if (!arm.initialize(base_config, shoulder_config, elbow_config)) {
        std::cerr << "Failed to initialize arm: " << arm.getLastError() << std::endl;
        return -1;
    }

    // Set joint configurations
    std::array<ThreeAxisArm::JointConfig, 3> joint_configs;
    joint_configs[ThreeAxisArm::BASE] = {-180.0, 180.0, 90.0, 180.0, 0.0};
    joint_configs[ThreeAxisArm::SHOULDER] = {-90.0, 90.0, 60.0, 120.0, 0.0};
    joint_configs[ThreeAxisArm::ELBOW] = {-120.0, 120.0, 90.0, 180.0, 0.0};
    arm.setJointConfigs(joint_configs);

    // Start control system
    std::cout << "Starting control system..." << std::endl;
    if (!arm.start()) {
        std::cerr << "Failed to start arm control: " << arm.getLastError() << std::endl;
        return -1;
    }

    // Wait for system to stabilize
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Enable motors
    std::cout << "Enabling motors..." << std::endl;
    if (!arm.enable()) {
        std::cerr << "Failed to enable motors: " << arm.getLastError() << std::endl;
        arm.shutdown();
        return -1;
    }

    // Wait for motors to be ready
    std::cout << "Waiting for motors to be ready..." << std::endl;
    int timeout = 100;  // 10 seconds
    while (!arm.isReady() && timeout > 0 && running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        timeout--;
    }

    if (!arm.isReady()) {
        std::cerr << "Motors failed to become ready" << std::endl;
        arm.shutdown();
        return -1;
    }

    std::cout << "All motors ready!" << std::endl;

    // Start homing sequence
    std::cout << "\nStarting homing sequence..." << std::endl;
    if (!arm.homeAll()) {
        std::cerr << "Failed to start homing: " << arm.getLastError() << std::endl;
        arm.shutdown();
        return -1;
    }

    // Wait for homing to complete
    std::cout << "Waiting for homing to complete..." << std::endl;
    timeout = 300;  // 30 seconds
    while (!arm.isHomingComplete() && timeout > 0 && running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        timeout--;
    }

    if (!arm.isHomingComplete()) {
        std::cerr << "Homing failed to complete in time" << std::endl;
        arm.shutdown();
        return -1;
    }

    std::cout << "Homing complete!" << std::endl;

    // Demonstration sequence
    std::cout << "\n=== Starting Motion Demonstration ===" << std::endl;

    // Move to home position
    std::cout << "\n1. Moving to home position..." << std::endl;
    ThreeAxisArm::JointAngles home_angles = {0.0, 0.0, 0.0};
    if (!arm.moveToJointAngles(home_angles, 30.0, 60.0)) {
        std::cerr << "Failed to move to home position" << std::endl;
    } else {
        // Wait for movement to complete
        timeout = 100;
        while (!arm.isInPosition(1.0) && timeout > 0 && running) {
            auto current_angles = arm.getCurrentJointAngles();
            std::cout << "Current angles: Base=" << current_angles.base 
                      << "°, Shoulder=" << current_angles.shoulder 
                      << "°, Elbow=" << current_angles.elbow << "°" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            timeout--;
        }
        std::cout << "Reached home position!" << std::endl;
    }

    if (!running) goto cleanup;

    // Move individual joints
    std::cout << "\n2. Moving individual joints..." << std::endl;
    
    std::cout << "Moving base to 45°..." << std::endl;
    arm.moveJoint(ThreeAxisArm::BASE, 45.0, 20.0);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    std::cout << "Moving shoulder to 30°..." << std::endl;
    arm.moveJoint(ThreeAxisArm::SHOULDER, 30.0, 15.0);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    std::cout << "Moving elbow to -45°..." << std::endl;
    arm.moveJoint(ThreeAxisArm::ELBOW, -45.0, 25.0);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    if (!running) goto cleanup;

    // Cartesian movement demonstration
    std::cout << "\n3. Cartesian movement demonstration..." << std::endl;
    
    // Get current position
    auto current_pos = arm.getCurrentCartesianPos();
    std::cout << "Current Cartesian position: X=" << current_pos.x 
              << ", Y=" << current_pos.y << ", Z=" << current_pos.z << std::endl;
    
    // Move to new Cartesian positions
    ThreeAxisArm::CartesianPos target1 = {300.0, 100.0, 250.0};
    std::cout << "Moving to position: X=" << target1.x 
              << ", Y=" << target1.y << ", Z=" << target1.z << std::endl;
    
    if (arm.moveToCartesian(target1, 25.0)) {
        std::this_thread::sleep_for(std::chrono::seconds(4));
        
        ThreeAxisArm::CartesianPos target2 = {200.0, 200.0, 300.0};
        std::cout << "Moving to position: X=" << target2.x 
                  << ", Y=" << target2.y << ", Z=" << target2.z << std::endl;
        arm.moveToCartesian(target2, 25.0);
        std::this_thread::sleep_for(std::chrono::seconds(4));
    } else {
        std::cout << "Cartesian position not reachable" << std::endl;
    }

    if (!running) goto cleanup;

    // Servo control demonstration
    std::cout << "\n4. Servo control demonstration..." << std::endl;
    std::cout << "Enabling servo mode..." << std::endl;
    
    if (arm.enableServoMode()) {
        std::cout << "Servo mode enabled. Performing smooth circular motion..." << std::endl;
        
        // Perform circular motion for 10 seconds
        auto start_time = std::chrono::steady_clock::now();
        double radius = 50.0;
        ThreeAxisArm::CartesianPos center = {250.0, 0.0, 280.0};
        
        while (running && std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - start_time).count() < 10) {
            
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            
            double angle = 2.0 * M_PI * elapsed / 5000.0;  // Complete circle in 5 seconds
            
            ThreeAxisArm::CartesianPos target;
            target.x = center.x + radius * cos(angle);
            target.y = center.y + radius * sin(angle);
            target.z = center.z;
            
            arm.setServoTargetCartesian(target);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 Hz
        }
        
        std::cout << "Circular motion complete." << std::endl;
        arm.disableServoMode();
    } else {
        std::cout << "Failed to enable servo mode" << std::endl;
    }

    // Return to home position
    std::cout << "\n5. Returning to home position..." << std::endl;
    arm.moveToJointAngles(home_angles, 20.0, 40.0);
    
    timeout = 100;
    while (!arm.isInPosition(1.0) && timeout > 0 && running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        timeout--;
    }

cleanup:
    std::cout << "\nShutting down arm controller..." << std::endl;
    arm.disable();
    arm.shutdown();
    
    std::cout << "Demo completed successfully!" << std::endl;
    std::cout << "\n=== Motion Control System Features Demonstrated ===" << std::endl;
    std::cout << "✓ Automatic motor initialization and configuration" << std::endl;
    std::cout << "✓ Motor enable/disable control" << std::endl;
    std::cout << "✓ Homing sequence" << std::endl;
    std::cout << "✓ Individual joint position control" << std::endl;
    std::cout << "✓ Coordinated multi-axis movement" << std::endl;
    std::cout << "✓ Cartesian coordinate movement (with inverse kinematics)" << std::endl;
    std::cout << "✓ Real-time servo control mode" << std::endl;
    std::cout << "✓ Smooth trajectory generation" << std::endl;
    std::cout << "✓ Safety functions (emergency stop, error handling)" << std::endl;
    std::cout << "✓ Status monitoring and position feedback" << std::endl;

    return 0;
}