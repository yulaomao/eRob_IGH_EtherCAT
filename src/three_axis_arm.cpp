#include "three_axis_arm.h"
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ThreeAxisArm::ThreeAxisArm() : initialized_(false) {
    // Initialize default joint configurations
    joint_configs_[BASE] = {-180.0, 180.0, 90.0, 180.0, 0.0};
    joint_configs_[SHOULDER] = {-90.0, 90.0, 60.0, 120.0, 0.0};
    joint_configs_[ELBOW] = {-120.0, 120.0, 90.0, 180.0, 0.0};
}

ThreeAxisArm::~ThreeAxisArm() {
    shutdown();
}

bool ThreeAxisArm::initialize(const MotorConfig& base_config,
                             const MotorConfig& shoulder_config,
                             const MotorConfig& elbow_config) {
    // Initialize motor controller
    if (!motor_controller_.initialize()) {
        std::cerr << "Failed to initialize motor controller: " << motor_controller_.getLastError() << std::endl;
        return false;
    }

    // Add motors
    int base_id = motor_controller_.addMotor(base_config);
    int shoulder_id = motor_controller_.addMotor(shoulder_config);
    int elbow_id = motor_controller_.addMotor(elbow_config);

    if (base_id != BASE || shoulder_id != SHOULDER || elbow_id != ELBOW) {
        std::cerr << "Failed to add motors in correct order" << std::endl;
        return false;
    }

    initialized_ = true;
    return true;
}

void ThreeAxisArm::setJointConfigs(const std::array<JointConfig, 3>& joint_configs) {
    joint_configs_ = joint_configs;
}

bool ThreeAxisArm::start() {
    if (!initialized_) {
        std::cerr << "Arm not initialized" << std::endl;
        return false;
    }

    return motor_controller_.start();
}

void ThreeAxisArm::shutdown() {
    motor_controller_.shutdown();
    initialized_ = false;
}

bool ThreeAxisArm::enable() {
    if (!initialized_) {
        return false;
    }

    return motor_controller_.enableAllMotors();
}

void ThreeAxisArm::disable() {
    motor_controller_.disableAllMotors();
}

bool ThreeAxisArm::homeAll() {
    if (!initialized_) {
        return false;
    }

    bool success = true;
    for (int i = 0; i < 3; ++i) {
        success &= motor_controller_.startHoming(i);
    }

    return success;
}

bool ThreeAxisArm::isHomingComplete() const {
    if (!initialized_) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        if (!motor_controller_.isHomingComplete(i)) {
            return false;
        }
    }

    return true;
}

bool ThreeAxisArm::moveToJointAngles(const JointAngles& angles, 
                                    double velocity, 
                                    double acceleration) {
    if (!initialized_) {
        return false;
    }

    // Check joint limits
    if (!checkJointLimits(angles)) {
        std::cerr << "Joint angles exceed limits" << std::endl;
        return false;
    }

    // Convert angles and send commands
    bool success = true;
    
    double base_pos = degreesToMotorUnits(BASE, angles.base);
    double shoulder_pos = degreesToMotorUnits(SHOULDER, angles.shoulder);
    double elbow_pos = degreesToMotorUnits(ELBOW, angles.elbow);

    double vel = (velocity > 0) ? velocity : joint_configs_[BASE].max_velocity;
    double accel = (acceleration > 0) ? acceleration : joint_configs_[BASE].max_acceleration;

    success &= motor_controller_.moveAbsolute(BASE, base_pos, vel, accel);
    success &= motor_controller_.moveAbsolute(SHOULDER, shoulder_pos, vel, accel);
    success &= motor_controller_.moveAbsolute(ELBOW, elbow_pos, vel, accel);

    return success;
}

bool ThreeAxisArm::moveJoint(Joint joint, double angle, 
                            double velocity, 
                            double acceleration) {
    if (!initialized_) {
        return false;
    }

    // Check joint limits
    const JointConfig& config = joint_configs_[joint];
    if (angle < config.min_angle || angle > config.max_angle) {
        std::cerr << "Joint angle " << angle << " exceeds limits [" 
                  << config.min_angle << ", " << config.max_angle << "]" << std::endl;
        return false;
    }

    double position = degreesToMotorUnits(joint, angle);
    double vel = (velocity > 0) ? velocity : config.max_velocity;
    double accel = (acceleration > 0) ? acceleration : config.max_acceleration;

    return motor_controller_.moveAbsolute(static_cast<int>(joint), position, vel, accel);
}

bool ThreeAxisArm::moveToCartesian(const CartesianPos& position,
                                  double velocity,
                                  double acceleration) {
    if (!initialized_) {
        return false;
    }

    // Calculate inverse kinematics
    JointAngles target_angles;
    if (!inverseKinematics(position, target_angles)) {
        std::cerr << "Cartesian position not reachable" << std::endl;
        return false;
    }

    return moveToJointAngles(target_angles, velocity, acceleration);
}

bool ThreeAxisArm::enableServoMode() {
    if (!initialized_) {
        return false;
    }

    bool success = true;
    for (int i = 0; i < 3; ++i) {
        success &= motor_controller_.enableServo(i);
    }

    return success;
}

void ThreeAxisArm::disableServoMode() {
    for (int i = 0; i < 3; ++i) {
        motor_controller_.disableServo(i);
    }
}

bool ThreeAxisArm::setServoTargetAngles(const JointAngles& angles) {
    if (!initialized_) {
        return false;
    }

    // Check joint limits
    if (!checkJointLimits(angles)) {
        std::cerr << "Joint angles exceed limits" << std::endl;
        return false;
    }

    bool success = true;
    
    double base_pos = degreesToMotorUnits(BASE, angles.base);
    double shoulder_pos = degreesToMotorUnits(SHOULDER, angles.shoulder);
    double elbow_pos = degreesToMotorUnits(ELBOW, angles.elbow);

    success &= motor_controller_.setServoTarget(BASE, base_pos);
    success &= motor_controller_.setServoTarget(SHOULDER, shoulder_pos);
    success &= motor_controller_.setServoTarget(ELBOW, elbow_pos);

    return success;
}

bool ThreeAxisArm::setServoTargetCartesian(const CartesianPos& position) {
    if (!initialized_) {
        return false;
    }

    // Calculate inverse kinematics
    JointAngles target_angles;
    if (!inverseKinematics(position, target_angles)) {
        std::cerr << "Cartesian position not reachable" << std::endl;
        return false;
    }

    return setServoTargetAngles(target_angles);
}

ThreeAxisArm::JointAngles ThreeAxisArm::getCurrentJointAngles() const {
    JointAngles angles = {0, 0, 0};
    
    if (initialized_) {
        angles.base = motorUnitsToDegrees(BASE, motor_controller_.getPosition(BASE));
        angles.shoulder = motorUnitsToDegrees(SHOULDER, motor_controller_.getPosition(SHOULDER));
        angles.elbow = motorUnitsToDegrees(ELBOW, motor_controller_.getPosition(ELBOW));
    }

    return angles;
}

ThreeAxisArm::CartesianPos ThreeAxisArm::getCurrentCartesianPos() const {
    JointAngles angles = getCurrentJointAngles();
    return forwardKinematics(angles);
}

bool ThreeAxisArm::isReady() const {
    if (!initialized_) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        if (!motor_controller_.isMotorReady(i)) {
            return false;
        }
    }

    return true;
}

bool ThreeAxisArm::isInPosition(double tolerance) const {
    if (!initialized_) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        if (!motor_controller_.isInPosition(i, tolerance)) {
            return false;
        }
    }

    return true;
}

void ThreeAxisArm::emergencyStop() {
    motor_controller_.emergencyStop();
}

bool ThreeAxisArm::clearErrors() {
    if (!initialized_) {
        return false;
    }

    bool success = true;
    for (int i = 0; i < 3; ++i) {
        success &= motor_controller_.clearErrors(i);
    }

    return success;
}

ThreeAxisArm::CartesianPos ThreeAxisArm::forwardKinematics(const JointAngles& angles) const {
    // Convert degrees to radians
    double theta1 = angles.base * M_PI / 180.0;
    double theta2 = angles.shoulder * M_PI / 180.0;
    double theta3 = angles.elbow * M_PI / 180.0;

    // Forward kinematics for a 3-DOF planar arm (simplified)
    // Assuming shoulder and elbow move in the same vertical plane
    double x = cos(theta1) * (L2 * cos(theta2) + L3 * cos(theta2 + theta3));
    double y = sin(theta1) * (L2 * cos(theta2) + L3 * cos(theta2 + theta3));
    double z = L1 + L2 * sin(theta2) + L3 * sin(theta2 + theta3);

    return {x, y, z};
}

bool ThreeAxisArm::inverseKinematics(const CartesianPos& position, JointAngles& angles) const {
    double x = position.x;
    double y = position.y;
    double z = position.z - L1;  // Remove base height

    // Base angle (rotation around Z axis)
    angles.base = atan2(y, x) * 180.0 / M_PI;

    // Project to 2D problem in the vertical plane
    double r = sqrt(x*x + y*y);  // Horizontal distance from base
    
    // Check if position is reachable
    double max_reach = L2 + L3;
    double min_reach = fabs(L2 - L3);
    double target_distance = sqrt(r*r + z*z);
    
    if (target_distance > max_reach || target_distance < min_reach) {
        return false;  // Position not reachable
    }

    // Use cosine law to find elbow angle
    double cos_elbow = (L2*L2 + L3*L3 - target_distance*target_distance) / (2*L2*L3);
    
    // Check if solution exists
    if (cos_elbow < -1.0 || cos_elbow > 1.0) {
        return false;
    }
    
    angles.elbow = acos(cos_elbow) * 180.0 / M_PI;
    
    // Calculate shoulder angle
    double alpha = atan2(z, r);
    double beta = acos((L2*L2 + target_distance*target_distance - L3*L3) / (2*L2*target_distance));
    angles.shoulder = (alpha + beta) * 180.0 / M_PI;

    // Check joint limits
    return checkJointLimits(angles);
}

bool ThreeAxisArm::checkJointLimits(const JointAngles& angles) const {
    return (angles.base >= joint_configs_[BASE].min_angle && 
            angles.base <= joint_configs_[BASE].max_angle &&
            angles.shoulder >= joint_configs_[SHOULDER].min_angle &&
            angles.shoulder <= joint_configs_[SHOULDER].max_angle &&
            angles.elbow >= joint_configs_[ELBOW].min_angle &&
            angles.elbow <= joint_configs_[ELBOW].max_angle);
}

double ThreeAxisArm::degreesToMotorUnits(Joint joint, double degrees) const {
    // For demonstration, assume 1 degree = 1 motor unit
    // In real implementation, this would depend on gear ratios and encoder resolution
    return degrees;
}

double ThreeAxisArm::motorUnitsToDegrees(Joint joint, double motor_units) const {
    // For demonstration, assume 1 motor unit = 1 degree
    // In real implementation, this would depend on gear ratios and encoder resolution
    return motor_units;
}