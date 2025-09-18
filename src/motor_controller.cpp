#include "motor_controller.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <cstring>

// EtherCAT constants from original driver
#define NSEC_PER_SEC (1000000000L)
#define ENCODER_RES 524287

// EtherCAT macros for reading/writing PDO data
#define EC_READ_U16(DATA) *(reinterpret_cast<const uint16_t*>(DATA))
#define EC_READ_S32(DATA) *(reinterpret_cast<const int32_t*>(DATA))
#define EC_READ_S16(DATA) *(reinterpret_cast<const int16_t*>(DATA))
#define EC_WRITE_U16(DATA, VAL) *(reinterpret_cast<uint16_t*>(DATA)) = (VAL)
#define EC_WRITE_S32(DATA, VAL) *(reinterpret_cast<int32_t*>(DATA)) = (VAL)

// CiA402 control words
#define CONTROL_WORD_SHUTDOWN           0x0006
#define CONTROL_WORD_SWITCH_ON         0x0007
#define CONTROL_WORD_ENABLE_OPERATION  0x000F
#define CONTROL_WORD_FAULT_RESET       0x0080
#define CONTROL_WORD_QUICK_STOP        0x0002

// Operation modes
#define OPERATION_MODE_PP   1  // Profile Position
#define OPERATION_MODE_PV   3  // Profile Velocity  
#define OPERATION_MODE_CST  10 // Cyclic Synchronous Torque
#define OPERATION_MODE_CSV  9  // Cyclic Synchronous Velocity
#define OPERATION_MODE_CSP  8  // Cyclic Synchronous Position

MotorController::MotorController() 
    : master_(nullptr), domain_(nullptr), domain_data_(nullptr),
      running_(false), frequency_hz_(1000), cycle_time_ns_(NSEC_PER_SEC / 1000) {
}

MotorController::~MotorController() {
    shutdown();
}

bool MotorController::initialize(const std::string& interface_name) {
    try {
        // Create EtherCAT master
        master_ = ecrt_request_master(0);
        if (!master_) {
            setError("Failed to request EtherCAT master");
            return false;
        }

        // Create domain
        domain_ = ecrt_master_create_domain(master_);
        if (!domain_) {
            setError("Failed to create EtherCAT domain");
            return false;
        }

        last_error_.clear();
        return true;
    }
    catch (const std::exception& e) {
        setError("Exception during initialization: " + std::string(e.what()));
        return false;
    }
}

int MotorController::addMotor(const MotorConfig& config) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (!master_) {
        setError("Master not initialized");
        return -1;
    }

    Motor motor;
    motor.config = config;
    
    // Create slave configuration
    motor.slave_config = ecrt_master_slave_config(master_, 0, config.position, 
                                                  config.vendor_id, config.product_code);
    if (!motor.slave_config) {
        setError("Failed to get slave configuration for motor at position " + std::to_string(config.position));
        return -1;
    }

    motors_.push_back(motor);
    return static_cast<int>(motors_.size() - 1);
}

bool MotorController::configurePDOs() {
    if (motors_.empty()) {
        setError("No motors configured");
        return false;
    }

    // Create PDO registration array
    std::vector<ec_pdo_entry_reg_t> pdo_entries;
    
    for (size_t i = 0; i < motors_.size(); ++i) {
        Motor& motor = motors_[i];
        const auto& config = motor.config;
        
        // Control word
        pdo_entries.push_back({0, config.position, config.vendor_id, config.product_code, 
                              0x6040, 0x00, &motor.control_word_offset});
        
        // Status word
        pdo_entries.push_back({0, config.position, config.vendor_id, config.product_code,
                              0x6041, 0x00, &motor.status_word_offset});
        
        // Target position
        pdo_entries.push_back({0, config.position, config.vendor_id, config.product_code,
                              0x607A, 0x00, &motor.target_position_offset});
        
        // Actual position
        pdo_entries.push_back({0, config.position, config.vendor_id, config.product_code,
                              0x6064, 0x00, &motor.actual_position_offset});
        
        // Target velocity
        pdo_entries.push_back({0, config.position, config.vendor_id, config.product_code,
                              0x60FF, 0x00, &motor.target_velocity_offset});
        
        // Actual velocity
        pdo_entries.push_back({0, config.position, config.vendor_id, config.product_code,
                              0x606C, 0x00, &motor.actual_velocity_offset});
        
        // Target torque
        pdo_entries.push_back({0, config.position, config.vendor_id, config.product_code,
                              0x6071, 0x00, &motor.target_torque_offset});
        
        // Actual torque
        pdo_entries.push_back({0, config.position, config.vendor_id, config.product_code,
                              0x6077, 0x00, &motor.actual_torque_offset});
    }
    
    // Terminate array
    pdo_entries.push_back({});
    
    // Register PDO entries
    if (ecrt_domain_reg_pdo_entry_list(domain_, pdo_entries.data())) {
        setError("Failed to register PDO entries");
        return false;
    }

    return true;
}

bool MotorController::activateMaster() {
    if (ecrt_master_activate(master_)) {
        setError("Failed to activate EtherCAT master");
        return false;
    }

    domain_data_ = ecrt_domain_data(domain_);
    if (!domain_data_) {
        setError("Failed to get domain data pointer");
        return false;
    }

    return true;
}

bool MotorController::start(int frequency_hz) {
    if (running_) {
        setError("Controller already running");
        return false;
    }

    frequency_hz_ = frequency_hz;
    cycle_time_ns_ = NSEC_PER_SEC / frequency_hz_;

    // Configure PDOs
    if (!configurePDOs()) {
        return false;
    }

    // Activate master
    if (!activateMaster()) {
        return false;
    }

    // Start control loop
    running_ = true;
    control_thread_ = std::thread(&MotorController::controlLoop, this);

    return true;
}

void MotorController::shutdown() {
    running_ = false;
    
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
    }
    
    domain_ = nullptr;
    domain_data_ = nullptr;
}

bool MotorController::enableMotor(int motor_id) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    motor.status.is_enabled = true;
    
    return true;
}

bool MotorController::disableMotor(int motor_id) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    motor.status.is_enabled = false;
    motor.servo_enabled = false;
    motor.motion_active = false;
    
    return true;
}

bool MotorController::enableAllMotors() {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    for (auto& motor : motors_) {
        motor.status.is_enabled = true;
    }
    
    return true;
}

void MotorController::disableAllMotors() {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    for (auto& motor : motors_) {
        motor.status.is_enabled = false;
        motor.servo_enabled = false;
        motor.motion_active = false;
    }
}

bool MotorController::moveAbsolute(int motor_id, double position, double velocity, double acceleration) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    
    if (!motor.status.is_enabled) {
        setError("Motor not enabled");
        return false;
    }

    // Set motion parameters
    motor.current_motion.target_position = position;
    motor.current_motion.max_velocity = (velocity > 0) ? velocity : motor.config.max_velocity;
    motor.current_motion.acceleration = (acceleration > 0) ? acceleration : motor.config.max_acceleration;
    motor.current_motion.deceleration = motor.current_motion.acceleration;
    motor.current_motion.relative = false;
    
    // Start motion
    motor.motion_active = true;
    motor.motion_start_position = encoderToUser(motor_id, motor.status.position);
    motor.motion_start_time = getCurrentTimeNs();
    motor.control_mode = ControlMode::POSITION;
    motor.servo_enabled = false;

    return true;
}

bool MotorController::moveRelative(int motor_id, double distance, double velocity, double acceleration) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    
    if (!motor.status.is_enabled) {
        setError("Motor not enabled");
        return false;
    }

    double current_pos = encoderToUser(motor_id, motor.status.position);
    return moveAbsolute(motor_id, current_pos + distance, velocity, acceleration);
}

bool MotorController::setVelocity(int motor_id, double velocity) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    
    if (!motor.status.is_enabled) {
        setError("Motor not enabled");
        return false;
    }

    motor.control_mode = ControlMode::VELOCITY;
    motor.servo_enabled = false;
    motor.motion_active = false;
    
    // Velocity will be set in the control loop
    motor.current_motion.max_velocity = velocity;

    return true;
}

bool MotorController::stopMotor(int motor_id, double deceleration) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    motor.motion_active = false;
    motor.servo_enabled = false;
    
    // Set velocity to zero
    motor.current_motion.max_velocity = 0;
    motor.control_mode = ControlMode::VELOCITY;

    return true;
}

bool MotorController::enableServo(int motor_id) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    
    if (!motor.status.is_enabled) {
        setError("Motor not enabled");
        return false;
    }

    motor.servo_enabled = true;
    motor.motion_active = false;
    motor.control_mode = ControlMode::POSITION;
    motor.servo_target_position = encoderToUser(motor_id, motor.status.position);

    return true;
}

void MotorController::disableServo(int motor_id) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id >= 0 && motor_id < static_cast<int>(motors_.size())) {
        motors_[motor_id].servo_enabled = false;
    }
}

bool MotorController::setServoTarget(int motor_id, double position) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    
    if (!motor.servo_enabled) {
        setError("Servo mode not enabled for this motor");
        return false;
    }

    motor.servo_target_position = position;
    return true;
}

bool MotorController::setServoParams(int motor_id, double max_velocity, double acceleration) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    motor.servo_max_velocity = max_velocity;
    motor.servo_acceleration = acceleration;

    return true;
}

MotorStatus MotorController::getMotorStatus(int motor_id) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        MotorStatus status{};
        status.has_error = true;
        return status;
    }

    return motors_[motor_id].status;
}

double MotorController::getPosition(int motor_id) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        return 0.0;
    }

    return encoderToUser(motor_id, motors_[motor_id].status.position);
}

double MotorController::getVelocity(int motor_id) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        return 0.0;
    }

    const Motor& motor = motors_[motor_id];
    double vel_user_per_sec = static_cast<double>(motor.status.velocity) / motor.config.encoder_resolution;
    return vel_user_per_sec / motor.config.gear_ratio;
}

bool MotorController::isMotorReady(int motor_id) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        return false;
    }

    const Motor& motor = motors_[motor_id];
    return motor.status.state == MotorState::OPERATION_ENABLED && 
           motor.status.is_enabled && !motor.status.has_error;
}

bool MotorController::isInPosition(int motor_id, double tolerance) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        return false;
    }

    const Motor& motor = motors_[motor_id];
    double current_pos = encoderToUser(motor_id, motor.status.position);
    double target_pos = motor.servo_enabled ? motor.servo_target_position : 
                       motor.current_motion.target_position;
    
    return std::abs(current_pos - target_pos) <= tolerance;
}

bool MotorController::startHoming(int motor_id, int method) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    motor.homing_active = true;
    motor.homing_method = method;
    motor.motion_active = false;
    motor.servo_enabled = false;

    return true;
}

bool MotorController::setZeroPosition(int motor_id) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    // In real implementation, this would write to the drive's position offset
    motor.status.is_homed = true;
    
    return true;
}

bool MotorController::isHomingComplete(int motor_id) const {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        return false;
    }

    const Motor& motor = motors_[motor_id];
    return !motor.homing_active && motor.status.is_homed;
}

bool MotorController::clearErrors(int motor_id) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        setError("Invalid motor ID");
        return false;
    }

    Motor& motor = motors_[motor_id];
    motor.status.has_error = false;
    motor.status.error_code = 0;

    return true;
}

void MotorController::emergencyStop() {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    for (auto& motor : motors_) {
        motor.motion_active = false;
        motor.servo_enabled = false;
        motor.current_motion.max_velocity = 0;
        motor.control_mode = ControlMode::VELOCITY;
    }
}

int32_t MotorController::userToEncoder(int motor_id, double user_units) const {
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        return 0;
    }

    const Motor& motor = motors_[motor_id];
    return static_cast<int32_t>(user_units * motor.config.gear_ratio * motor.config.encoder_resolution);
}

double MotorController::encoderToUser(int motor_id, int32_t encoder_counts) const {
    if (motor_id < 0 || motor_id >= static_cast<int>(motors_.size())) {
        return 0.0;
    }

    const Motor& motor = motors_[motor_id];
    return static_cast<double>(encoder_counts) / (motor.config.gear_ratio * motor.config.encoder_resolution);
}

void MotorController::controlLoop() {
    struct timespec wakeup_time;
    struct timespec cycle_time = {0, static_cast<long>(cycle_time_ns_)};
    
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    while (running_) {
        // Add cycle time to wakeup time
        wakeup_time.tv_nsec += cycle_time.tv_nsec;
        if (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_sec++;
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
        }

        // Sleep until next cycle
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, nullptr);

        if (!running_) break;

        // Receive EtherCAT frames
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);

        // Update motor states and process control
        updateMotorStates();

        // Process control for each motor
        {
            std::lock_guard<std::mutex> lock(motors_mutex_);
            for (auto& motor : motors_) {
                processMotorControl(motor);
            }
        }

        // Send EtherCAT frames
        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);
    }
}

void MotorController::updateMotorStates() {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    for (auto& motor : motors_) {
        if (!domain_data_) continue;

        // Read status word
        motor.status.status_word = EC_READ_U16(domain_data_ + motor.status_word_offset);
        motor.status.state = getMotorState(motor.status.status_word);
        
        // Read actual values
        motor.status.position = EC_READ_S32(domain_data_ + motor.actual_position_offset);
        motor.status.velocity = EC_READ_S32(domain_data_ + motor.actual_velocity_offset);
        motor.status.torque = EC_READ_S16(domain_data_ + motor.actual_torque_offset);
        
        // Update status flags
        motor.status.has_error = (motor.status.state == MotorState::FAULT ||
                                 motor.status.state == MotorState::FAULT_REACTION_ACTIVE);
    }
}

void MotorController::processMotorControl(Motor& motor) {
    if (!domain_data_) return;

    uint64_t current_time = getCurrentTimeNs();
    
    // Write control word
    uint16_t control_word = getControlWord(motor);
    EC_WRITE_U16(domain_data_ + motor.control_word_offset, control_word);

    // Process based on control mode
    if (motor.servo_enabled) {
        processServoControl(motor, current_time);
    } else if (motor.motion_active) {
        processMotionControl(motor, current_time);
    } else {
        // Set values based on control mode
        switch (motor.control_mode) {
            case ControlMode::VELOCITY:
                {
                    int32_t vel_enc = userToEncoder(0, motor.current_motion.max_velocity);
                    EC_WRITE_S32(domain_data_ + motor.target_velocity_offset, vel_enc);
                }
                break;
                
            case ControlMode::POSITION:
                {
                    int32_t pos_enc = userToEncoder(0, motor.current_motion.target_position);
                    EC_WRITE_S32(domain_data_ + motor.target_position_offset, pos_enc);
                }
                break;
                
            default:
                break;
        }
    }
}

void MotorController::processServoControl(Motor& motor, uint64_t current_time_ns) {
    // Simple servo control - calculate intermediate position based on current position and target
    double current_pos = encoderToUser(0, motor.status.position);
    double error = motor.servo_target_position - current_pos;
    
    // Calculate maximum distance we can move in one cycle
    double max_distance_per_cycle = motor.servo_max_velocity * (cycle_time_ns_ / 1e9);
    
    // Limit the movement to max distance
    double next_pos = current_pos;
    if (std::abs(error) > max_distance_per_cycle) {
        next_pos += (error > 0) ? max_distance_per_cycle : -max_distance_per_cycle;
    } else {
        next_pos = motor.servo_target_position;
    }
    
    // Write target position
    int32_t target_pos_enc = userToEncoder(0, next_pos);
    EC_WRITE_S32(domain_data_ + motor.target_position_offset, target_pos_enc);
}

void MotorController::processMotionControl(Motor& motor, uint64_t current_time_ns) {
    double elapsed_time = (current_time_ns - motor.motion_start_time) / 1e9;
    
    double target_pos = calculateTrajectoryPosition(motor.current_motion, 
                                                  motor.motion_start_position, 
                                                  elapsed_time);
    
    // Check if motion is complete
    double distance_to_target = std::abs(target_pos - motor.current_motion.target_position);
    if (distance_to_target < 0.001) { // Small tolerance
        motor.motion_active = false;
        target_pos = motor.current_motion.target_position;
    }
    
    // Write target position
    int32_t target_pos_enc = userToEncoder(0, target_pos);
    EC_WRITE_S32(domain_data_ + motor.target_position_offset, target_pos_enc);
}

MotorState MotorController::getMotorState(uint16_t status_word) const {
    // Extract state bits according to CiA402
    uint16_t state_bits = status_word & 0x006F;
    
    switch (state_bits) {
        case 0x0000: return MotorState::NOT_READY_TO_SWITCH_ON;
        case 0x0040: return MotorState::SWITCH_ON_DISABLED;
        case 0x0021: return MotorState::READY_TO_SWITCH_ON;
        case 0x0023: return MotorState::SWITCHED_ON;
        case 0x0027: return MotorState::OPERATION_ENABLED;
        case 0x0007: return MotorState::QUICK_STOP_ACTIVE;
        case 0x000F: return MotorState::FAULT_REACTION_ACTIVE;
        case 0x0008: return MotorState::FAULT;
        default: return MotorState::NOT_READY_TO_SWITCH_ON;
    }
}

uint16_t MotorController::getControlWord(Motor& motor) const {
    switch (motor.status.state) {
        case MotorState::FAULT:
            return CONTROL_WORD_FAULT_RESET;
            
        case MotorState::SWITCH_ON_DISABLED:
            return CONTROL_WORD_SHUTDOWN;
            
        case MotorState::READY_TO_SWITCH_ON:
            return CONTROL_WORD_SWITCH_ON;
            
        case MotorState::SWITCHED_ON:
            return CONTROL_WORD_ENABLE_OPERATION;
            
        case MotorState::OPERATION_ENABLED:
            if (motor.status.is_enabled) {
                return CONTROL_WORD_ENABLE_OPERATION;
            } else {
                return CONTROL_WORD_SHUTDOWN;
            }
            
        default:
            return CONTROL_WORD_SHUTDOWN;
    }
}

void MotorController::setError(const std::string& error) const {
    last_error_ = error;
    std::cerr << "MotorController Error: " << error << std::endl;
}

uint64_t MotorController::getCurrentTimeNs() const {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
}

double MotorController::calculateTrajectoryPosition(const MotionParams& params, 
                                                   double start_pos, double elapsed_time) const {
    // Simple trapezoidal velocity profile
    double distance = params.target_position - start_pos;
    double sign = (distance >= 0) ? 1.0 : -1.0;
    distance = std::abs(distance);
    
    double max_vel = params.max_velocity;
    double accel = params.acceleration;
    
    // Calculate acceleration and deceleration times
    double t_accel = max_vel / accel;
    double t_decel = max_vel / params.deceleration;
    
    // Calculate distances during acceleration and deceleration
    double d_accel = 0.5 * accel * t_accel * t_accel;
    double d_decel = 0.5 * params.deceleration * t_decel * t_decel;
    
    // Check if we can reach max velocity
    if (d_accel + d_decel > distance) {
        // Triangular profile
        t_accel = std::sqrt(distance / accel);
        d_accel = distance / 2.0;
    }
    
    double d_const = distance - d_accel - d_decel;
    double t_const = (d_const > 0) ? d_const / max_vel : 0;
    
    double position = start_pos;
    
    if (elapsed_time <= t_accel) {
        // Acceleration phase
        position += sign * 0.5 * accel * elapsed_time * elapsed_time;
    } else if (elapsed_time <= t_accel + t_const) {
        // Constant velocity phase
        double t_in_const = elapsed_time - t_accel;
        position += sign * (d_accel + max_vel * t_in_const);
    } else {
        // Deceleration phase
        double t_in_decel = elapsed_time - t_accel - t_const;
        if (t_in_decel <= t_decel) {
            double vel_at_decel_start = max_vel;
            position += sign * (d_accel + d_const + 
                              vel_at_decel_start * t_in_decel - 
                              0.5 * params.deceleration * t_in_decel * t_in_decel);
        } else {
            // Motion complete
            position = params.target_position;
        }
    }
    
    return position;
}