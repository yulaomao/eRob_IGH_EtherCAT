#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ecrt.h>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

/**
 * @brief Motor control modes
 */
enum class ControlMode {
    VELOCITY,           // CSV - Cyclic Synchronous Velocity
    POSITION,           // CSP - Cyclic Synchronous Position  
    TORQUE,             // CST - Cyclic Synchronous Torque
    PROFILE_POSITION,   // PP - Profile Position
    PROFILE_VELOCITY    // PV - Profile Velocity
};

/**
 * @brief Motor states based on CiA402 standard
 */
enum class MotorState {
    NOT_READY_TO_SWITCH_ON = 0x00,
    SWITCH_ON_DISABLED = 0x40,
    READY_TO_SWITCH_ON = 0x21,
    SWITCHED_ON = 0x23,
    OPERATION_ENABLED = 0x27,
    QUICK_STOP_ACTIVE = 0x07,
    FAULT_REACTION_ACTIVE = 0x0F,
    FAULT = 0x08
};

/**
 * @brief Motor configuration structure
 */
struct MotorConfig {
    uint16_t position;          // EtherCAT position
    uint32_t vendor_id;         // Vendor ID
    uint32_t product_code;      // Product code
    std::string name;           // Motor name/identifier
    int32_t encoder_resolution; // Encoder counts per revolution
    double gear_ratio;          // Gear ratio
    double max_velocity;        // Maximum velocity (user units/s)
    double max_acceleration;    // Maximum acceleration (user units/s²)
};

/**
 * @brief Motor status information
 */
struct MotorStatus {
    MotorState state;           // Current motor state
    int32_t position;           // Current position (encoder counts)
    int32_t velocity;           // Current velocity (encoder counts/s)
    int16_t torque;             // Current torque
    uint16_t status_word;       // Raw status word
    uint16_t error_code;        // Error code if any
    bool is_enabled;            // Motor is enabled
    bool is_homed;              // Motor is homed
    bool has_error;             // Motor has error
};

/**
 * @brief Motion parameters for position control
 */
struct MotionParams {
    double target_position;     // Target position (user units)
    double max_velocity;        // Maximum velocity for this move (user units/s)
    double acceleration;        // Acceleration (user units/s²)
    double deceleration;        // Deceleration (user units/s²)
    bool relative;              // True for relative move, false for absolute
};

/**
 * @brief Trajectory point for servo control
 */
struct TrajectoryPoint {
    double position;            // Position (user units)
    double velocity;            // Velocity (user units/s)
    double acceleration;        // Acceleration (user units/s²)
    uint64_t timestamp_ns;      // Timestamp in nanoseconds
};

/**
 * @brief Comprehensive motor controller class for EtherCAT servo drives
 * 
 * This class provides a complete interface for controlling multiple servo motors
 * via EtherCAT using the IGH EtherCAT master. It supports:
 * - Automatic motor discovery and configuration
 * - Position, velocity, and torque control modes
 * - Real-time servo control with trajectory planning
 * - Safety functions (emergency stop, error handling)
 * - Homing and referencing operations
 */
class MotorController {
public:
    /**
     * @brief Constructor
     */
    MotorController();
    
    /**
     * @brief Destructor
     */
    ~MotorController();

    // === INITIALIZATION ===
    
    /**
     * @brief Initialize the EtherCAT master and scan for motors
     * @param interface_name Network interface name (e.g., "eth0")
     * @return true if initialization successful
     */
    bool initialize(const std::string& interface_name = "");
    
    /**
     * @brief Add motor configuration manually
     * @param config Motor configuration
     * @return Motor ID (index) if successful, -1 if failed
     */
    int addMotor(const MotorConfig& config);
    
    /**
     * @brief Start the control loop
     * @param frequency_hz Control loop frequency (default: 1000 Hz)
     * @return true if started successfully
     */
    bool start(int frequency_hz = 1000);
    
    /**
     * @brief Stop the control loop and shutdown
     */
    void shutdown();

    // === MOTOR ENABLE/DISABLE ===
    
    /**
     * @brief Enable a specific motor
     * @param motor_id Motor ID
     * @return true if successful
     */
    bool enableMotor(int motor_id);
    
    /**
     * @brief Disable a specific motor
     * @param motor_id Motor ID
     * @return true if successful
     */
    bool disableMotor(int motor_id);
    
    /**
     * @brief Enable all motors
     * @return true if all motors enabled successfully
     */
    bool enableAllMotors();
    
    /**
     * @brief Disable all motors
     */
    void disableAllMotors();

    // === MOTION CONTROL ===
    
    /**
     * @brief Move motor to absolute position
     * @param motor_id Motor ID
     * @param position Target position (user units)
     * @param velocity Maximum velocity (user units/s)
     * @param acceleration Acceleration (user units/s²)
     * @return true if command accepted
     */
    bool moveAbsolute(int motor_id, double position, double velocity = 0, double acceleration = 0);
    
    /**
     * @brief Move motor relative distance
     * @param motor_id Motor ID
     * @param distance Relative distance (user units)
     * @param velocity Maximum velocity (user units/s)
     * @param acceleration Acceleration (user units/s²)
     * @return true if command accepted
     */
    bool moveRelative(int motor_id, double distance, double velocity = 0, double acceleration = 0);
    
    /**
     * @brief Set velocity for continuous motion
     * @param motor_id Motor ID
     * @param velocity Target velocity (user units/s)
     * @return true if command accepted
     */
    bool setVelocity(int motor_id, double velocity);
    
    /**
     * @brief Stop motor motion
     * @param motor_id Motor ID
     * @param deceleration Deceleration rate (0 = use default)
     * @return true if command accepted
     */
    bool stopMotor(int motor_id, double deceleration = 0);

    // === SERVO CONTROL ===
    
    /**
     * @brief Enable servo mode for real-time position control
     * @param motor_id Motor ID
     * @return true if servo mode enabled
     */
    bool enableServo(int motor_id);
    
    /**
     * @brief Disable servo mode
     * @param motor_id Motor ID
     */
    void disableServo(int motor_id);
    
    /**
     * @brief Set target position for servo control
     * @param motor_id Motor ID
     * @param position Target position (user units)
     * @return true if command accepted
     */
    bool setServoTarget(int motor_id, double position);
    
    /**
     * @brief Set servo parameters
     * @param motor_id Motor ID
     * @param max_velocity Maximum velocity (user units/s)
     * @param acceleration Acceleration (user units/s²)
     * @return true if parameters set
     */
    bool setServoParams(int motor_id, double max_velocity, double acceleration);

    // === STATUS AND MONITORING ===
    
    /**
     * @brief Get motor status
     * @param motor_id Motor ID
     * @return Motor status structure
     */
    MotorStatus getMotorStatus(int motor_id) const;
    
    /**
     * @brief Get current position
     * @param motor_id Motor ID
     * @return Current position in user units
     */
    double getPosition(int motor_id) const;
    
    /**
     * @brief Get current velocity
     * @param motor_id Motor ID
     * @return Current velocity in user units/s
     */
    double getVelocity(int motor_id) const;
    
    /**
     * @brief Check if motor is ready
     * @param motor_id Motor ID
     * @return true if motor is operational
     */
    bool isMotorReady(int motor_id) const;
    
    /**
     * @brief Check if motor is in position
     * @param motor_id Motor ID
     * @param tolerance Position tolerance (user units)
     * @return true if motor is within tolerance of target
     */
    bool isInPosition(int motor_id, double tolerance = 0.1) const;
    
    /**
     * @brief Get number of configured motors
     * @return Number of motors
     */
    int getMotorCount() const { return static_cast<int>(motors_.size()); }

    // === HOMING AND REFERENCING ===
    
    /**
     * @brief Start homing sequence for a motor
     * @param motor_id Motor ID
     * @param method Homing method (implementation specific)
     * @return true if homing started
     */
    bool startHoming(int motor_id, int method = 1);
    
    /**
     * @brief Set current position as zero
     * @param motor_id Motor ID
     * @return true if zero set
     */
    bool setZeroPosition(int motor_id);
    
    /**
     * @brief Check if homing is complete
     * @param motor_id Motor ID
     * @return true if homing is complete
     */
    bool isHomingComplete(int motor_id) const;

    // === ERROR HANDLING ===
    
    /**
     * @brief Clear motor errors
     * @param motor_id Motor ID
     * @return true if errors cleared
     */
    bool clearErrors(int motor_id);
    
    /**
     * @brief Emergency stop all motors
     */
    void emergencyStop();
    
    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const { return last_error_; }

    // === UTILITY FUNCTIONS ===
    
    /**
     * @brief Convert user units to encoder counts
     * @param motor_id Motor ID
     * @param user_units Position in user units
     * @return Position in encoder counts
     */
    int32_t userToEncoder(int motor_id, double user_units) const;
    
    /**
     * @brief Convert encoder counts to user units
     * @param motor_id Motor ID
     * @param encoder_counts Position in encoder counts
     * @return Position in user units
     */
    double encoderToUser(int motor_id, int32_t encoder_counts) const;

private:
    // Internal structures
    struct Motor {
        MotorConfig config;
        MotorStatus status;
        ec_slave_config_t* slave_config;
        
        // PDO offsets
        unsigned int control_word_offset;
        unsigned int status_word_offset;
        unsigned int target_position_offset;
        unsigned int actual_position_offset;
        unsigned int target_velocity_offset;
        unsigned int actual_velocity_offset;
        unsigned int target_torque_offset;
        unsigned int actual_torque_offset;
        
        // Control state
        ControlMode control_mode;
        bool servo_enabled;
        double servo_target_position;
        double servo_max_velocity;
        double servo_acceleration;
        
        // Motion state
        bool motion_active;
        MotionParams current_motion;
        double motion_start_position;
        uint64_t motion_start_time;
        
        // Homing state
        bool homing_active;
        int homing_method;
        
        Motor() : slave_config(nullptr), control_mode(ControlMode::POSITION),
                 servo_enabled(false), servo_target_position(0),
                 servo_max_velocity(100), servo_acceleration(1000),
                 motion_active(false), motion_start_position(0),
                 motion_start_time(0), homing_active(false), homing_method(1) {}
    };

    // EtherCAT master and domain
    ec_master_t* master_;
    ec_domain_t* domain_;
    uint8_t* domain_data_;
    
    // Motor management
    std::vector<Motor> motors_;
    mutable std::mutex motors_mutex_;
    
    // Control loop
    std::atomic<bool> running_;
    std::thread control_thread_;
    int frequency_hz_;
    uint64_t cycle_time_ns_;
    
    // Error handling
    mutable std::string last_error_;
    
    // Private methods
    void controlLoop();
    bool configurePDOs();
    bool activateMaster();
    void updateMotorStates();
    void processMotorControl(Motor& motor);
    void processServoControl(Motor& motor, uint64_t current_time_ns);
    void processMotionControl(Motor& motor, uint64_t current_time_ns);
    MotorState getMotorState(uint16_t status_word) const;
    uint16_t getControlWord(Motor& motor) const;
    void setError(const std::string& error) const;
    uint64_t getCurrentTimeNs() const;
    double calculateTrajectoryPosition(const MotionParams& params, 
                                     double start_pos, double elapsed_time) const;
};

#endif // MOTOR_CONTROLLER_H