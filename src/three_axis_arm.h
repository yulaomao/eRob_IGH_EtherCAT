#ifndef THREE_AXIS_ARM_H
#define THREE_AXIS_ARM_H

#include "motor_controller.h"
#include <array>

/**
 * @brief Three-axis robotic arm controller
 * 
 * This class provides high-level control for a three-axis robotic arm,
 * built on top of the MotorController class.
 */
class ThreeAxisArm {
public:
    /**
     * @brief Arm joint identifiers
     */
    enum Joint {
        BASE = 0,       // Base rotation (J1)
        SHOULDER = 1,   // Shoulder (J2) 
        ELBOW = 2       // Elbow (J3)
    };

    /**
     * @brief Joint configuration
     */
    struct JointConfig {
        double min_angle;       // Minimum joint angle (degrees)
        double max_angle;       // Maximum joint angle (degrees)
        double max_velocity;    // Maximum velocity (degrees/s)
        double max_acceleration; // Maximum acceleration (degrees/s²)
        double home_position;   // Home position (degrees)
    };

    /**
     * @brief Cartesian position
     */
    struct CartesianPos {
        double x;  // X coordinate (mm)
        double y;  // Y coordinate (mm) 
        double z;  // Z coordinate (mm)
    };

    /**
     * @brief Joint angles
     */
    struct JointAngles {
        double base;      // Base angle (degrees)
        double shoulder;  // Shoulder angle (degrees)
        double elbow;     // Elbow angle (degrees)
    };

    /**
     * @brief Constructor
     */
    ThreeAxisArm();

    /**
     * @brief Destructor
     */
    ~ThreeAxisArm();

    /**
     * @brief Initialize the arm controller
     * @param base_config Configuration for base motor
     * @param shoulder_config Configuration for shoulder motor
     * @param elbow_config Configuration for elbow motor
     * @return true if initialization successful
     */
    bool initialize(const MotorConfig& base_config,
                   const MotorConfig& shoulder_config,
                   const MotorConfig& elbow_config);

    /**
     * @brief Set joint configurations
     * @param joint_configs Array of joint configurations
     */
    void setJointConfigs(const std::array<JointConfig, 3>& joint_configs);

    /**
     * @brief Start the arm controller
     * @return true if started successfully
     */
    bool start();

    /**
     * @brief Shutdown the arm controller
     */
    void shutdown();

    /**
     * @brief Enable all joints
     * @return true if all joints enabled
     */
    bool enable();

    /**
     * @brief Disable all joints
     */
    void disable();

    /**
     * @brief Home all joints
     * @return true if homing started
     */
    bool homeAll();

    /**
     * @brief Check if homing is complete
     * @return true if all joints are homed
     */
    bool isHomingComplete() const;

    /**
     * @brief Move to joint angles
     * @param angles Target joint angles
     * @param velocity Maximum velocity (degrees/s, 0 = use default)
     * @param acceleration Acceleration (degrees/s², 0 = use default)
     * @return true if command accepted
     */
    bool moveToJointAngles(const JointAngles& angles, 
                          double velocity = 0, 
                          double acceleration = 0);

    /**
     * @brief Move single joint
     * @param joint Joint to move
     * @param angle Target angle (degrees)
     * @param velocity Maximum velocity (degrees/s, 0 = use default)
     * @param acceleration Acceleration (degrees/s², 0 = use default)
     * @return true if command accepted
     */
    bool moveJoint(Joint joint, double angle, 
                  double velocity = 0, 
                  double acceleration = 0);

    /**
     * @brief Move to Cartesian position (simple forward kinematics)
     * @param position Target Cartesian position
     * @param velocity Maximum joint velocity (degrees/s)
     * @param acceleration Joint acceleration (degrees/s²)
     * @return true if position is reachable and command accepted
     */
    bool moveToCartesian(const CartesianPos& position,
                        double velocity = 0,
                        double acceleration = 0);

    /**
     * @brief Enable servo mode for real-time control
     * @return true if servo mode enabled
     */
    bool enableServoMode();

    /**
     * @brief Disable servo mode
     */
    void disableServoMode();

    /**
     * @brief Set servo target joint angles
     * @param angles Target joint angles
     * @return true if command accepted
     */
    bool setServoTargetAngles(const JointAngles& angles);

    /**
     * @brief Set servo target Cartesian position
     * @param position Target Cartesian position
     * @return true if position is reachable and command accepted
     */
    bool setServoTargetCartesian(const CartesianPos& position);

    /**
     * @brief Get current joint angles
     * @return Current joint angles
     */
    JointAngles getCurrentJointAngles() const;

    /**
     * @brief Get current Cartesian position
     * @return Current Cartesian position
     */
    CartesianPos getCurrentCartesianPos() const;

    /**
     * @brief Check if arm is ready (all joints enabled and operational)
     * @return true if arm is ready
     */
    bool isReady() const;

    /**
     * @brief Check if arm is in position
     * @param tolerance Angle tolerance (degrees)
     * @return true if all joints are within tolerance
     */
    bool isInPosition(double tolerance = 0.5) const;

    /**
     * @brief Emergency stop
     */
    void emergencyStop();

    /**
     * @brief Clear all errors
     * @return true if errors cleared
     */
    bool clearErrors();

    /**
     * @brief Get motor controller reference
     * @return Reference to underlying motor controller
     */
    MotorController& getMotorController() { return motor_controller_; }

    /**
     * @brief Get last error message
     * @return Error message
     */
    std::string getLastError() const { return motor_controller_.getLastError(); }

    /**
     * @brief Forward kinematics - convert joint angles to Cartesian position
     * @param angles Joint angles
     * @return Cartesian position
     */
    CartesianPos forwardKinematics(const JointAngles& angles) const;

    /**
     * @brief Convert degrees to motor units for a joint
     * @param joint Joint index
     * @param degrees Angle in degrees
     * @return Angle in motor units
     */
    double degreesToMotorUnits(Joint joint, double degrees) const;

    /**
     * @brief Convert motor units to degrees for a joint
     * @param joint Joint index
     * @param motor_units Angle in motor units
     * @return Angle in degrees
     */
    double motorUnitsToDegrees(Joint joint, double motor_units) const;

private:
    MotorController motor_controller_;
    std::array<JointConfig, 3> joint_configs_;
    bool initialized_;

    // Arm geometry (simplified for demonstration)
    static constexpr double L1 = 200.0;  // Base to shoulder length (mm)
    static constexpr double L2 = 250.0;  // Shoulder to elbow length (mm)
    static constexpr double L3 = 150.0;  // Elbow to end effector length (mm)

    /**
     * @brief Inverse kinematics - convert Cartesian position to joint angles
     * @param position Cartesian position
     * @param angles Output joint angles
     * @return true if solution exists
     */
    bool inverseKinematics(const CartesianPos& position, JointAngles& angles) const;

    /**
     * @brief Check if joint angles are within limits
     * @param angles Joint angles to check
     * @return true if all angles are within limits
     */
    bool checkJointLimits(const JointAngles& angles) const;
};

#endif // THREE_AXIS_ARM_H