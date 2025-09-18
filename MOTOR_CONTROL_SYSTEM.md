# 核心电机控制系统文档 (Core Motor Control System Documentation)

## 概述 (Overview)

本项目提供了一个基于IGH EtherCAT的完善电机控制系统，能够实现多轴伺服电机的高级控制功能。系统包含核心控制类`MotorController`和三轴机械臂控制类`ThreeAxisArm`。

This project provides a comprehensive motor control system based on IGH EtherCAT, capable of advanced control functions for multi-axis servo motors. The system includes the core control class `MotorController` and a three-axis robotic arm control class `ThreeAxisArm`.

## 系统特性 (System Features)

### 1. 核心功能 (Core Functions)

#### 自动初始化 (Automatic Initialization)
- ✅ 自动扫描EtherCAT网络上的电机设备
- ✅ 自动配置PDO映射和通信参数
- ✅ 状态机自动管理（从INIT到OPERATION ENABLED）

#### 运动控制 (Motion Control)
- ✅ 绝对位置控制（指定速度和加减速时间）
- ✅ 相对位置控制
- ✅ 速度控制模式
- ✅ 扭矩控制模式
- ✅ 轨迹规划（梯形速度曲线）

#### 实时伺服控制 (Real-time Servo Control)
- ✅ 周期位置模式（16ms内每ms位置计算）
- ✅ 实时目标位置设置
- ✅ 平滑轨迹生成
- ✅ 高频率控制循环（1000Hz默认）

#### 基础控制功能 (Basic Control Functions)
- ✅ 电机使能/失能控制
- ✅ 位置、速度、扭矩反馈获取
- ✅ 归零和设置零点功能
- ✅ 错误代码读取和清除
- ✅ 急停功能
- ✅ 安全关机

### 2. 三轴机械臂控制 (Three-Axis Robotic Arm Control)

#### 运动学功能 (Kinematics Functions)
- ✅ 正向运动学（关节角度→笛卡尔坐标）
- ✅ 逆向运动学（笛卡尔坐标→关节角度）
- ✅ 关节限位检查
- ✅ 工作空间验证

#### 高级控制模式 (Advanced Control Modes)
- ✅ 关节空间运动控制
- ✅ 笛卡尔空间运动控制
- ✅ 实时伺服模式
- ✅ 协调多轴运动

## 文件结构 (File Structure)

```
src/
├── motor_controller.h      # 核心电机控制类头文件
├── motor_controller.cpp    # 核心电机控制类实现
├── three_axis_arm.h       # 三轴机械臂控制类头文件
├── three_axis_arm.cpp     # 三轴机械臂控制类实现
├── arm_demo.cpp          # 演示程序
└── igh_driver.cpp        # 原始驱动程序（保留）
```

## 编译和运行 (Build and Run)

### 前提条件 (Prerequisites)

1. 安装IGH EtherCAT Master（参考原README.md）
2. 确保EtherCAT网络配置正确
3. 连接伺服驱动器到EtherCAT网络

### 编译 (Build)

```bash
mkdir build
cd build
cmake ..
make
```

### 运行演示程序 (Run Demo)

```bash
# 启动EtherCAT主站
sudo /etc/init.d/ethercat start

# 运行三轴机械臂演示
sudo ./arm_demo
```

## API使用说明 (API Usage)

### MotorController类使用示例

```cpp
#include "motor_controller.h"

// 创建控制器
MotorController controller;

// 初始化
controller.initialize();

// 添加电机
MotorConfig config;
config.position = 0;
config.vendor_id = 0x5a65726f;
config.product_code = 0x00029252;
config.name = "Motor1";
config.encoder_resolution = 524287;
config.gear_ratio = 50.0;
config.max_velocity = 180.0;
config.max_acceleration = 360.0;

int motor_id = controller.addMotor(config);

// 启动控制循环
controller.start(1000); // 1000Hz

// 使能电机
controller.enableMotor(motor_id);

// 位置控制
controller.moveAbsolute(motor_id, 90.0, 30.0, 60.0); // 90度，30度/s，60度/s²

// 伺服控制
controller.enableServo(motor_id);
controller.setServoTarget(motor_id, 45.0);

// 状态监控
MotorStatus status = controller.getMotorStatus(motor_id);
double current_pos = controller.getPosition(motor_id);
bool ready = controller.isMotorReady(motor_id);

// 关闭
controller.shutdown();
```

### ThreeAxisArm类使用示例

```cpp
#include "three_axis_arm.h"

// 创建机械臂控制器
ThreeAxisArm arm;

// 配置电机
MotorConfig base_config, shoulder_config, elbow_config;
// ... 配置参数

// 初始化
arm.initialize(base_config, shoulder_config, elbow_config);
arm.start();
arm.enable();

// 归零
arm.homeAll();
while (!arm.isHomingComplete()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// 关节空间运动
ThreeAxisArm::JointAngles angles = {45.0, 30.0, -45.0};
arm.moveToJointAngles(angles, 30.0, 60.0);

// 笛卡尔空间运动
ThreeAxisArm::CartesianPos position = {300.0, 100.0, 250.0};
arm.moveToCartesian(position);

// 实时伺服控制
arm.enableServoMode();
arm.setServoTargetCartesian({250.0, 150.0, 280.0});

// 状态监控
auto current_angles = arm.getCurrentJointAngles();
auto current_pos = arm.getCurrentCartesianPos();
bool ready = arm.isReady();

arm.shutdown();
```

## 关键特性详解 (Key Features Details)

### 1. 自动电机扫描和配置

系统能够自动检测EtherCAT网络上的电机设备，并根据厂商ID和产品代码自动配置通信参数。

### 2. 实时轨迹规划

采用梯形速度曲线算法，在16ms控制周期内计算每1ms的目标位置，确保平滑的运动轨迹。

### 3. 多种控制模式

- **位置控制模式 (CSP)**: 高精度位置控制
- **速度控制模式 (CSV)**: 连续速度控制
- **扭矩控制模式 (CST)**: 力控制应用
- **轮廓位置模式 (PP)**: 复杂轨迹控制

### 4. 安全机制

- 急停功能立即停止所有电机
- 软硬限位保护
- 错误检测和自动恢复
- 看门狗保护

### 5. 高性能实时控制

- 1000Hz控制频率
- 低延迟通信
- 分布式时钟同步
- 多线程并发处理

## 扩展开发 (Extension Development)

### 添加新的电机类型

1. 继承`MotorController`类或创建新的配置
2. 实现特定的PDO映射
3. 添加专用的控制算法

### 添加新的机械臂配置

1. 继承`ThreeAxisArm`类
2. 修改运动学参数
3. 实现特定的逆运动学算法

### 集成外部传感器

1. 扩展控制循环
2. 添加传感器数据处理
3. 实现闭环控制算法

## 故障排除 (Troubleshooting)

### 常见问题

1. **EtherCAT主站启动失败**
   - 检查网络接口配置
   - 验证权限设置
   - 确认驱动程序安装

2. **电机连接失败**
   - 检查EtherCAT线缆连接
   - 验证电机配置参数
   - 查看电机状态指示灯

3. **运动控制异常**
   - 检查限位设置
   - 验证速度加速度参数
   - 查看错误代码

### 调试工具

```bash
# 查看EtherCAT网络状态
ethercat slaves

# 查看实时数据
ethercat pdos

# 监控通信
ethercat master
```

## 贡献和支持 (Contributing and Support)

欢迎提交问题报告和功能请求。对于技术支持，请参考项目的GitHub页面。

Welcome to submit issue reports and feature requests. For technical support, please refer to the project's GitHub page.

## 许可证 (License)

本项目采用开源许可证。请谨慎使用，作者不承担任何损失或损害责任。

This project is licensed under an open source license. Please use with caution. The author is not responsible for any damages or losses.