# Robot Interfaces Library

A ROS2 hardware interface library that provides unified robot control abstractions for various robot manipulators. This library bridges the ROS2 control framework with physical and simulated robots through a modular, extensible architecture.

## Overview

The `robot_interfaces` package implements hardware interfaces for ROS2 control, enabling seamless integration between high-level controllers and robot hardware. The library provides:

- **Unified Interface Abstraction**: Common API for different robot types and control modes
- **Forward Kinematics**: KDL-based real-time pose computation for any URDF-described robot
- **Multi-Robot Support**: Interfaces for Franka Emika, Kinova Gen3, and custom robots
- **Multiple Control Modes**: Cartesian velocity and joint position control interfaces
- **Thread-Safe Operation**: Executor-driven state management with proper synchronization

## Architecture

### Core Components

```
ROS2 Control Framework
         ↓
Robot Interface Library
    ├── GenericComponent (Base Class)
    │   ├── KDL Forward Kinematics (URDF-based)
    │   ├── Joint State Management
    │   ├── Thread-Safe State Access
    │   └── Lifecycle Management
    │
    ├── Cartesian Velocity Interfaces
    │   ├── FrankaCartesianVelocity
    │   ├── KinovaCartesianVelocity
    │   └── ExplorerCartesianVelocity
    │
    └── Joint Position Interfaces
        └── GenericJointPosition
```

### GenericComponent Base Class

The foundation of all robot interfaces, providing:

- **KDL-Based Forward Kinematics**: Real-time end-effector pose computation from joint states
- **Joint State Integration**: Executor-driven subscription to `/joint_states` topic
- **URDF Support**: Automatic model parsing and kinematic chain extraction
- **Frame Management**: Configurable base and tool frame mapping
- **Thread Safety**: Mutex-protected state access for real-time operation

## Available Interfaces

### Cartesian Velocity Interfaces

High-level velocity control in Cartesian space with 6DOF twist commands.

#### FrankaCartesianVelocity
**Robot**: Franka Emika FR3/Panda (7-DOF)

Impedance-controlled arm with built-in force/torque sensing. Provides Cartesian velocity control with real-time pose feedback.

**Command Interfaces**:
- `vx/cartesian_velocity`, `vy/cartesian_velocity`, `vz/cartesian_velocity` - Linear velocity (m/s)
- `wx/cartesian_velocity`, `wy/cartesian_velocity`, `wz/cartesian_velocity` - Angular velocity (rad/s)

**Configuration**:
```cpp
robot_interfaces::FrankaCartesianVelocity component;
component.initKinematics(urdf_xml, "base", "fr3_hand_tcp");
```

#### KinovaCartesianVelocity
**Robot**: Kinova Gen3 (7-DOF) with Robotiq gripper

Cartesian velocity control for Kinova Gen3 manipulators with integrated gripper support.

**Command Interfaces**:
- `tcp/twist.linear.x`, `tcp/twist.linear.y`, `tcp/twist.linear.z` - Linear velocity (m/s)
- `tcp/twist.angular.x`, `tcp/twist.angular.y`, `tcp/twist.angular.z` - Angular velocity (rad/s)

**Configuration**:
```cpp
robot_interfaces::KinovaCartesianVelocity component;
component.initKinematics(urdf_xml, "gen3_base_link", "gen3_end_effector_link");
```

#### ExplorerCartesianVelocity
**Robot**: Custom Explorer platform

Generic Cartesian velocity interface for custom robot implementations.

**Command Interfaces**:
- 6DOF Cartesian twist commands (configurable naming)

### Joint Position Interfaces

Direct joint-space position control with trajectory support.

#### GenericJointPosition
**Control Mode**: Joint position commands

Provides joint-space position control with configurable joint limits and safety bounds.

**Features**:
- Multi-joint position command handling
- Joint type awareness (revolute, continuous, prismatic)
- Configurable velocity and position limits
- Trajectory validation and safety checking

**Supported Joint Types**:
- `REVOLUTE`: Rotational joints with position limits
- `CONTINUOUS`: Unlimited rotational joints
- `PRISMATIC`: Linear joints
- `FIXED`: Immovable joints
- `FLOATING`: 6DOF free joints
- `PLANAR`: 2D planar motion joints

## Forward Kinematics

All interfaces inherit KDL-based forward kinematics from `GenericComponent`, providing real-time end-effector pose computation.

### Requirements

- **URDF Source**: Robot description via `/robot_description` parameter or direct XML
- **Frame Names**: Base and tool frame names matching URDF
- **Joint States**: Current joint positions via `/joint_states` topic

### Usage

```cpp
// Initialize kinematics
component.initKinematics(urdf_xml, "base_frame", "tool_frame");

// Get current end-effector pose
robot_interfaces::CartesianPosition pose = component.getCurrentEndEffectorPose();
```

### Features

- **Real-time Computation**: Optimized for control loop execution
- **Quaternion Normalization**: Automatic normalization and hemisphere continuity
- **Thread Safety**: Safe access from multiple threads
- **Error Handling**: Graceful degradation on missing data
- **Debug Logging**: Configurable diagnostic output

## Factory Interface

The library provides a unified factory function for creating robot components:

```cpp
#include "robot_interfaces/robot_interfaces_algos.hpp"

// Create interface by robot type
auto component = robot_interfaces::create_robot_component("franka_velocity");
auto component = robot_interfaces::create_robot_component("kinova_velocity");
auto component = robot_interfaces::create_robot_component("explorer_velocity");
```

**Supported Robot Types**:
- `"franka_velocity"` - Franka Emika Cartesian velocity control
- `"kinova_velocity"` - Kinova Gen3 Cartesian velocity control
- `"explorer_velocity"` - Explorer platform Cartesian velocity control
- `"joint_position"` - Generic  joint position control

## Dependencies

- **ROS2**: `rclcpp`, `geometry_msgs`, `hardware_interface`, `controller_interface`
- **KDL**: Orocos KDL for kinematics (via `orocos_kdl_vendor`)
- **Eigen3**: Linear algebra library
- **URDF**: Robot description parsing

## Installation

1. Clone the package into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url>
   ```

2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build --packages-select robot_interfaces --symlink-install
   ```

## Testing

### Unit Tests

The package includes comprehensive unit tests for forward kinematics:

```bash
# Build tests
colcon build --packages-select robot_interfaces

# Run FK tests for Kinova Gen3
./build/robot_interfaces/test_generic_fk_unit

# Run FK tests for Franka FR3
source /path/to/franka_ws/install/setup.bash
TEST_ROBOT_NAME=fr3 \
TEST_BASE_FRAME=base \
TEST_EE_FRAME=fr3_hand_tcp \
TEST_NUM_JOINTS=7 \
TEST_JOINT_PREFIX=fr3_joint \
./build/robot_interfaces/test_generic_fk_unit
```

### Test Configuration

Environment variables for custom robot testing:

- `TEST_ROBOT_NAME`: Robot identifier (default: `gen3_2f85`)
- `TEST_BASE_FRAME`: Base frame name (default: `gen3_base_link`)
- `TEST_EE_FRAME`: End-effector frame name (default: `gen3_end_effector_link`)
- `TEST_NUM_JOINTS`: Number of joints (default: `7`)
- `TEST_JOINT_PREFIX`: Joint name prefix (default: `gen3_joint_`)

## Extending the Library

### Adding New Robot Interfaces

1. Create a new class inheriting from `GenericComponent`
2. Implement robot-specific command interfaces
3. Add factory support in `robot_interfaces_algos.cpp`
4. Update documentation and tests

### Custom Control Modes

1. Extend `GenericComponent` with new command types
2. Implement mode-specific logic
3. Add appropriate state and command interfaces
4. Update factory and configuration systems

## Future Development

- **Inverse Kinematics**: KDL-based IK solver integration
- **Collision Detection**: Real-time collision checking
- **Trajectory Planning**: Built-in trajectory generation
- **Multi-Robot Coordination**: Support for coordinated multi-arm systems
- **Simulation Interfaces**: Enhanced Gazebo integration
