# Robot Interfaces Library

A ROS2 hardware interface library that provides unified robot control abstractions for various robot manipulators. This library bridges the ROS2 control framework with physical and simulated robots through a modular, extensible architecture.

## Overview

The `robot_interfaces` package implements hardware interfaces for ROS2 control, enabling seamless integration between high-level controllers and robot hardware. The library provides:

- **Unified Interface Abstraction**: Common API for different robot types and control modes
- **Kinematics & Dynamics**: Pinocchio-based real-time forward kinematics, Jacobians, mass matrix, and dynamics computation for any URDF-described robot
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
    │   ├── Pinocchio-Based Kinematics & Dynamics (URDF-based)
    │   ├── Joint State Management with syncState()
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

- **Pinocchio-Based Kinematics & Dynamics**: Real-time end-effector pose computation, Jacobians, and dynamics from joint states via Pinocchio algorithms (FK, CRBA, RNEA)
- **Joint State Integration**: Explicit `syncState()` for updating internal Pinocchio model with current sensor data
- **URDF Support**: Automatic model parsing via Pinocchio's URDF parser
- **Frame Management**: Configurable base and tool frame mapping using Pinocchio frame IDs
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
component.initKinematics(urdf_xml, "fr3_hand_tcp");
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
component.initKinematics(urdf_xml, "gen3_end_effector_link");
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

## Kinematics & Dynamics

All interfaces inherit Pinocchio-based kinematics and dynamics from `GenericComponent`, providing real-time end-effector pose computation, Jacobians, and dynamics calculations.

### Core Algorithms

- **Forward Kinematics (FK)**: `pinocchio::forwardKinematics()` for end-effector pose from joint positions
- **Jacobians**: `pinocchio::computeJointJacobians()` for analytical 6×n Jacobian matrix
- **Mass Matrix**: `pinocchio::crba()` (Composite Rigid Body Algorithm) for inertia matrix computation
- **Non-Linear Effects**: `pinocchio::nonLinearEffects()` for gravity and Coriolis vectors (via RNEA with τ=0)
- **Full Dynamics**: `pinocchio::rnea()` (Recursive Newton-Euler Algorithm) for inverse dynamics

### Requirements

- **URDF Source**: Robot description via `/robot_description` parameter or direct XML
- **Frame Names**: tool frame names matching URDF
- **Joint States**: Current joint positions and velocities (updated via `syncState()`)

### Critical: State Synchronization

All forward kinematics and dynamics computations require calling `syncState()` at the start of each control loop to update the internal Pinocchio model with current sensor data:

```cpp
// Per control loop (CRITICAL - must be called before any FK/dynamics queries)
component.syncState();  // Update q, v from hardware interfaces

// Now query kinematics/dynamics
robot_interfaces::CartesianPosition pose = component.getCurrentEndEffectorPose();
Eigen::MatrixXd jacobian = component.getEndEffectorJacobian();
Eigen::MatrixXd mass_matrix = component.getMassMatrix();
Eigen::VectorXd nonlin_effects = component.getNonLinearEffects();
```

**Why `syncState()` is necessary**: Pinocchio requires explicit state synchronization. This allows precise control over when the model is updated, essential for real-time determinism.

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
- **Pinocchio**: Rigid body dynamics library for kinematics and dynamics computation
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