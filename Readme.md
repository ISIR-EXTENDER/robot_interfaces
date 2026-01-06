# Robot Interfaces

Robot hardware interface implementations for ROS 2 control, including Franka and Kinova robot arms.

## Robot Interface Architecture

The `robot_interfaces` package provides hardware interfaces (controllers) that bridge ROS 2 control framework with physical or simulated robots. All interfaces inherit from `GenericComponent`, which provides:
- **KDL-Based Forward Kinematics**: Compute end-effector pose from joint angles (works for any robot with URDF)
- **Joint State Subscription**: Executor-driven subscription to `/joint_states` (no extra spinner)
- **Configurable Frame Names**: Flexible base and tool frame mapping via `setFrameNames()`

### Architecture Overview

```
ROS 2 Control Framework
         ↓
Hardware Interface (GenericComponent + Robot-Specific)
    ├── KDL Forward Kinematics (generic, any robot)
    │   ├── URDF Parsing
    │   └── Chain Extraction (base → tool frame)
    │
    ├── Joint State Management (thread-safe)
    │
    ├── Robot-Specific Commands (velocity, impedance, etc)
    │
    └── Hardware Communication
         ↓
Physical/Simulated Robot
```

**Key Design**:
- **GenericComponent**: Base class with reusable FK and state management for any robot
- **Derived Components** (kinovaCartesianVelocity, frankaCartesianVelocity): Robot-specific command handling

---

## Available Interfaces

### frankaCartesianVelocity
Franka Emika FR3 Cartesian velocity interface - 7-DOF impedance-controlled arm with built-in force/torque sensing. Inherits KDL-based FK from GenericComponent.

### kinovaCartesianVelocity
Kinova Gen3 7-DOF Cartesian velocity interface. Inherits KDL-based FK from GenericComponent for real-time end-effector pose estimation.

---

## kinovaCartesianVelocity Interface

The `kinovaCartesianVelocity` interface is a Cartesian velocity controller for Kinova Gen3 7-DOF robot arms with Robotiq grippers. It provides command interface passthrough and inherits forward kinematics from `GenericComponent`.

### Command Interfaces (6 total)

- `tcp/twist.linear.x`, `tcp/twist.linear.y`, `tcp/twist.linear.z` - Linear velocity (m/s)
- `tcp/twist.angular.x`, `tcp/twist.angular.y`, `tcp/twist.angular.z` - Angular velocity (rad/s)

### FK Configuration for Kinova

To enable end-effector pose feedback, set frames on the component (frames are not read from parameters in this package):

```cpp
robot_interfaces::KinovaCartesianVelocity component;
// Set frames for KDL chain extraction
component.initKinematics(string_urdf_xml, "gen3_base_link", "gen3_end_effector_link");
```

---

## frankaCartesianVelocity Interface

The `frankaCartesianVelocity` interface is a Cartesian velocity controller for Franka FR3 (7-DOF). It provides the same 6D twist command interfaces and inherits forward kinematics from `GenericComponent`.

### Command Interfaces (6 total)

- `vx/cartesian_velocity`, `vy/cartesian_velocity`, `vz/cartesian_velocity` - Linear velocity (m/s)
- `wx/cartesian_velocity`, `wy/cartesian_velocity`, `wz/cartesian_velocity` - Angular velocity (rad/s)

### FK Configuration for FR3

Set frames on the component (frames are not read from parameters in this package):

```cpp
robot_interfaces::FrankaCartesianVelocity component;
// Set frames for KDL chain extraction (Franka FR3)
component.initKinematics(string_urdf_xml, "base", "fr3_hand_tcp");
```

---

## Forward Kinematics (All Robots)

All derived components inherit **KDL-based forward kinematics** from `GenericComponent`. Call `getCurrentEndEffectorPose()` to get the current end-effector pose as a `CartesianPosition` (position + quaternion).

**Requirements**:
- URDF source:
  - Either get it via `/robot_description` parameter (published by `robot_state_publisher`), or
  - provide XML directly via `setRobotDescription(xml)`
- Frames names: both base and the frame you want to control (End-effector)
  - Examples: Gen3 → `gen3_base_link` → `gen3_end_effector_link`; FR3 → `base` → `fr3_hand_tcp`

**How it works**:
4. FK uses current joint positions available through the `state_interfaces` to compute pose; quaternion is normalized and cached

Diagnostics:
- Verifies joint-name alignment between KDL chain and `/joint_states` on init
- Logs FK pose with age of last joint state; quaternion is normalized
- Logging can be toggled via `fk_debug_logs` (default: true)

---

## Testing

### Build

```bash
cd <workspace>
colcon build --packages-select robot_interfaces
```

### FK Unit Tests

**Standalone test** for forward kinematics with 4 generic cases (zero pose, different joint configs, individual joint rotations, multi-joint) plus 1 FR3-specific test.

Supports **any robot** via environment variables:

**Kinova Gen3**:
```bash
cd <workspace>
./build/robot_interfaces/test_generic_fk_unit
# Expected: [PASSED] 4 tests
```

**Franka FR3**:
```bash
cd <workspace>
source <workspace_parent>/franka_ros2_ws/install/setup.bash
TEST_ROBOT_NAME=fr3 \
TEST_BASE_FRAME=base \
TEST_EE_FRAME=fr3_hand_tcp \
TEST_NUM_JOINTS=7 \
TEST_JOINT_PREFIX=fr3_joint \
./build/robot_interfaces/test_generic_fk_unit
# Expected: [PASSED] 5 tests
```

**Configuration**:
- `TEST_ROBOT_NAME`: Robot ID (default: `gen3_2f85`)
- `TEST_BASE_FRAME`: Base frame name (default: `gen3_base_link`)
- `TEST_EE_FRAME`: End-effector frame name (default: `gen3_end_effector_link`)
- `TEST_NUM_JOINTS`: Joint count (default: `7`)
- `TEST_JOINT_PREFIX`: Joint name prefix (default: `gen3_joint_`)

**URDF Discovery**:
- Searches `ROS_PACKAGE_PATH` and `AMENT_PREFIX_PATH`
- Falls back to relative paths (`../kinova_ros2_ws`, `../franka_ros2_ws`)
- Supports both `.urdf` and `.urdf.xacro` files (xacro auto-converted)

**Test Notes**:
- Gen3 zero-pose expectation (asserted in tests, ±1e-3):
  - Translation: [-0.000, -0.025, 1.187]
  - Quaternion (xyzw): [0.000, -0.000, 0.000, 1.000]
- FR3 Home-ish configuration test runs only when `TEST_ROBOT_NAME=fr3`; it is skipped for other robots.

**Note**:
- KDL FK is working fine without `rt_buffer` in these unit tests for both Kinova Gen3 and Franka FR3.

---

## Implementation Details

### GenericComponent (Base Class)

Purpose: reusable FK + state plumbing for any robot. Derived classes focus on command interfaces.

- Initialization:
  - `setNodeInterfaces(node|lifecycle_node)`: Stores node interfaces and immediately creates subscriptions:
    - `/robot_description` (transient local) via LifecycleNode
    - `/joint_states` (SensorDataQoS)
  - Optional parameters read at init: `joint_states_topic` (default: `/joint_states`), `use_rt_joint_buffer` (default: false).
  - Frames are provided via `setFrameNames(base_frame, tool_frame)`; there is no base/tool frame parameter read.
  - URDF can be provided directly with `setRobotDescription(xml)` or via the `/robot_description` subscription.

- Lazy KDL setup (`initializeKDLChain()`):
  - Requires node interfaces and a non-empty URDF string.
  - Parses URDF → builds KDL tree → extracts chain `base_frame → tool_frame`.
  - Instantiates `KDL::ChainFkSolverPos_recursive`, pre-allocates joint arrays.
  - Calls `verifyJointNameAlignment()` to build KDL-to-state index mapping.
  - Allocates RT joint buffer and marks it ready only if `use_rt_joint_buffer` is true.

- FK computation (`computeForwardKinematics()`):
  - Non-blocking; returns last cached pose on errors or missing data.
  - Reads joint states under mutex; prefers precomputed name mapping, falls back to per-call lookup.
  - If RT buffer is enabled and state is fresh (≤ 500 ms), reads DOF values from the RT buffer; otherwise from current positions.
  - Runs KDL FK, normalizes quaternion, enforces hemisphere continuity against last cached pose, caches pose.

- Joint-state handling:
  - `onJointStateReceived()` stores names/positions, updates last-update timestamp, and (optionally) updates the RT buffer using the mapping.
  - Mapping is KDL-order → joint_state indices; mismatches are logged with guidance.

- Diagnostics and logging:
  - `fk_debug_logs` default: true. First joint-state arrival, FK init, joint-name alignment, RT-buffer usage, and throttled FK pose logs (with `joint_state_age`) are emitted.
  - No busy waits/timeouts; relies on controller_manager’s executor for callbacks.

### Derived Components

- `kinovaCartesianVelocity`, `frankaCartesianVelocity`:
  - Provide 6D twist command interfaces and delegate pose to `computeForwardKinematics()`.
  - Use `setFrameNames()` and `setRobotDescription()`/`/robot_description` to enable FK.

---

## Troubleshooting

### "Forward kinematics failed to initialize"

**Problem**: KDL chain initialization fails during FK computation
- **Cause**: Robot URDF not available
- **Solution**: Ensure `robot_state_publisher` is publishing `/robot_description` or call `setRobotDescription(xml)` before calling FK.

### "KDL chain extraction failed for frame names X → Y"

**Problem**: Error message indicates frames don't exist in URDF
- **Cause**: Mismatch between URDF frame names and configured `base_frame`/`tool_frame` parameters
- **Solution**: Check URDF for actual frame names, update parameters accordingly

### "No joint states received"

**Problem**: FK computation returns zero pose
- **Cause**: `/joint_states` topic not being published or joint names don't match URDF
- **Solution**: Verify joint_state_broadcaster is running and publishing correct joint names. The component logs expected vs received joint names to help diagnose mismatches.

---

## Future Work

- **Inverse Kinematics Support**: Add KDL-based IK solver to `GenericComponent` for all derived components
  - Enable joint trajectory generation from Cartesian pose commands
  - Support multiple IK solutions with collision avoidance
  - Implement in `computeInverseKinematics()` method

---

## How to Define Your Own Interface

TBD
