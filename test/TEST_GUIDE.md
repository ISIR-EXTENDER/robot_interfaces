# Forward Kinematics Tests - Quick Start Guide

## Overview

Standalone C++ unit tests for forward kinematics (FK) using KDL with URDF support. Tests work for any robot with proper configuration via environment variables.

**Supports**:
- ✅ Kinova Gen3 (7-DOF)
- ✅ Franka FR3 (7-DOF)
- ✅ Any robot with URDF (via parameters)

> **Note**: Replace `<workspace>` and `<workspace_parent>` with your actual paths throughout this guide.

---

## Quick Start

### Build

```bash
cd <workspace>
colcon build --packages-select robot_interfaces
```

---

## Running Tests

### Kinova Gen3 (Default)

**Run**:
```bash
cd <workspace>
./build/robot_interfaces/test_generic_fk_unit
```

**Expected Output**:
```
=== Robot Configuration ===
Robot: gen3_2f85
Base frame: gen3_base_link
EE frame: gen3_end_effector_link
Joints: 7 (prefix: gen3_joint_)

[==========] 4 tests from GenericRobotFKTest
[  PASSED  ] 4 tests.
```

**Test Coverage**:
1. Zero Pose for Zero Joints - Verify home configuration
2. Different Configs Produce Different Poses - Joint 3 rotation
3. Joint 2 Rotation Changes Pose - Individual joint effect
4. Multiple Joint Rotations - Complex configuration

---

### Franka FR3

**Requirements**:
```bash
source <workspace_parent>/franka_ros2_ws/install/setup.bash
```

**Run**:
```bash
cd <workspace>
source <workspace_parent>/franka_ros2_ws/install/setup.bash
TEST_ROBOT_NAME=fr3 \
TEST_BASE_FRAME=base \
TEST_EE_FRAME=fr3_hand_tcp \
TEST_NUM_JOINTS=7 \
TEST_JOINT_PREFIX=fr3_joint \
./build/robot_interfaces/test_generic_fk_unit
```

**Expected Output**:
```
=== Robot Configuration ===
Robot: fr3
Base frame: base
EE frame: fr3_hand_tcp
Joints: 7 (prefix: fr3_joint)

[==========] 4 tests from GenericRobotFKTest
[  PASSED  ] 4 tests.
```

---

### Test Notes

- Test 1 (Zero Pose):
   - For Kinova Gen3, the expected pose at zero configuration is:
      - Translation: [-0.000, -0.025, 1.187]
      - Quaternion (xyzw): [0.000, -0.000, 0.000, 1.000]
   - The test asserts these within 1e-3 tolerance.
- Test 5 (FR3 Home-ish):
   - This test is FR3-specific and is automatically skipped when `TEST_ROBOT_NAME` is not `fr3`.

---

### Custom Robot

Set environment variables for any robot:

```bash
TEST_ROBOT_NAME=my_robot \
TEST_BASE_FRAME=base_link \
TEST_EE_FRAME=tool_frame \
TEST_NUM_JOINTS=6 \
TEST_JOINT_PREFIX=joint_ \
./build/robot_interfaces/test_generic_fk_unit
```

**Configuration**:
- `TEST_ROBOT_NAME`: Robot identifier (used to find URDF)
- `TEST_BASE_FRAME`: Base frame name in URDF
- `TEST_EE_FRAME`: End-effector frame name in URDF
- `TEST_NUM_JOINTS`: Number of joints to test
- `TEST_JOINT_PREFIX`: Joint name prefix (append numbers: `prefix1`, `prefix2`, etc.)

### Note on TF

This generic test focuses on KDL-based forward kinematics without requiring TF. If you need TF-based validation, consider adding a separate integration test or script outside of this unit test.

---

### URDF Discovery

The test automatically searches for robot URDF files:

1. **ROS_PACKAGE_PATH** - Searches for `<robot_name>.urdf` and `<robot_name>.urdf.xacro`
2. **AMENT_PREFIX_PATH** - Additional workspace locations
3. **Relative Paths** - Falls back to:
   - `<workspace_parent>/kinova_ros2_ws/src/ros2_kortex_ws/src/ros2_kortex/kortex_description/robots/`
   - `<workspace_parent>/franka_ros2_ws/src/franka_description/robots/`

**Supported Formats**:
- `.urdf` files (direct URDF)
- `.urdf.xacro` files (auto-converted via `xacro` command)

**Direct override**:
- Set `TEST_URDF_PATH` to a specific URDF/xacro file to skip discovery.
   Example:
   ```bash
   export TEST_URDF_PATH="<workspace_parent>/kinova_ros2_ws/src/ros2_kortex_ws/install/kortex_description/share/kortex_description/arms/gen3/7dof/urdf/gen3_macro.xacro"
   <workspace>/build/robot_interfaces/test_generic_fk_unit
   ```

---

### Implementation Notes

- KDL FK works without `rt_buffer` in this unit test setup (Kinova Gen3 and Franka FR3 validated).

## Troubleshooting

### "URDF file not found"

**Problem**: Test can't locate robot URDF

**Solution**:
1. Ensure kinova_ros2_ws or franka_ros2_ws is installed at expected path (`<workspace_parent>/`)
2. Verify URDF file exists: `find <workspace_parent>/kinova_ros2_ws -name "*.urdf" -o -name "*.urdf.xacro" | head -5`
3. Check `ROS_PACKAGE_PATH` is set if using custom location

### "KDL chain extraction failed"

**Problem**: Error extracting chain between frames

**Solution**:
1. Verify frame names match URDF. Examples:
   - Kinova Gen3: `TEST_BASE_FRAME=gen3_base_link TEST_EE_FRAME=gen3_end_effector_link ./test`
   - Franka FR3: `TEST_BASE_FRAME=base TEST_EE_FRAME=fr3_hand_tcp ./test`
2. Check URDF has expected frame: `grep -E "(link|frame)" <urdf_file>`
3. Test with known working robot first (Kinova or Franka)

### Tests fail for Franka

**Problem**: FR3 tests fail or hang

**Solution**:
1. Source Franka workspace: `source <workspace_parent>/franka_ros2_ws/install/setup.bash`
2. Verify xacro is installed: `which xacro`
3. Check URDF conversion works: `xacro <workspace_parent>/franka_ros2_ws/src/franka_description/robots/fr3/fr3.urdf.xacro`

---

## Files

- `test_generic_fk_unit.cpp` - Standalone unit tests (C++) ⭐
- `TEST_GUIDE.md` - This file
- `../../../README.md` - Full architecture documentation (top-level)

---

## Test Behavior

**No External Dependencies**:
- ✅ No ROS launch files needed
- ✅ No simulator or physical robot required
- ✅ No parameter server or node spinning
- ✅ Pure unit testing with GTest

**Lazy Initialization**:
- URDF loaded on first FK call
- Joint state subscription created once
- Background thread for processing (cleanup on exit)

**Exit Codes**:
- `0` - All tests passed
- `1` - One or more tests failed
- Use `$?` to check: `./test_generic_fk_unit && echo "SUCCESS" || echo "FAILED"`

---

## Next Steps

- Add custom robots by updating URDF paths
- Integrate into CI/CD pipeline
- Plan: Add inverse kinematics (IK) support via KDL
