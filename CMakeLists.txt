cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### ROS2 Control dependencies
set(ROS2_CONTROL_DEPS
  hardware_interface
  controller_interface)

############## Franka dependencies
set(FRANKA_DEPS
  Franka
  franka_hardware
  franka_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

### ROS2 messages
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

### KDL and URDF
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

### ROS2_control packages
foreach(dep IN ITEMS ${ROS2_CONTROL_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

### Franka specific dependencies
foreach(dep IN ITEMS ${FRANKA_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

# Create library
add_library(
  robot_interfaces SHARED
  src/franka_velocity_component.cpp
  src/kinova_velocity_component.cpp 
  src/explorer_joint_position_component.cpp
  src/generic_component.cpp
  src/robot_interfaces_algos.cpp)


target_include_directories(
  robot_interfaces PRIVATE include ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(robot_interfaces Franka::Franka Eigen3::Eigen ${orocos_kdl_LIBRARIES})

ament_target_dependencies(robot_interfaces Eigen3 geometry_msgs sensor_msgs 
                          orocos_kdl kdl_parser urdf
                          ${ROS2_CONTROL_DEPS} ${FRANKA_DEPS})

install(TARGETS robot_interfaces DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(robot_interfaces)
ament_export_dependencies(Eigen3 geometry_msgs sensor_msgs orocos_kdl kdl_parser urdf
                          ${ROS2_CONTROL_DEPS} ${FRANKA_DEPS})

# ==================== Testing ====================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Generic FK Unit Test (works for any robot via environment variables)
  ament_add_gtest(test_generic_fk_unit test/test_generic_fk_unit.cpp)
  target_include_directories(test_generic_fk_unit PRIVATE include)
  target_link_libraries(test_generic_fk_unit robot_interfaces)
  ament_target_dependencies(test_generic_fk_unit rclcpp sensor_msgs Eigen3 orocos_kdl kdl_parser urdf)

endif()

ament_package()
