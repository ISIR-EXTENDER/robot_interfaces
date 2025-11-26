cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### ROS2 Control dependencies
set(ROS2_CONTROL_DEPS
  hardware_interface
  controller_interface)

### Franka dependencies
set(FRANKA_DEPS
  Franka
  franka_hardware
  franka_msgs)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

### ROS2 messages
find_package(geometry_msgs REQUIRED)

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
  src/generic_component.cpp
  src/robot_interfaces_algos.cpp)


target_include_directories(
  robot_interfaces PRIVATE include ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(robot_interfaces Franka::Franka Eigen3::Eigen)

ament_target_dependencies(robot_interfaces Eigen3
                          geometry_msgs ${ROS2_CONTROL_DEPS} ${FRANKA_DEPS})

install(TARGETS robot_interfaces DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(robot_interfaces)
ament_export_dependencies(Eigen3 geometry_msgs ${ROS2_CONTROL_DEPS} ${FRANKA_DEPS})

ament_package()
