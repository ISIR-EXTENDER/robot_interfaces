cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#### Options
option(ENABLE_FRANKA "Build with Franka Emika support" OFF)
option(BUILD_TESTING "Build tests" OFF)
### ROS2 Control dependencies
set(ROS2_CONTROL_DEPS
  hardware_interface
  controller_interface)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

### ROS2 messages
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

### Pinocchio
find_package(pinocchio REQUIRED)

### ROS2_control packages
foreach(dep IN ITEMS ${ROS2_CONTROL_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

### Franka
if(ENABLE_FRANKA)
  set(FRANKA_DEPS
    Franka
    franka_hardware
    franka_msgs)

  foreach(dep IN ITEMS ${FRANKA_DEPS})
    find_package(${dep} REQUIRED)
  endforeach()
endif()


### Sources
set(SRCS
  src/kinova_velocity_component.cpp 
  src/generic_component.cpp
  src/robot_interfaces_algos.cpp
  src/joint_position_component.cpp
  src/explorer_velocity_component.cpp
)

if(ENABLE_FRANKA)
    list(APPEND SRCS src/franka_velocity_component.cpp)
endif()

# Create library
add_library(
  robot_interfaces SHARED ${SRCS})


target_include_directories(
  robot_interfaces PRIVATE include ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(robot_interfaces Eigen3::Eigen pinocchio::pinocchio)

ament_target_dependencies(robot_interfaces geometry_msgs sensor_msgs 
                          ${ROS2_CONTROL_DEPS})

# Franka specific linking and definitions
if(ENABLE_FRANKA)
  target_compile_definitions(robot_interfaces PUBLIC BUILD_FRANKA_SUPPORT)

  target_link_libraries(robot_interfaces Franka::Franka)
  ament_target_dependencies(robot_interfaces ${FRANKA_DEPS})
  target_compile_definitions(robot_interfaces PUBLIC ENABLE_FRANKA)
endif()


install(TARGETS robot_interfaces DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(robot_interfaces)

# Base exports
ament_export_dependencies(
  Eigen3 geometry_msgs sensor_msgs pinocchio
  ${ROS2_CONTROL_DEPS}
)

# Conditional exports
if(ENABLE_FRANKA)
  ament_export_dependencies(${FRANKA_DEPS})
endif()

# ==================== Testing ====================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_generic_fk_unit test/test_generic_fk_unit.cpp)
  
  target_include_directories(test_generic_fk_unit PRIVATE include)
  
  target_link_libraries(test_generic_fk_unit 
    robot_interfaces 
    pinocchio::pinocchio
    Eigen3::Eigen
  )

  ament_target_dependencies(test_generic_fk_unit 
    rclcpp 
    sensor_msgs 
    geometry_msgs
    hardware_interface
  )
endif()

ament_package()
