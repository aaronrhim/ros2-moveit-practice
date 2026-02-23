cmake_minimum_required(VERSION 3.8)
project(aaron_ros2)

# 1. Find all your dependencies (must match package.xml)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 2. Tell CMake where your header files are
include_directories(include)

# 3. Create an executable from your .cpp file
add_executable(aarons_node src/moveit_control_2.cpp)

# 4. Link the dependencies to your executable
ament_target_dependencies(aarons_node
  rclcpp
  std_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
  trajectory_msgs
  rover_msgs
  moveit_servo
  moveit_ros_planning_interface
)

# 5. Install the executable so ros2 run can find it
install(TARGETS aarons_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()