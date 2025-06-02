cmake_minimum_required(VERSION 3.5)
project(limo_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/kinematics.py
  scripts/utils.py

  scripts/controller/basic/basic_kinematics.py
  scripts/controller/basic/basic_drive.py

  scripts/controller/ackermann/ackermann_kinematics.py
  scripts/controller/ackermann/ackermann_drive.py

  scripts/odometry.py
  scripts/gps_emulator.py

  scripts/path_tracking/path_tracking_pid.py
  scripts/path_tracking/path_tracking_pure_pursuit.py
  scripts/path_tracking/path_tracking_stanley_control.py

  scripts/cmd_vel.py
  scripts/liveplotter.py
  scripts/pathplotter.py
  scripts/ekfplotter.py

  DESTINATION lib/${PROJECT_NAME}
)


# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node_test src/cpp_node.cpp)
ament_target_dependencies(cpp_node_test rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_node_test
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
