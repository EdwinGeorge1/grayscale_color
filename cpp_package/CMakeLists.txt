cmake_minimum_required(VERSION 3.10)  # Ensure at least CMake 3.10 for ROS 2 compatibility
project(cpp_package)

# Compiler settings (optional but useful for warnings)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

# If testing is enabled, find the linter and testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create executable from source file
add_executable(cpp_node src/cpp_node.cpp)

# Specify C++ standard
target_compile_features(cpp_node PUBLIC cxx_std_17)  # C++17 is a good standard to use

# Link dependencies to the executable
ament_target_dependencies(cpp_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

# Install the node executable
install(TARGETS cpp_node
        DESTINATION lib/${PROJECT_NAME})

# Install the launch files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)


# Declare this as an ament package
ament_package()

