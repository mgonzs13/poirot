cmake_minimum_required(VERSION 3.10)
project(poirot_demos)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(poirot REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(demo_node src/demo_node.cpp)
target_link_libraries(demo_node 
  poirot::poirot
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

# Install executable
install(TARGETS demo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python demo
install(PROGRAMS
  poirot_demos/demo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()