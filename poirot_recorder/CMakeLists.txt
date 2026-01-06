cmake_minimum_required(VERSION 3.8)
project(poirot_recorder)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(poirot_msgs REQUIRED)

# Add executable
add_executable(poirot_recorder_node src/poirot_recorder_node.cpp)
target_include_directories(poirot_recorder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(poirot_recorder_node
  rclcpp::rclcpp
  ${poirot_msgs_TARGETS}
)

# Install executable
install(TARGETS
  poirot_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
