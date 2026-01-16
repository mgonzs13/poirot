cmake_minimum_required(VERSION 3.10)
project(poirot_tui)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(poirot_msgs REQUIRED)

# Find ncurses
find_package(Curses REQUIRED)

# Add executable
add_executable(poirot_tui
  src/main.cpp
  src/tui_node.cpp
  src/tui_renderer.cpp
  src/data_manager.cpp
)

target_include_directories(poirot_tui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CURSES_INCLUDE_DIR}
)

target_link_libraries(poirot_tui
  ${CURSES_LIBRARIES}
  rclcpp::rclcpp
  ${poirot_msgs_TARGETS}
)

# Install executable
install(TARGETS poirot_tui
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
