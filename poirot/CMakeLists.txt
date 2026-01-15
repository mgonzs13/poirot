cmake_minimum_required(VERSION 3.8)
project(poirot)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(poirot_msgs REQUIRED)
find_package(CURL REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/poirot.cpp
  src/utils/sysfs_reader.cpp
  src/utils/thread_metrics.cpp
  src/utils/process_metrics.cpp
  src/utils/energy_monitor.cpp
  src/utils/hwmon_scanner.cpp
  src/utils/power_estimator.cpp
  src/utils/gpu_monitor.cpp
  src/utils/co2_manager.cpp
  src/utils/system_info_reader.cpp
  src/utils/string_utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CURL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  CURL::libcurl
  rclcpp::rclcpp
  ${poirot_msgs_TARGETS}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp poirot_msgs CURL)

install(DIRECTORY include/
  DESTINATION include/
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME})

# Install Python package
ament_python_install_package(${PROJECT_NAME})

ament_package()
