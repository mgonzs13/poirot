cmake_minimum_required(VERSION 3.10)
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
find_package(ament_index_cpp REQUIRED)
find_package(CURL REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/poirot.cpp
  src/utils/sysfs_reader.cpp
  src/utils/thread_metrics.cpp
  src/utils/rapl_monitor.cpp
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
  ament_index_cpp::ament_index_cpp
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp poirot_msgs CURL ament_index_cpp)

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

# Install data files
install(FILES data/iso_country_codes.csv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME})

# Install Python package
ament_python_install_package(${PROJECT_NAME})

ament_package()
