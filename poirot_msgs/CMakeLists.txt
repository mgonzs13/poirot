cmake_minimum_required(VERSION 3.10)
project(poirot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate messages (simplified - only 2 messages)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Data.msg"
  "msg/FunctionCall.msg"
  "msg/FunctionStats.msg"
  "msg/GpuInfo.msg"
  "msg/CpuInfo.msg"
  "msg/SystemInfo.msg"
  "msg/ProcessInfo.msg"
  "msg/ProfilingData.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()