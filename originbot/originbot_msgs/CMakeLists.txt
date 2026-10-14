cmake_minimum_required(VERSION 3.5)
project(originbot_msgs)

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

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 这里声明所有 msg/srv 文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OriginbotStatus.msg"
  "srv/CancelNavigation.srv"
  "srv/ControlSystem.srv"
  "srv/MapFile.srv"
  "srv/MarkerEntry.srv"
  "srv/NavigateToStation.srv"
  "srv/OriginbotBuzzer.srv"
  "srv/OriginbotLed.srv"
  "srv/OriginbotPID.srv"
  "srv/ParamsEntry.srv"
  "srv/RouteGraph.srv"
  "srv/SaveMap.srv"
  "srv/SetAuthority.srv"
  "srv/SetPose.srv"
  "srv/SetInt32.srv"
  "srv/StationEntry.srv"
)

# 这一行必须加，否则外部包（比如 originbot_base）找不到生成的 .hpp
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
