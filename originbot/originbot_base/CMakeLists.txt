cmake_minimum_required(VERSION 3.5)
project(originbot_base)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(originbot_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${tf2_geometry_msgs_INCLUDE_DIRS}   # 确保 tf2_geometry_msgs 头文件可用
)

# Executable
add_executable(originbot_base src/originbot_base.cpp)

# Link dependencies
ament_target_dependencies(originbot_base
  rclcpp
  std_srvs
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  serial
  tf2
  tf2_geometry_msgs
  tf2_ros
  originbot_msgs
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install executable
install(TARGETS
  originbot_base
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install 标定/自检工具脚本（脚本本体装在 share/，可直接 python3 运行或拷走单独用）
# 排除 __pycache__：直接跑过 tools/ 里的脚本后本地会生成 .pyc，
# install(DIRECTORY) 会把整个目录拷进去，没必要污染安装目录。
install(
  DIRECTORY tools
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# 为每个工具脚本生成一个启动器装到 lib/${PROJECT_NAME}/，
# 这样 `ros2 run originbot_base <脚本名>` 才能找到它
# （ros2 run 只搜索 lib/<pkg>/，不搜索 share/<pkg>/）。
#
# 注意 calib_common.py 是共享模块、不是可执行工具，故**故意不列在这里**：
# 它由同级脚本 import 生效（启动器用 python3 <share>/tools/<脚本> 运行，
# sys.path[0] 就是那个目录）。单独拷走某个工具时请连它一起拷，
# 否则该工具会自行退化为"只能人工测量"的模式（不会崩）。
set(_tool_scripts
  imu_check.py
  odom_calib.py
  lidar_crop_check.py
  vth_calib_measure.py
)
foreach(_tool ${_tool_scripts})
  set(TOOLS_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/tools")
  set(SCRIPT "${_tool}")
  configure_file(
    tools/run_tool.sh.in
    ${CMAKE_CURRENT_BINARY_DIR}/launcher_${_tool}
    @ONLY
  )
  install(
    PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/launcher_${_tool}
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${_tool}
  )
endforeach()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export include directories
ament_export_include_directories(include)

# Finalize package
ament_package()
