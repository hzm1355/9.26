cmake_minimum_required(VERSION 3.8)
project(smart_car_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---- C++ Gazebo 故障模拟插件 ----
add_library(v2x_fault_sim SHARED
  src/v2x_fault_sim_plugin.cc
)
target_include_directories(v2x_fault_sim PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)
target_link_directories(v2x_fault_sim PUBLIC ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(v2x_fault_sim
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)
ament_target_dependencies(v2x_fault_sim
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  gazebo_ros
)
set_target_properties(v2x_fault_sim PROPERTIES PREFIX "lib")

install(TARGETS v2x_fault_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---- 安装资源目录 ----
install(
  DIRECTORY launch worlds config models
  DESTINATION share/${PROJECT_NAME}
)

# ---- 安装 Python 节点 ----
install(
  PROGRAMS
    nodes/car_generator.py
    nodes/smart_car_spawner.py
    nodes/trajectory_recorder.py
    nodes/trajectory_player.py
    nodes/obstacle_avoidance.py
    nodes/smart_car_demo.py
    nodes/car_selftest.py
    nodes/trajectory_selftest.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 安装 shell 脚本 ----
install(
  PROGRAMS
    scripts/run_simulation.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
