cmake_minimum_required(VERSION 3.5)
project(v2x_gazebo_plugins)

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
find_package(v2x_msgs REQUIRED)

add_library(v2x_traffic_light SHARED
  src/v2x_traffic_light_plugin.cc
)
target_include_directories(v2x_traffic_light PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)
target_link_directories(v2x_traffic_light PUBLIC ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(v2x_traffic_light
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)
ament_target_dependencies(v2x_traffic_light
  rclcpp
  std_msgs
  gazebo_ros
)

# Gazebo 以 lib<name>.so 加载插件，产物名需为 libv2x_traffic_light.so
set_target_properties(v2x_traffic_light PROPERTIES PREFIX "lib")

# 协议 §三.(二) 信号灯插件形态（仓库根 TrafficLightPlugin.cpp 整合版）
add_library(traffic_light_plugin SHARED
  src/traffic_light_plugin.cpp
)
target_include_directories(traffic_light_plugin PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)
target_link_directories(traffic_light_plugin PUBLIC ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(traffic_light_plugin
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)
ament_target_dependencies(traffic_light_plugin
  rclcpp
  std_msgs
  gazebo_ros
)
set_target_properties(traffic_light_plugin PROPERTIES PREFIX "lib")

# 任务三（三）车牌贴牌插件
# 插件要直接查 OGRE 材质是否已注册；Gazebo 的 pkg-config 不带这些路径。
# 只取目录，用 GLOB 直接展开文件会让编译器刷 "not a directory" 警告。
file(GLOB OGRE_ENTRIES "/usr/include/OGRE/*")
set(OGRE_SUBDIRS "")
foreach(_e ${OGRE_ENTRIES})
  if(IS_DIRECTORY ${_e})
    list(APPEND OGRE_SUBDIRS ${_e})
  endif()
endforeach()
add_library(v2x_plate SHARED
  src/v2x_plate_plugin.cc
)
target_include_directories(v2x_plate PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  /usr/include/OGRE
  ${OGRE_SUBDIRS}
)
target_link_directories(v2x_plate PUBLIC ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(v2x_plate
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)
ament_target_dependencies(v2x_plate
  rclcpp
  std_msgs
  gazebo_ros
)
set_target_properties(v2x_plate PROPERTIES PREFIX "lib")

# 任务三（三）卡口龙门架抓拍插件
add_library(v2x_gate SHARED
  src/v2x_gate_plugin.cc
)
target_include_directories(v2x_gate PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${v2x_msgs_INCLUDE_DIRS}
)
target_link_directories(v2x_gate PUBLIC ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(v2x_gate
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  ${v2x_msgs_LIBRARIES}
)
ament_target_dependencies(v2x_gate
  rclcpp
  std_msgs
  gazebo_ros
  v2x_msgs
)
set_target_properties(v2x_gate PROPERTIES PREFIX "lib")

# 任务三（三）卡口光照预设插件（WorldPlugin，只碰 std_msgs）
add_library(v2x_lighting SHARED
  src/v2x_lighting_plugin.cc
)
target_include_directories(v2x_lighting PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)
target_link_directories(v2x_lighting PUBLIC ${GAZEBO_LIBRARY_DIRS})
# 光照预设只发 gz-transport 的 ~/scene 与 ~/light/modify，改的是 gzserver 里
# rendering::Scene 的消息队列，不需要链接 gazebo_rendering（它的头在本机还是残缺的，
# rendering/Light.hh 依赖的 BaseLight.hh 根本没装）。
target_link_libraries(v2x_lighting
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)
ament_target_dependencies(v2x_lighting
  rclcpp
  std_msgs
  gazebo_ros
)
set_target_properties(v2x_lighting PROPERTIES PREFIX "lib")

# 任务五（三）道闸动力学插件（ModelPlugin：读 revolute 实测值 + SetForce 位置环）
add_library(v2x_barrier SHARED
  src/v2x_barrier_plugin.cc
)
target_include_directories(v2x_barrier PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${v2x_msgs_INCLUDE_DIRS}
)
target_link_directories(v2x_barrier PUBLIC ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(v2x_barrier
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  ${v2x_msgs_LIBRARIES}
)
ament_target_dependencies(v2x_barrier
  rclcpp
  std_msgs
  gazebo_ros
  v2x_msgs
)
set_target_properties(v2x_barrier PROPERTIES PREFIX "lib")

install(TARGETS v2x_traffic_light traffic_light_plugin v2x_plate v2x_gate v2x_lighting v2x_barrier
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
