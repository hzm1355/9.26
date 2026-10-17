cmake_minimum_required(VERSION 3.8)
project(v2x_demo)

find_package(ament_cmake REQUIRED)

# 安装资源目录：launch / worlds / models / config / web
install(
  DIRECTORY launch worlds models config web
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 节点（信号灯协调 / 区域触发 / 多车交通流 / 自适应配时 / 交通灯记录）
install(
  PROGRAMS
    v2x_demo/traffic_light_controller.py
    v2x_demo/traffic_light_manager.py
    v2x_demo/adaptive_timing_node.py
    v2x_demo/traffic_light_recorder.py
    v2x_demo/signal_drive_node.py
    v2x_demo/region_trigger_node.py
    v2x_demo/region_trigger_selftest.py
    v2x_demo/traffic_flow_node.py
    v2x_demo/v2x_dashboard.py
    v2x_demo/traffic_light_selftest.py
    v2x_demo/websocket_bridge.py
    v2x_demo/plate_gen_node.py
    v2x_demo/plate_recognizer_node.py
    v2x_demo/plate_recorder_node.py
    v2x_demo/speed_detect_node.py
    v2x_demo/speed_recorder_node.py
    v2x_demo/speed_selftest.py
    v2x_demo/parking_manager_node.py
    v2x_demo/parking_manager_selftest.py
    v2x_demo/parking_recorder_node.py
    v2x_demo/parking_recorder_selftest.py
    v2x_demo/parking_shuttle_node.py
    v2x_demo/parking_shuttle_selftest.py
  DESTINATION lib/${PROJECT_NAME}
)

# plate_layout.py 是被上面两个节点 import 的库，不是可执行入口，所以走 install(FILES)。
# 必须落到和节点同一个目录：ros2 run 执行的是 install/lib/v2x_demo/*.py，
# Python 只会把该脚本所在目录放进 sys.path。
install(
  FILES v2x_demo/plate_layout.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
