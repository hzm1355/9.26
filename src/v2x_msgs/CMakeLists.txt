cmake_minimum_required(VERSION 3.8)
project(v2x_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RegionEvent.msg"
  "msg/RegionState.msg"
  "msg/TrafficLightState.msg"
  "msg/TrafficLightStats.msg"
  "msg/PlateCapture.msg"
  "msg/PlateResult.msg"
  "msg/PlateAlert.msg"
  "msg/SpeedCapture.msg"
  "msg/SpeedSegment.msg"
  "msg/SpeedAlert.msg"
  "msg/SpeedStats.msg"
  "msg/BarrierState.msg"
  "msg/ParkingPass.msg"
  "msg/ParkingAction.msg"
  "msg/EtcIdentify.msg"
  "msg/EtcPass.msg"
  "msg/EtcDeviceState.msg"
  "srv/SetPlate.srv"
  "srv/SetBlacklist.srv"
  "srv/SetSpeedZone.srv"
  "srv/SetParkingGate.srv"
  "srv/SetParkingWhitelist.srv"
  "srv/ParkingQuery.srv"
  "srv/ParkingExport.srv"
  "srv/SetEtcParams.srv"
  "srv/EtcQuery.srv"
  "srv/EtcExport.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
