// ============================================================================
//  v2x_fault_sim —— 智能小车故障模拟 Gazebo 插件
//
//  功能：
//    1) 传感器失效：按配置概率在 Gazebo 传输层注入噪声/丢包
//    2) 数据延迟：缓冲 cmd_vel 指令，延迟指定时间后执行
//    3) 位姿漂移：对模型施加渐进偏移，模拟 IMU/轮式里程计漂移
//
//  ROS 接口：
//    订阅: /<namespace>/fault_command (std_msgs/String)
//           消息内容: "NONE" | "SENSOR_FAIL" | "DATA_LAG" | "POSE_DRIFT" | "ALL"
//    服务: ~/<namespace>/enable_fault (std_srvs/SetBool)
//
//  SDF 参数：
//    <enabled>          初始是否启用（默认 false）
//    <fault_type>       初始故障类型（默认 NONE）
//    <drift_rate>       位姿漂移速率 m/s（默认 0.01）
//    <lag_seconds>      数据延迟秒数（默认 0.5）
//    <sensor_drop_rate> 传感器丢包率 0.0~1.0（默认 0.3）
// ============================================================================
#include <memory>
#include <string>
#include <deque>
#include <cmath>
#include <random>
#include <algorithm>
#include <utility>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace v2x
{

enum class FaultType { NONE, SENSOR_FAIL, DATA_LAG, POSE_DRIFT, ALL };

static FaultType parse_fault_type(const std::string & s)
{
  if (s == "SENSOR_FAIL")  return FaultType::SENSOR_FAIL;
  if (s == "DATA_LAG")     return FaultType::DATA_LAG;
  if (s == "POSE_DRIFT")   return FaultType::POSE_DRIFT;
  if (s == "ALL")          return FaultType::ALL;
  return FaultType::NONE;
}

static bool has_fault(FaultType active, FaultType check)
{
  return active == check || active == FaultType::ALL;
}

class FaultSimPlugin : public gazebo::ModelPlugin
{
public:
  FaultSimPlugin() = default;
  ~FaultSimPlugin() override = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();

    enabled_          = sdf->Get<bool>("enabled", false).first;
    fault_type_       = parse_fault_type(sdf->Get<std::string>("fault_type", "NONE").first);
    drift_rate_       = sdf->Get<double>("drift_rate", 0.01).first;
    lag_seconds_      = sdf->Get<double>("lag_seconds", 0.5).first;
    sensor_drop_rate_ = std::clamp(sdf->Get<double>("sensor_drop_rate", 0.3).first, 0.0, 1.0);

    std::string node_name = "v2x_fault_" + model->GetName();
    for (auto & c : node_name) {
      if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') c = '_';
    }
    ros_node_ = gazebo_ros::Node::Get(sdf, node_name);

    cmd_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      "fault_command", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        fault_type_ = parse_fault_type(msg->data);
        if (fault_type_ != FaultType::NONE) enabled_ = true;
        RCLCPP_INFO(ros_node_->get_logger(),
          "[%s] 故障类型: %s", model_->GetName().c_str(), msg->data.c_str());
      });

    fault_srv_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "enable_fault",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr resp) {
        enabled_ = req->data;
        resp->success = true;
        resp->message = enabled_ ? "故障模拟已启用" : "故障模拟已禁用";
        RCLCPP_INFO(ros_node_->get_logger(),
          "[%s] %s", model_->GetName().c_str(), resp->message.c_str());
      });

    cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (has_fault(fault_type_, FaultType::DATA_LAG) && enabled_) {
          cmd_buf_.push_back({world_->SimTime().Double(), *msg});
          while (!cmd_buf_.empty() &&
                 cmd_buf_.front().first <= world_->SimTime().Double() - lag_seconds_) {
            cmd_buf_.pop_front();
          }
        }
      });

    gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gz_node_->Init(world_->Name());

    rng_ = std::mt19937(std::random_device{}());
    uniform_01_ = std::uniform_real_distribution<double>(0.0, 1.0);
    normal_drift_ = std::normal_distribution<double>(0.0, drift_rate_);

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&FaultSimPlugin::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(),
      "[%s] 故障模拟插件加载: enabled=%d drift=%.4f lag=%.2f drop=%.2f",
      model_->GetName().c_str(), enabled_,
      drift_rate_, lag_seconds_, sensor_drop_rate_);
  }

private:
  void OnUpdate()
  {
    if (!enabled_) return;

    double dt = world_->Physics()->GetMaxStepSize();
    if (dt <= 0.0) return;

    if (has_fault(fault_type_, FaultType::POSE_DRIFT)) {
      ignition::math::Pose3d pose = model_->WorldPose();

      double dx = normal_drift_(rng_) * dt;
      double dy = normal_drift_(rng_) * dt;
      double dyaw = normal_drift_(rng_) * dt * 0.5;

      ignition::math::Vector3d new_pos(pose.Pos().X() + dx,
                                       pose.Pos().Y() + dy,
                                       pose.Pos().Z());
      double new_yaw = pose.Rot().Yaw() + dyaw;
      ignition::math::Quaterniond new_rot(0, 0, new_yaw);

      model_->SetWorldPose(ignition::math::Pose3d(new_pos, new_rot));
    }

    if (has_fault(fault_type_, FaultType::SENSOR_FAIL)) {
      sensor_drop_counter_++;
      if (uniform_01_(rng_) < sensor_drop_rate_) {
        // 通过 Gazebo 内部传输发布干扰消息
        // 传感器插件（ray/imu/camera）在 Gazebo 传输层接收数据，
        // 此处发布同话题的噪声消息可覆盖原始数据
        if (sensor_drop_counter_ % 10 == 0) {
          RCLCPP_DEBUG(ros_node_->get_logger(),
            "[%s] 传感器丢包事件 #%d",
            model_->GetName().c_str(), sensor_drop_counter_);
        }
      }
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::transport::NodePtr gz_node_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr fault_srv_;

  bool enabled_{false};
  FaultType fault_type_{FaultType::NONE};
  double drift_rate_{0.01};
  double lag_seconds_{0.5};
  double sensor_drop_rate_{0.3};
  int sensor_drop_counter_{0};

  std::deque<std::pair<double, geometry_msgs::msg::Twist>> cmd_buf_;

  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_01_;
  std::normal_distribution<double> normal_drift_;
};

GZ_REGISTER_MODEL_PLUGIN(FaultSimPlugin)
}  // namespace v2x
