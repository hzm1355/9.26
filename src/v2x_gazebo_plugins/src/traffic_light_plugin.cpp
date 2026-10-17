// ============================================================================
//  traffic_light_plugin —— 协议 §三.(二) 信号灯配时仿真模块的"插件形态"整合版
//
//  来源：仓库根目录 TrafficLightPlugin.cpp（任务二交付的插件形态），整合进本包时
//  做了三处工程化修正，接口与视觉约定保持不变：
//    1) 灯态字符串大小写兼容（管理器发布 GREEN/RED/YELLOW，交付版只认小写）
//    2) 周期重发视觉消息（gzclient 晚于 gzserver 启动时也能看到当前灯色）
//    3) ROS 接入改用 gazebo_ros::Node（跟随 SDF <ros> 命名空间，无需自建 spin 线程）
//
//  机制：运行于 gzserver，订阅 ROS std_msgs/String 灯态话题，构造
//  gazebo::msgs::Visual 发布到 Gazebo 内部 "~/visual" 话题，gzclient 渲染端
//  据此更新红/黄/绿三个灯球 visual 的材质颜色。
//
//  SDF 参数：
//    <topic>  ROS 灯态话题（默认 /traffic_light/cmd）
//  模型结构约定（与交付版一致）：
//    red_link::red_visual / yellow_link::yellow_visual / green_link::green_visual
// ============================================================================
#include <cctype>
#include <map>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>

#include <std_msgs/msg/string.hpp>

namespace gazebo
{

class TrafficLightPluginSimple : public ModelPlugin
{
public:
  TrafficLightPluginSimple() = default;
  ~TrafficLightPluginSimple() override = default;

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model_ = _model;
    world_ = _model->GetWorld();

    topic_ = _sdf->Get<std::string>("topic", "/traffic_light/cmd").first;

    gz_node_ = transport::NodePtr(new transport::Node());
    gz_node_->Init(world_->Name());
    visual_pub_ = gz_node_->Advertise<msgs::Visual>("~/visual");
    if (!visual_pub_->WaitForConnection(common::Time(2, 0))) {
      gzwarn << "[traffic_light_plugin] " << model_->GetName()
             << ": no visual subscriber yet (gzclient not connected?)\n";
    }

    visuals_["red"]    = model_->GetName() + "::red_link::red_visual";
    visuals_["yellow"] = model_->GetName() + "::yellow_link::yellow_visual";
    visuals_["green"]  = model_->GetName() + "::green_link::green_visual";
    parents_["red"]    = model_->GetName() + "::red_link";
    parents_["yellow"] = model_->GetName() + "::yellow_link";
    parents_["green"]  = model_->GetName() + "::green_link";

    ros_node_ = gazebo_ros::Node::Get(_sdf, "traffic_light_plugin_" + model_->GetName());
    sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      topic_, 10,
      std::bind(&TrafficLightPluginSimple::OnStateMsg, this, std::placeholders::_1));

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TrafficLightPluginSimple::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(),
      "[traffic_light_plugin] model=%s topic=%s", model_->GetName().c_str(), topic_.c_str());
    Apply(current_);
  }

private:
  struct Color { double r, g, b; };

  // 状态 → 各灯球颜色（交付版配色表：点亮灯全亮，其余压暗到 0.05）
  static const std::map<std::string, std::map<std::string, Color>> STATE_COLORS;

  static std::string Lower(std::string s)
  {
    for (auto & c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
  }

  void OnStateMsg(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string state = Lower(msg->data);
    if (STATE_COLORS.find(state) == STATE_COLORS.end()) {
      RCLCPP_WARN(ros_node_->get_logger(), "unknown state: %s", msg->data.c_str());
      return;
    }
    if (state == current_) return;
    current_ = state;
    Apply(current_);
    RCLCPP_INFO(ros_node_->get_logger(), "%s -> %s", model_->GetName().c_str(), state.c_str());
  }

  void Apply(const std::string & state)
  {
    if (!visual_pub_) return;
    auto it = STATE_COLORS.find(state);
    if (it == STATE_COLORS.end()) return;

    msgs::Visual visual_msg;
    for (const auto & entry : it->second) {
      const Color & rgb = entry.second;
      visual_msg.Clear();
      visual_msg.set_name(visuals_[entry.first]);
      visual_msg.set_parent_name(parents_[entry.first]);
      auto * mat = visual_msg.mutable_material();
      auto * amb = mat->mutable_ambient();
      amb->set_r(rgb.r); amb->set_g(rgb.g); amb->set_b(rgb.b); amb->set_a(1);
      auto * dif = mat->mutable_diffuse();
      dif->set_r(rgb.r); dif->set_g(rgb.g); dif->set_b(rgb.b); dif->set_a(1);
      auto * emi = mat->mutable_emissive();
      emi->set_r(rgb.r); emi->set_g(rgb.g); emi->set_b(rgb.b); emi->set_a(1);
      visual_pub_->Publish(visual_msg);
    }
  }

  // gzclient 可能在 gzserver 之后才订阅视觉话题，周期重发保证灯色同步
  void OnUpdate()
  {
    const double t = world_->SimTime().Double();
    if (t - last_republish_ < 0.5) return;
    last_republish_ = t;
    Apply(current_);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  transport::NodePtr gz_node_;
  transport::PublisherPtr visual_pub_;
  event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  std::string topic_;
  std::map<std::string, std::string> visuals_;
  std::map<std::string, std::string> parents_;
  std::string current_{"red"};
  double last_republish_{-1e9};
};

const std::map<std::string, std::map<std::string, TrafficLightPluginSimple::Color>>
TrafficLightPluginSimple::STATE_COLORS = {
  {"red", {
    {"red",    {1, 0, 0}},
    {"yellow", {0.05, 0.05, 0}},
    {"green",  {0, 0.05, 0}}
  }},
  {"green", {
    {"red",    {0.05, 0, 0}},
    {"yellow", {0.05, 0.05, 0}},
    {"green",  {0, 1, 0}}
  }},
  {"yellow", {
    {"red",    {0.05, 0, 0}},
    {"yellow", {1, 1, 0}},
    {"green",  {0, 0.05, 0}}
  }}
};

GZ_REGISTER_MODEL_PLUGIN(TrafficLightPluginSimple)

}  // namespace gazebo
