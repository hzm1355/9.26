// ============================================================================
//  v2x_traffic_light_plugin —— 由 ROS 话题驱动的信号灯"视觉点亮"Gazebo 模型插件
//
//  支持两种灯型：
//    1) 机动车信号灯（默认）：红/黄/绿三态，visual 名为 red/yellow/green
//    2) 行人信号灯（is_pedestrian=true）：停止/行走/闪烁三态，visual 名为
//       red_hand / green_walking
//
//  机制（对齐 Gazebo 官方 LedPlugin）：
//    本插件运行于 gzserver，订阅 ROS std_msgs/String 灯态话题，
//    构造 gazebo::msgs::Visual 并发布到 Gazebo 内部 "~/visual" 话题；
//    gzclient 渲染端据此更新对应灯球 visual 的 emissive（自发光）颜色。
//
//  SDF 参数：
//    <state_topic>    ROS 灯态话题（默认 traffic_light/state）
//    <link_name>      承载灯球 visual 的 link 名（默认 link）
//    <is_pedestrian>  是否为行人信号灯（默认 false）
// ============================================================================
#include <memory>
#include <string>
#include <map>
#include <cctype>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>

#include <ignition/math/Color.hh>
#include <std_msgs/msg/string.hpp>

namespace v2x
{
class TrafficLightPlugin : public gazebo::ModelPlugin
{
public:
  TrafficLightPlugin() = default;
  ~TrafficLightPlugin() override = default;

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model_ = _model;

    state_topic_ = _sdf->Get<std::string>("state_topic", "traffic_light/state").first;
    link_name_   = _sdf->Get<std::string>("link_name", "link").first;
    is_pedestrian_ = _sdf->Get<bool>("is_pedestrian", false).first;

    gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gz_node_->Init(model_->GetWorld()->Name());
    vis_pub_ = gz_node_->Advertise<gazebo::msgs::Visual>("~/visual");

    gazebo::physics::LinkPtr link = model_->GetLink(link_name_);
    if (!link) {
      auto links = model_->GetLinks();
      if (!links.empty()) link = links[0];
    }
    if (link) {
      gazebo::msgs::Link lmsg;
      link->FillMsg(lmsg);
      for (int i = 0; i < lmsg.visual_size(); ++i) {
        const gazebo::msgs::Visual & v = lmsg.visual(i);
        std::string nm = v.name();
        std::string leaf = nm.substr(nm.find_last_of(":") + 1);
        if (is_pedestrian_) {
          if (leaf == "red_hand" || leaf == "green_walking") {
            templates_[leaf] = v;
            has_visuals_ = true;
          }
        } else {
          if (leaf == "red" || leaf == "yellow" || leaf == "green") {
            templates_[leaf] = v;
            has_visuals_ = true;
          }
        }
      }
    }

    std::string node_name = "v2x_tl_" + model_->GetName();
    for (auto & c : node_name) {
      if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') c = '_';
    }
    ros_node_ = gazebo_ros::Node::Get(_sdf, node_name);
    sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      state_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&TrafficLightPlugin::on_state, this, std::placeholders::_1));

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&TrafficLightPlugin::on_update, this));

    RCLCPP_INFO(ros_node_->get_logger(),
      "[v2x_traffic_light] model=%s topic=%s pedestrian=%d visuals=%d",
      model_->GetName().c_str(), state_topic_.c_str(),
      is_pedestrian_ ? 1 : 0, has_visuals_ ? 1 : 0);

    apply(is_pedestrian_ ? "DONT_WALK" : "RED");
  }

private:
  ignition::math::Color vehicle_color(const std::string & phase, const std::string & lamp)
  {
    if (phase == lamp) {
      if (lamp == "red")    return ignition::math::Color(1.0f, 0.05f, 0.05f, 1.0f);
      if (lamp == "yellow") return ignition::math::Color(1.0f, 0.8f, 0.0f, 1.0f);
      if (lamp == "green")  return ignition::math::Color(0.1f, 1.0f, 0.15f, 1.0f);
    }
    return ignition::math::Color(0.0f, 0.0f, 0.0f, 1.0f);
  }

  ignition::math::Color ped_color(const std::string & phase, const std::string & lamp)
  {
    if (phase == "WALK" && lamp == "green_walking")
      return ignition::math::Color(0.1f, 1.0f, 0.15f, 1.0f);
    if (phase == "DONT_WALK" && lamp == "red_hand")
      return ignition::math::Color(1.0f, 0.05f, 0.05f, 1.0f);
    if (phase == "FLASHING") {
      if (lamp == "red_hand") {
        double t = model_->GetWorld()->SimTime().Double();
        float blink = (std::fmod(t, 1.0) < 0.5) ? 1.0f : 0.0f;
        return ignition::math::Color(blink, 0.05f * blink, 0.05f * blink, 1.0f);
      }
    }
    return ignition::math::Color(0.0f, 0.0f, 0.0f, 1.0f);
  }

  void apply(const std::string & phase_upper)
  {
    current_ = phase_upper;
    if (!has_visuals_ || !vis_pub_) return;

    if (is_pedestrian_) {
      for (const char * lamp : {"red_hand", "green_walking"}) {
        auto it = templates_.find(lamp);
        if (it == templates_.end()) continue;
        gazebo::msgs::Visual v = it->second;
        gazebo::msgs::Set(v.mutable_material()->mutable_emissive(),
                          ped_color(phase_upper, lamp));
        vis_pub_->Publish(v);
      }
    } else {
      for (const char * lamp : {"red", "yellow", "green"}) {
        auto it = templates_.find(lamp);
        if (it == templates_.end()) continue;
        gazebo::msgs::Visual v = it->second;
        gazebo::msgs::Set(v.mutable_material()->mutable_emissive(),
                          vehicle_color(phase_upper, lamp));
        vis_pub_->Publish(v);
      }
    }
  }

  void on_state(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string s = msg->data;
    for (auto & ch : s) ch = static_cast<char>(::toupper(ch));

    if (is_pedestrian_) {
      if (s != "WALK" && s != "DONT_WALK" && s != "FLASHING") return;
    } else {
      if (s != "RED" && s != "YELLOW" && s != "GREEN") return;
    }
    if (s == current_) return;
    apply(s);
  }

  void on_update()
  {
    double t = model_->GetWorld()->SimTime().Double();
    if (t - last_republish_ < 0.5) return;
    last_republish_ = t;
    if (has_visuals_ && vis_pub_) {
      apply(current_);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::PublisherPtr vis_pub_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  std::string state_topic_;
  std::string link_name_;
  bool is_pedestrian_{false};
  std::map<std::string, gazebo::msgs::Visual> templates_;
  bool has_visuals_{false};
  std::string current_{"RED"};
  double last_republish_{-1e9};
};

GZ_REGISTER_MODEL_PLUGIN(TrafficLightPlugin)
}  // namespace v2x
