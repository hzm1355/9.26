// ============================================================================
//  v2x_plate —— 任务三（三）道路卡口车牌识别仿真：虚拟车牌贴牌插件
//
//  运行于 gzserver。车牌纹理由 plate_gen_node（Python/PIL）离线渲染，本插件只做
//  两件事：① 让 OGRE 注册新材质 ② 把材质换到车辆 chassis 上预声明的 plate_* 面片。
//
//  为什么必须是这两步（实测结论，别再走回头路）：
//    · OGRE 只在"目录首次被注册"时扫描其中的文件，往已注册过的目录里新增
//      .material 会静默失败 -> 每张车牌必须放独立目录；
//    · 同名覆盖 PNG 无效（按路径缓存纹理）；
//    · 外部进程向 /world/<w>/visual 发布无人接收，运行时新建 visual 也不显示，
//      所以面片必须预先声明在车辆 SDF 里，只换材质不换几何。
//
//  注册材质靠 spawn 一个"材质载体"模型（埋在地下），它和车辆无关节点，
//  走的是 plate_test 实验里已验证可行的运行时加载路径。
//
//  SDF 参数：
//    <set_topic>     ROS 换牌话题（默认 /plate/set）
//    <plate_dir>     车牌纹理根目录（默认 /tmp/v2x_plates）
//    <chassis_link>  车牌面片所在 link（默认 chassis）
//
//  话题载荷（std_msgs/String）： "<模型名>|<rear|front>|<车牌序号>"
//    例： car_1|rear|p0007  ->  加载 <plate_dir>/p0007/p0007.material 的 v2x/p0007
//    序号为 "-" 时还原成占位材质。
// ============================================================================
#include <cctype>
#include <chrono>
#include <cstdio>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>

#include <OgreMaterialManager.h>
#include <OgreResourceGroupManager.h>

#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>

namespace v2x
{
class PlatePlugin : public gazebo::ModelPlugin
{
public:
  PlatePlugin() = default;
  ~PlatePlugin() override = default;

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model_ = _model;
    world_ = _model->GetWorld();
    set_topic_ = _sdf->Get<std::string>("set_topic", "/plate/set").first;
    plate_dir_ = _sdf->Get<std::string>("plate_dir", "/tmp/v2x_plates").first;
    chassis_link_ = _sdf->Get<std::string>("chassis_link", "chassis").first;

    gz_node_.reset(new gazebo::transport::Node());
    gz_node_->Init(world_->Name());
    fac_pub_ = gz_node_->Advertise<gazebo::msgs::Factory>("~/factory");
    vis_pub_ = gz_node_->Advertise<gazebo::msgs::Visual>("~/visual");

    std::string node_name = "v2x_plate_" + model_->GetName();
    for (auto & c : node_name) {
      if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') c = '_';
    }
    ros_node_ = gazebo_ros::Node::Get(_sdf, node_name);
    sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      set_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&PlatePlugin::on_set, this, std::placeholders::_1));

    conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&PlatePlugin::on_update, this));
    last_step_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(500);

    RCLCPP_INFO(ros_node_->get_logger(),
      "[v2x_plate] plate_dir=%s topic=%s", plate_dir_.c_str(), set_topic_.c_str());
  }

private:
  struct Pending
  {
    std::string model;         // car_1
    std::string visual_name;   // car_1::chassis::plate_rear
    std::string material;      // v2x/p0007
    std::string carrier;       // 载体模型名：每张车牌一个，绝不复用
    std::string carrier_sdf;   // 载体 SDF
    std::string mat_file;      // .material 绝对路径，发给 gzclient 时带上，客户端可自行注册
    int retries = 0;
  };

  // ROS 回调跑在 gazebo_ros 的独立线程上，这里只解析入队，
  // 任何 physics::World 访问都留到 world 线程（on_update）里做。
  void on_set(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & raw = msg->data;
    size_t a = raw.find('|');
    size_t b = raw.find('|', a == std::string::npos ? 0 : a + 1);
    if (a == std::string::npos || b == std::string::npos) {
      RCLCPP_WARN(ros_node_->get_logger(),
        "[v2x_plate] 载荷应为 模型|rear|序号，收到: %s", raw.c_str());
      return;
    }
    Pending p;
    p.model = raw.substr(0, a);
    const std::string slot = raw.substr(a + 1, b - a - 1);
    const std::string seq = raw.substr(b + 1);
    if (slot != "rear" && slot != "front") {
      RCLCPP_WARN(ros_node_->get_logger(), "[v2x_plate] 未知车牌位置: %s", slot.c_str());
      return;
    }

    p.visual_name = p.model + "::" + chassis_link_ + "::plate_" + slot;
    if (seq == "-" || seq.empty()) {
      // 占位材质在 world 加载时就随车辆 SDF 注册好了，不需要载体
      p.material = "v2x/plate_placeholder";
      p.mat_file = plate_dir_ + "/placeholder/placeholder.material";
    } else {
      p.material = "v2x/" + seq;
      p.mat_file = plate_dir_ + "/" + seq + "/" + seq + ".material";
      p.carrier = "plate_carrier_" + seq;
      p.carrier_sdf = carrier_sdf(p.carrier, p.mat_file, p.material);
    }

    std::lock_guard<std::mutex> lk(inbox_mtx_);
    inbox_.push_back(p);
  }

  // 每帧推进待办队列：生成新载体注册材质 -> 材质就绪后换到面片上
  void on_update()
  {
    {
      std::lock_guard<std::mutex> lk(inbox_mtx_);
      while (!inbox_.empty()) {
        pending_.push_back(inbox_.front());
        inbox_.pop_front();
      }
    }

    // 用真实时钟而不是 SimTime 限流：世界暂停时也要能换牌，且仿真倍速不定
    using clock = std::chrono::steady_clock;
    auto now = clock::now();
    if (pending_.empty() || now - last_step_ < std::chrono::milliseconds(100)) return;
    last_step_ = now;

    Pending & p = pending_.front();

    if (world_->ModelByName(p.model) == nullptr) {
      RCLCPP_WARN(ros_node_->get_logger(), "[v2x_plate] 找不到模型: %s", p.model.c_str());
      pending_.pop_front();
      return;
    }

    // 载体只负责"让 OGRE 解析这个 .material"，之后必须一直存在：
    // 它是材质注册状态的持有者，删掉会让下一张牌重新踩目录快照的坑。
    if (!p.carrier.empty() && world_->ModelByName(p.carrier) == nullptr) {
      gazebo::msgs::Factory f;
      f.set_sdf(p.carrier_sdf);
      fac_pub_->Publish(f);
      return;
    }

    if (!material_ready(p.material)) {
      if (++p.retries > 150) {          // 15 s 还没注册上就放弃，避免队列卡死
        RCLCPP_ERROR(ros_node_->get_logger(),
          "[v2x_plate] 材质 %s 注册超时", p.material.c_str());
        pending_.pop_front();
      }
      return;
    }

    apply(p);
    pending_.pop_front();
  }

  void apply(const Pending & p)
  {
    // ① 服务端：传感器渲染用的就是这个场景，直接换材质，立即对相机生效
    gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
    if (scene) {
      gazebo::rendering::VisualPtr v = scene->GetVisual(p.visual_name);
      if (v) {
        v->SetMaterial(p.material, true);
      } else {
        RCLCPP_WARN(ros_node_->get_logger(), "[v2x_plate] 场景内无面片 %s",
                    p.visual_name.c_str());
      }
    }

    // ② 客户端：gzclient 有自己的一份场景，走 ~/visual 让它同步换材质
    gazebo::physics::ModelPtr m = world_->ModelByName(p.model);
    if (m) {
      gazebo::physics::LinkPtr link = m->GetLink(chassis_link_);
      if (link) {
        gazebo::msgs::Link lm;
        link->FillMsg(lm);
        for (int i = 0; i < lm.visual_size(); ++i) {
          if (lm.visual(i).name() != p.visual_name) continue;
          gazebo::msgs::Visual v = lm.visual(i);
          v.clear_material();
          gazebo::msgs::Material::Script *s = v.mutable_material()->mutable_script();
          s->add_uri("file://" + p.mat_file);   // 客户端没有载体模型，靠 uri 自行注册
          s->set_name(p.material);
          vis_pub_->Publish(v, true);
          break;
        }
      }
    }

    RCLCPP_INFO(ros_node_->get_logger(), "[v2x_plate] %s -> %s",
                p.visual_name.c_str(), p.material.c_str());
  }

  // 载体模型是异步生成的，材质要等它被 OGRE 解析完才可用。AUTODETECT 组可省掉
  // 猜 Gazebo 用的是哪个 resource group。
  static bool material_ready(const std::string & mat)
  {
    return !Ogre::MaterialManager::getSingleton().getByName(
      mat, Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME).isNull();
  }

  static std::string carrier_sdf(const std::string & name, const std::string & mat_file,
                                 const std::string & mat_name)
  {
    // 载体本身埋在地下 2 m，只起"让 OGRE 解析这个 .material"的作用
    return "<sdf version='1.6'><model name='" + name + "'><static>true</static>"
           "<pose>0 0 -2 0 0 0</pose><link name='link'><visual name='visual'>"
           "<geometry><box><size>0.02 0.02 0.02</size></box></geometry>"
           "<material><script><uri>file://" + mat_file + "</uri><name>" + mat_name +
           "</name></script></material></visual></link></model></sdf>";
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::PublisherPtr fac_pub_;
  gazebo::transport::PublisherPtr vis_pub_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  gazebo::event::ConnectionPtr conn_;

  std::string set_topic_, plate_dir_, chassis_link_;
  std::mutex inbox_mtx_;
  std::deque<Pending> inbox_;
  std::deque<Pending> pending_;
  std::chrono::steady_clock::time_point last_step_;
};

GZ_REGISTER_MODEL_PLUGIN(PlatePlugin)
}  // namespace v2x
