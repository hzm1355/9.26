// ============================================================================
//  v2x_lighting —— 任务三（三）卡口识别的光照预设插件（WorldPlugin）
//
//  订阅 ROS2 /gate/lighting（std_msgs/String），切到 day / backlit / lowlight /
//  night 四套预设，并回一句 /gate/lighting/current 作为到位确认。
//
//  为什么要绕一圈 gz-transport，而不是插件里直接改渲染对象：
//  第一版这里发的是**全局** /scene + /light/modify，日志全绿、四套预设的抓拍图
//  均值却只在 152~162 之间抖，等于什么都没改。原因在命名空间：Gazebo 11 里
//  这两类消息各有消费者，而且都订在带世界名的话题上——
//    · rendering::Scene::OnScene / ProcessSceneMsg   ← ~/scene
//    · rendering::Scene::OnLightModifyMsg / ProcessLightModifyMsg ← ~/light/modify
//    · physics::World::OnLightModifyMsg              ← ~/light/modify
//  全局 /scene 谁都不理（能看到的只有 gzclient 那份同名话题），所以发了个寂寞。
//  transport::Node::Init(worldName) 之后 "~/scene" 解析成
//  /gazebo/v2x_city_scene/scene，才是相机用的那份场景在听的话题。
//
//  另一层道理：Scene 的这些消息不是在回调里立即生效，而是排进队列、等
//  Scene::PreRender()（每次相机出帧前都会走一遍）才落到 OGRE。换句话说走消息
//  这条路天然是渲染线程执行；插件要是从 ROS 回调线程直接调
//  rendering::Scene::SetAmbientColor，就和出帧并发，属于自己找崩。
//
//  四套预设对应条目 3 的"逆光 / 弱光 / 夜间"：
//    day       与 world 里的 <scene>/<light name="sun"> 一致，作基线
//    backlit   天空拉到 0.95、环境光压到 0.10，并把太阳抬到接近水平、转向 -X：
//              北环相机正好朝 +X 看，镜头对着来光方向，车尾（牌面朝 -X）成了
//              背光面——整块欠曝而背景过曝，这才是逆光在图像上的表现，不是加光晕。
//              平行光是全局的，同一时刻只能给一个朝向的卡口制造逆光：此时南环
//              牌面仍顺光，所以逆光档的证据取北环。
//    lowlight  环境光和太阳一起砍到三成、阴影关掉：阴天 / 隧道口
//    night     环境光 0.03、太阳基本熄灭，只留卡口补光灯（world 里龙门架上的
//              point light，挂在相机同侧照着牌面）——电子警察晚上是真带补光的
//
//  补光灯的"关"：这套 msgs::Light 没有 visible 字段，白天把 diffuse/specular 写 0
//  等价于关灯，渲染侧按亮度算就是黑的。
//
//  除了四个具名预设，还收 ad-hoc 形式 custom:a,b,s,sh,f（环境光,天空,太阳,
//  阴影01,补光01）。加这条是因为标定过程中"这一档到底是过曝还是死黑"只能从相机
//  均值看出来，而具名预设编在 .so 里，改一个数就要 colcon + 重启 gzserver 四分钟；
//  有了这条旁路，tools/light_probe.py 一轮采完四档只要十几秒。具名表仍保留，
//  正式测试跑的都是具名档。
//
//  SDF 参数：
//    <default>      启动即套用的预设名（默认 day；填 none 则保持 world 原样）
//    <sun>          太阳灯名（默认 sun）
//    <fill_lights>  夜间点亮的补光灯名，逗号分隔
//    <fill_diffuse> 补光灯亮度（默认 1.0；这版 Gazebo 在 1 附近就夹住了）
// ============================================================================
#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>

namespace v2x
{
class LightingPlugin : public gazebo::WorldPlugin
{
  struct Preset
  {
    const char *name;
    const char *cn;
    double ambient;      // 场景环境光（三通道同值）
    double background;   // 天空球颜色
    double sun;          // 太阳 diffuse
    double dir[3];       // 太阳传播方向（单位化后转成姿态四元数）
    bool shadows;
    bool fill;           // 是否点亮卡口补光灯
  };

  // 数值取自实机标定（tools/light_probe.py 在卡口相机上实测）：本场景的成像基本
  // 由环境光撑着，均值 ≈ 260×ambient，太阳只补顶部受光面，所以档位之间的区分
  // 主要靠 ambient。day 一行必须与 v2x_city_scene.world 里的 <scene> 和
  // <light name="sun"> 一致，否则"套用 day"会把基线悄悄改掉，
  // 后面所有光照对比都失真。day 的方向就是 world 里的 <direction>。
  // 实测均值（北环/南环，tools/light_probe.py --warm 1.2，补光参数定稿后）：
  //   day 163/174，backlit 70/82，lowlight 60/63，night 14/30。
  // night 两行的差别就是补光灯：南环取景里被灯照到的部分多，最亮 180；
  // 同一档把补光关掉（custom:0.03,0.02,0.05,0,0）两环都只剩均值 3、最亮 6。
  // backlit 与 lowlight 的均值只差 10，但靠 background(0.95 的天空)和
  // 低角度太阳分开——逆光要的是"背景过曝+牌面剪影"，不是"整幅更暗"。
  static constexpr Preset PRESETS[] = {
    {"day",      "白天",   0.60, 0.70, 0.80, {-0.50, 0.10, -0.90}, true,  false},
    {"backlit",  "逆光",   0.32, 0.95, 1.30, {-0.85, 0.10, -0.35}, true,  false},
    {"lowlight", "弱光",   0.20, 0.30, 0.30, {-0.30, 0.10, -0.90}, false, false},
    {"night",    "夜间",   0.03, 0.02, 0.05, { 0.00, 0.00, -1.00}, false, true},
  };

public:
  LightingPlugin() = default;
  ~LightingPlugin() override = default;

  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    world_ = _world;
    sun_ = _sdf->Get<std::string>("sun", "sun").first;
    fill_ = _sdf->Get<std::string>("fill_lights",
      "gate_j3_north_fill,gate_j2_south_fill").first;
    fill_diffuse_ = _sdf->Get<double>("fill_diffuse", 1.0).first;

    // 命名空间这一步不能省：解析出来的话题必须是 /gazebo/<world>/scene 和
    // /gazebo/<world>/light/modify，见文件头。
    gzNode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gzNode_->Init(_world->Name());
    scenePub_ = gzNode_->Advertise<gazebo::msgs::Scene>("~/scene", true);
    lightPub_ = gzNode_->Advertise<gazebo::msgs::Light>("~/light/modify", true);

    std::string node_name = "v2x_lighting";
    ros_node_ = gazebo_ros::Node::Get(_sdf, node_name);
    sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      "/gate/lighting", 10,
      std::bind(&LightingPlugin::on_request, this, std::placeholders::_1));
    cur_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
      "/gate/lighting/current", rclcpp::QoS(10).transient_local());

    const std::string def = _sdf->Get<std::string>("default", "day").first;
    if (def != "none") {
      const Preset *p = lookup(def);
      if (p) apply(*p, def);
    }

    RCLCPP_INFO(ros_node_->get_logger(),
      "[v2x_lighting] 预设 day/backlit/lowlight/night，太阳 %s，补光灯 %s，"
      "话题 /gazebo/%s/scene", sun_.c_str(), fill_.c_str(),
      _world->Name().c_str());
  }

private:
  void on_request(const std_msgs::msg::String::SharedPtr _msg)
  {
    std::string name = _msg->data;
    std::transform(name.begin(), name.end(), name.begin(),
      [](unsigned char c) { return std::tolower(c); });

    Preset q{};
    const Preset *p = lookup(name);
    if (!p && name.compare(0, 7, "custom:") == 0 && parse_custom(name.substr(7), q))
      p = &q;

    if (p) {
      apply(*p, name);
    } else if (rclcpp::ok()) {
      RCLCPP_WARN(ros_node_->get_logger(),
        "[v2x_lighting] 无法识别的光照请求 '%s'，可选 day/backlit/lowlight/night"
        " 或 custom:环境光,天空,太阳,阴影01,补光01", _msg->data.c_str());
    }
  }

  static bool parse_custom(const std::string &_spec, Preset &_q)
  {
    std::vector<std::string> f = split(_spec);
    if (f.size() < 3) return false;
    try {
      _q.ambient = std::stod(f[0]);
      _q.background = std::stod(f[1]);
      _q.sun = std::stod(f[2]);
      _q.shadows = f.size() > 3 ? std::stoi(f[3]) != 0 : true;
      _q.fill = f.size() > 4 ? std::stoi(f[4]) != 0 : false;
    } catch (const std::exception &) {
      return false;
    }
    _q.name = "custom";
    _q.cn = "自定义";
    // 方向跟 day 一致：这条旁路只调亮度，太阳方位要变就直接改具名预设。
    _q.dir[0] = -0.50; _q.dir[1] = 0.10; _q.dir[2] = -0.90;
    return true;
  }

  const Preset *lookup(const std::string &_name)
  {
    for (const auto &p : PRESETS)
      if (_name == p.name) return &p;
    return nullptr;
  }

  bool apply(const Preset &_p, const std::string &_ack)
  {
    gazebo::msgs::Scene sc;
    sc.set_name(world_->Name());
    set_color(sc.mutable_ambient(), _p.ambient);
    set_color(sc.mutable_background(), _p.background);
    sc.set_shadows(_p.shadows);
    sc.set_grid(false);
    scenePub_->Publish(sc);

    // 平行光朝向：渲染侧处理的是姿态，所以把传播方向写成 roll/pitch/yaw。
    // 本机这版 gz/math 没有"两向量求四元数"的接口（Quaternion.hh 只有
    // Euler/轴角/矩阵三种构造），而 SDF 姿态的约定是 R = Rz(yaw)·Ry(pitch)·Rx(roll)，
    // 灯的本征光轴是 -Z。取 roll=0 时
    //   R·(0,0,-1) = (-cos(yaw)·sin(pitch), -sin(yaw)·sin(pitch), -cos(pitch))
    // 令它等于单位化的目标方向 dn，就直接得到
    //   pitch = acos(-dn.z)，yaw = atan2(-dn.y, -dn.x)
    // 平行光在 OGRE 里不看位置，写死 0 0 10 与 world 里一致，免得灯被挪走。
    double dx = _p.dir[0], dy = _p.dir[1], dz = _p.dir[2];
    double n = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (n < 1e-6) { dx = 0.0; dy = 0.0; dz = -1.0; n = 1.0; }
    dx /= n; dy /= n; dz /= n;
    ignition::math::Quaterniond rot(
      0.0, std::acos(-dz), std::atan2(-dy, -dx));

    gazebo::msgs::Light sun;
    sun.set_name(sun_);
    // 不发 set_type：灯已经按 SDF 建好了，modify 里重述类型会让渲染侧按新类型
    // 重建 OGRE 灯对象，姿态/朝向有被复位成默认值的风险（spot 复位成垂直向下
    // 就是实测到的现象）。只发改亮度/姿态需要的字段。
    sun.mutable_pose()->mutable_position()->set_x(0.0);
    sun.mutable_pose()->mutable_position()->set_y(0.0);
    sun.mutable_pose()->mutable_position()->set_z(10.0);
    sun.mutable_pose()->mutable_orientation()->set_w(rot.W());
    sun.mutable_pose()->mutable_orientation()->set_x(rot.X());
    sun.mutable_pose()->mutable_orientation()->set_y(rot.Y());
    sun.mutable_pose()->mutable_orientation()->set_z(rot.Z());
    set_color(sun.mutable_diffuse(), _p.sun);
    set_color(sun.mutable_specular(), _p.sun * 0.25);
    sun.mutable_direction()->set_x(dx);
    sun.mutable_direction()->set_y(dy);
    sun.mutable_direction()->set_z(dz);
    sun.set_cast_shadows(_p.shadows);
    sun.set_attenuation_constant(0.9);
    sun.set_attenuation_linear(0.0);
    sun.set_attenuation_quadratic(0.0);
    sun.set_range(1000.0);
    lightPub_->Publish(sun);

    const double v = _p.fill ? fill_diffuse_ : 0.0;
    for (const auto &name : split(fill_)) {
      gazebo::msgs::Light m;
      m.set_name(name);
      set_color(m.mutable_diffuse(), v);
      set_color(m.mutable_specular(), v * 0.4);
      m.set_cast_shadows(false);
      // 衰减必须和 world 里的 <attenuation> 一致，否则一改预设就把灯的物理
      // 行为换掉了。0.002/60 是实测定的：补光灯与相机同在龙门架顶，到牌面约
      // 9.6 m，quadratic 0.02 时衰减到 0.35，开关灯量不出差别；放松到 0.002
      // 剩 0.84 才拍得到。亮度本身加不上去（1.8 与 30 实测无差别，这版在 1
      // 附近就夹住了），所以只能动距离项。
      m.set_attenuation_constant(1.0);
      m.set_attenuation_linear(0.0);
      m.set_attenuation_quadratic(0.002);
      m.set_range(60.0);
      lightPub_->Publish(m);
    }

    current_ = _ack;
    if (rclcpp::ok()) {
      std_msgs::msg::String ack;
      ack.data = _ack;
      cur_pub_->publish(ack);
      RCLCPP_INFO(ros_node_->get_logger(), "[v2x_lighting] 已切换到 %s（%s）"
        " 环境光%.2f 天空%.2f 太阳%.2f 阴影%d 补光%.2f",
        _ack.c_str(), _p.cn, _p.ambient, _p.background, _p.sun,
        _p.shadows ? 1 : 0, v);
    }
    return true;
  }

  static void set_color(gazebo::msgs::Color *_c, double _v)
  {
    _c->set_r(_v); _c->set_g(_v); _c->set_b(_v); _c->set_a(1.0);
  }

  static std::vector<std::string> split(const std::string &_s)
  {
    std::vector<std::string> out;
    std::stringstream ss(_s);
    std::string item;
    while (std::getline(ss, item, ',')) {
      item.erase(0, item.find_first_not_of(" \t"));
      item.erase(item.find_last_not_of(" \t") + 1);
      if (!item.empty()) out.push_back(item);
    }
    return out;
  }

  gazebo::physics::WorldPtr world_;
  gazebo::transport::NodePtr gzNode_;
  gazebo::transport::PublisherPtr scenePub_;
  gazebo::transport::PublisherPtr lightPub_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cur_pub_;

  std::string sun_, fill_, current_;
  double fill_diffuse_ = 1.8;
};

GZ_REGISTER_WORLD_PLUGIN(LightingPlugin)
}  // namespace v2x
