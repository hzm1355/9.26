// ============================================================================
//  v2x_gate —— 任务三（三）道路卡口车牌识别仿真：龙门架抓拍插件
//
//  挂在卡口相机的 camera 传感器上，运行于 gzserver。车辆进入触发半径时抓拍一帧存盘并
//  发布 PlateCapture，保证"图像 / 位姿 / 时刻"三者取自同一次触发。
//
//  为什么用 SensorPlugin 而不是 ModelPlugin + 订阅图像话题：
//    · CamImage() 拿到的就是当前这一帧的显存回读结果，配传感器时间戳是真正的
//      帧精确抓拍；绕一圈 DDS 订阅会引入不确定的帧延迟。
//    · 环境里没有 cv_bridge，这里用 gazebo::common::Image 直接写 PNG，不依赖 OpenCV。
//
//  边沿触发（进入 near 抓拍一次，离开 far 才重新武装），避免慢速车辆在一圈内被连拍。
//
//  触发判据与回填位姿用的是**画面时刻**的位姿：相机由渲染线程异步出帧，`ImageData()`
//  里那帧总比仿真钟旧一点，判触发和记坐标都按
//      p_frame = p_now − v·Δt
//  回投到"画面里车所在的位置"，保证下游读到的坐标与图里那辆车严格同刻。
//  Δt 由 rendering::Camera::LastRenderWallTime() 现量，实测 0.04~0.05 s（6 m/s 时 0.3 m），
//  所以这是个小修正，不是高速档的解药——高速档空号的真因是车队间距不随速度放大 +
//  北环卡口压在停车线上导致的遮挡/排队，见 tools/speed_sweep.sh 的 follow_gap。
//  曾经按订阅端量到的"帧戳落后仿真钟 1.1 s"（tools/cam_rate.py）配过 <render_lag>1.1</render_lag>，
//  那是链路延迟（出帧攒批 + DDS 排队）不是缓冲新鲜度，照它回投会把 0.7 m/s 档的牌推出画面。
//  另外 world 里 focus 相对抓拍点前移的 1.81 m 与速度无关，那是 near_radius(1.00)
//  加车尾牌挂在底盘原点后面 0.81 m 的固定偏置，保持不动。
//
//  触发后并不保证当场就有帧可用：WorldUpdateBegin 跑在物理/传感器更新的开头，而相机是
//  渲染线程按 update_rate 异步出帧的。实测 12 次触发里有 1 次拿到的是一整幅 128 灰的裸
//  缓冲（唯一色数 1，识别端自然什么都找不到）。所以触发只登记一次待拍请求，等到缓冲区
//  真的出现内容才落盘发布；超过 <frame_wait> 秒仍无帧则放弃并告警。
//
//  SDF 参数：
//    <gate_id>        卡口 ID，写进 PlateCapture.gate_id（必填）
//    <capture_dir>    抓拍图落盘目录（默认 /tmp/v2x_gates）
//    <focus>          抓拍参考点（画面里的车应当出现在这里），world 帧 "x y z"
//    <near_radius>    进入即抓拍的水平半径（默认 6 m）
//    <far_radius>     离开到此半径才重新武装（默认 12 m，必须大于 near）
//    <frame_wait>     触发后最多等渲染线程出帧的仿真秒数（默认 0.25）
//    <rearm_on_fail>  超时放弃后是否立即解除武装（默认 false，保持任务三高速档的旧行为）。
//                    停车场必须开：车在核验位是**停着等**的，一次失败若不解武装，armed 会
//                    永久留在 true —— 而进场车道内离 focus 最远只 1.66 m，够不到 far 2.5，
//                    车就在核验位/等待位之间空转，这道闸再也不会触发过（实测复现过整轮零抓拍）。
//    <render_lag>     拿不到渲染线程墙钟戳时假定的帧龄秒数（默认 0 = 不回投；
//                    实测正常取值 0.04~0.05 s，别拿订阅端的 1.1 s 填这里）
//    <vehicle_prefix> 参与触发的模型名前缀（默认 car_）
//    <chassis_link>   读取位姿所用的 link（默认 chassis）
//    <image_format>   像素格式，需与 <camera><image><format> 一致（默认 R8G8B8）
// ============================================================================
#include <cctype>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Image.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/sensors/sensors.hh>

#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <v2x_msgs/msg/plate_capture.hpp>

namespace v2x
{
class GatePlugin : public gazebo::SensorPlugin
{
public:
  GatePlugin() = default;
  ~GatePlugin() override = default;

  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override
  {
    sensor_ = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(_sensor);
    if (!sensor_) {
      gzerr << "v2x_gate 只能挂在 camera 类型的传感器上" << std::endl;
      return;
    }

    gate_id_ = _sdf->Get<std::string>("gate_id", "gate_unknown").first;
    capture_dir_ = _sdf->Get<std::string>("capture_dir", "/tmp/v2x_gates").first;
    focus_ = _sdf->Get<ignition::math::Vector3d>(
      "focus", ignition::math::Vector3d(0, 0, 0)).first;
    near_ = _sdf->Get<double>("near_radius", 6.0).first;
    far_ = _sdf->Get<double>("far_radius", 12.0).first;
    frame_wait_ = _sdf->Get<double>("frame_wait", 0.25).first;
    render_lag_ = _sdf->Get<double>("render_lag", 0.0).first;
    prefix_ = _sdf->Get<std::string>("vehicle_prefix", "car_").first;
    link_name_ = _sdf->Get<std::string>("chassis_link", "chassis").first;
    // CameraSensor 只给 ImageData()，不带像素格式；格式由 SDF 的 <camera><image><format> 决定，
    // 这里显式声明以免和 world 里的设置脱节。
    fmt_ = _sdf->Get<std::string>("image_format", "R8G8B8").first;
    rearm_on_fail_ = _sdf->Get<bool>("rearm_on_fail", false).first;
    if (far_ <= near_) {
      far_ = near_ * 2.0;
      gzwarn << "far_radius 必须大于 near_radius，已自动调整为 " << far_ << std::endl;
    }

    world_ = gazebo::physics::get_world(_sensor->WorldName());
    std::string node_name = "v2x_gate_" + gate_id_;
    for (auto & c : node_name) {
      if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') c = '_';
    }
    ros_node_ = gazebo_ros::Node::Get(_sdf, node_name);
    cap_pub_ = ros_node_->create_publisher<v2x_msgs::msg::PlateCapture>(
      "/plate/capture", rclcpp::QoS(50).keep_last(20));

    conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GatePlugin::on_update, this));

    RCLCPP_INFO(ros_node_->get_logger(),
      "[v2x_gate] %s focus=(%.1f, %.1f) near=%.1f far=%.1f -> /plate/capture",
      gate_id_.c_str(), focus_.X(), focus_.Y(), near_, far_);
  }

private:
  struct Pending
  {
    std::string name;
    gazebo::physics::ModelPtr model;
    gazebo::physics::LinkPtr link;
    double dist = 0.0;
    gazebo::common::Time deadline{0, 0};
    bool warned = false;
  };

  /// 缓冲区里那一帧到底是多久之前渲染的。
  /// 两个"看起来能用"的接口都不量这个：
  ///   · Sensor::LastMeasurementTime() 在**主线程传感器更新**时就推进，打出来恒在
  ///     0.07~0.11 s（正好一个 10 Hz 更新周期），跟渲染线程无关；
  ///   · tools/cam_rate.py 量到的"图像话题帧戳落后仿真钟 1.1 s"是**订阅端**的数，
  ///     含 DDS 排队，比进程内缓冲的真实年龄大——按它配 <render_lag>1.1</render_lag>
  ///     实测把 0.7 m/s 档的取景整个推歪（车掉出画面底沿，见 gate_j3_north-33.png）。
  /// 真正问得出年龄的是渲染线程自己盖的墙钟戳 rendering::Camera::LastRenderWallTime()。
  /// 墙钟差当仿真钟差用（本栈实时率 ~1.0），只在拿不到时才退回 <render_lag> 常量。
  double frame_lag()
  {
    const gazebo::rendering::CameraPtr cam = sensor_->Camera();
    if (cam) {
      const double dt = gazebo::common::Time::GetWallTime().Double() -
                        cam->LastRenderWallTime().Double();
      if (dt > 0.0 && dt < 5.0) return dt;
    }
    return render_lag_;
  }

  /// 画面时刻的位姿：把当前位姿按速度回退一个实测帧龄（0.04~0.05 s 量级）。
  /// 匀速段这是精确的；转弯段连角速度一起回退，误差不超过一个帧龄内的转角。
  ignition::math::Pose3d frame_pose(gazebo::physics::LinkPtr _l)
  {
    const double dt = frame_lag();
    const auto p = _l->WorldPose();
    const auto v = _l->WorldLinearVel();
    const double w = _l->WorldAngularVel().Z();
    const auto r = p.Rot();
    return ignition::math::Pose3d(
      p.Pos() - v * dt,
      ignition::math::Quaterniond(r.Roll(), r.Pitch(), r.Yaw() - w * dt));
  }

  void on_update()
  {
    if (!sensor_ || !world_ || !ros_node_) return;

    gazebo::physics::ModelPtr hit;
    gazebo::physics::LinkPtr hit_link;
    double hit_d = 1e18;
    double hit_dist = 0.0;

    for (auto & m : world_->Models()) {
      if (m->GetName().compare(0, prefix_.size(), prefix_) != 0) continue;
      gazebo::physics::LinkPtr l = m->GetLink(link_name_);
      if (!l) continue;
      // 判据用画面时刻的位置，不是物理此刻的位置——见文件头 frame_lag() 一段
      const auto p = frame_pose(l).Pos();
      double d = std::hypot(p.X() - focus_.X(), p.Y() - focus_.Y());
      bool & armed = armed_[m->GetName()];
      if (armed && d > far_) {
        armed = false;                      // 离开检测区，重新武装
        if (pending_.name == m->GetName()) pending_ = Pending();
      } else if (!armed && d <= near_ && d < hit_d) {
        armed = true;
        hit = m;
        hit_link = l;
        hit_d = d;
        hit_dist = d;
      }
    }

    if (hit) {
      pending_ = Pending();
      pending_.name = hit->GetName();
      pending_.model = hit;
      pending_.link = hit_link;
      pending_.dist = hit_dist;
      pending_.deadline = world_->SimTime() + gazebo::common::Time(frame_wait_);
    }

    if (!pending_.model) return;

    if (capture(pending_.model, pending_.link, pending_.dist)) {
      pending_ = Pending();
      return;
    }
    if (world_->SimTime() > pending_.deadline) {
      const std::string lost = pending_.name;
      pending_ = Pending();
      if (rearm_on_fail_) {
        // 车还停在近圈里，解除武装后下一拍就会重新登记一次待拍请求（每 frame_wait 一次），
        // 直到渲染线程真的出帧。停着等的核验位必须这样，否则一次超时就把这道闸永久锁死。
        armed_[lost] = false;
        RCLCPP_WARN(ros_node_->get_logger(),
          "[v2x_gate] %s 触发后 %.2f s 相机仍未出帧，放弃对 %s 的抓拍（已重新武装，"
          "车若仍在近圈内会按 %.2f s 一次继续重试）",
          gate_id_.c_str(), frame_wait_, lost.c_str(), frame_wait_);
      } else {
        RCLCPP_WARN(ros_node_->get_logger(),
          "[v2x_gate] %s 触发后 %.2f s 相机仍未出帧，放弃对 %s 的抓拍",
          gate_id_.c_str(), frame_wait_, lost.c_str());
      }
    }
  }

  /// 缓冲区是否已经是一次真正的渲染结果。未渲染的裸缓冲是清一色的 128 灰，
  /// 整幅按 ~13 像素间隔采样、只要出现第二个不同值就认为有内容。
  static bool has_content(const unsigned char * _raw, size_t _w, size_t _h)
  {
    const size_t line = _w * 3;
    unsigned char ref[3];
    bool first = true;
    for (size_t y = 0; y < _h; y += 13) {
      const unsigned char * row = _raw + y * line;
      for (size_t x = 0; x < _w; x += 13) {
        const unsigned char * px = row + x * 3;
        if (first) {
          ref[0] = px[0]; ref[1] = px[1]; ref[2] = px[2];
          first = false;
          continue;
        }
        if (px[0] != ref[0] || px[1] != ref[1] || px[2] != ref[2]) return true;
      }
    }
    return false;
  }

  /// 返回 false 表示这一帧还不可用，调用方应在下一拍重试。
  bool capture(gazebo::physics::ModelPtr _m, gazebo::physics::LinkPtr _l, double _dist)
  {
    const unsigned char * raw = sensor_->ImageData();
    const unsigned w = sensor_->ImageWidth();
    const unsigned h = sensor_->ImageHeight();
    const std::string fmt = fmt_;
    if (!raw || w == 0 || h == 0 || !has_content(raw, w, h)) {
      if (!pending_.warned) {
        RCLCPP_INFO(ros_node_->get_logger(),
          "[v2x_gate] %s 触发时渲染线程还没写入帧，等待出帧", gate_id_.c_str());
        pending_.warned = true;
      }
      return false;
    }

    v2x_msgs::msg::PlateCapture pc;
    const gazebo::common::Time & st = world_->SimTime();
    pc.header.stamp.sec = st.sec;
    pc.header.stamp.nanosec = static_cast<uint32_t>(st.nsec);
    pc.header.frame_id = "world";
    ++seq_;
    pc.capture_id = gate_id_ + "-" + std::to_string(seq_);
    pc.gate_id = gate_id_;
    pc.vehicle_id = _m->GetName();

    // 位姿也取画面时刻的：直接记当前位姿会让下游读到的坐标和图里的车差出 v·Δt
    // （2.8 m/s 时是 3 m），高速档"位置说车在抓拍点、图里车已经开出去了"就是这么来的。
    const auto pose = frame_pose(_l);
    pc.pose.position.x = pose.Pos().X();
    pc.pose.position.y = pose.Pos().Y();
    pc.pose.position.z = pose.Pos().Z();
    pc.pose.orientation.x = pose.Rot().X();
    pc.pose.orientation.y = pose.Rot().Y();
    pc.pose.orientation.z = pose.Rot().Z();
    pc.pose.orientation.w = pose.Rot().W();
    // 水平速度。复位/摆车会把车放到 z=7.0，落回路面（6.70）那 0.3 m 自由落体就有
    // 2.4 m/s 的垂直分量，用 Length() 时卡口"实测速度"会凭空多出 2~5 m/s。
    const ignition::math::Vector3d lv = _l->WorldLinearVel();
    pc.speed = std::hypot(lv.X(), lv.Y());

    pc.image_width = static_cast<uint16_t>(w);
    pc.image_height = static_cast<uint16_t>(h);
    pc.image_encoding = "bgr8";

    // OGRE 的 PF_R8G8B8 在显存里按 B,G,R 的字节序回读，gazebo 原样把这段 buffer 交给我们。
    // 直接按 R8G8B8 存 PNG 会得到整张红蓝互换的图：实测蓝色车身成橘色、蓝牌成砖红色，
    // 车牌定位的 HSV 色相窗口（100~130）一个像素都命中不了。
    // 这里先换回真正的 RGB 顺序再落盘，PNG 用 cv2 读出来就是标准 bgr8，
    // 所以下面 pc.image_encoding = "bgr8" 才是诚实的。
    std::vector<unsigned char> rgb;
    const std::string want = "R8G8B8";
    if (fmt == want)
    {
      rgb.assign(raw, raw + static_cast<size_t>(w) * h * 3);
      for (size_t i = 0; i + 2 < rgb.size(); i += 3)
        std::swap(rgb[i], rgb[i + 2]);
    }
    const unsigned char *pixels = rgb.empty() ? raw : rgb.data();

    gazebo::common::Image img;
    img.SetFromData(pixels, w, h, gazebo::common::Image::ConvertPixelFormat(fmt));
    std::string path = capture_dir_ + "/" + pc.capture_id + ".png";
    if (img.Valid()) {
      img.SavePNG(path);
      pc.image_path = path;
    } else {
      // 格式与 <camera><image><format> 对不上，重试也不会好，照旧发布让下游看得见。
      RCLCPP_ERROR(ros_node_->get_logger(), "[v2x_gate] 帧无法转 PNG（格式 %s）",
                   fmt.c_str());
      pc.image_path = "";
    }

    cap_pub_->publish(pc);
    // lag 打出来是为了核对 LastMeasurementTime 到底给不给真滞后：给出 ~1.1 就是实测值，
    // 恒定 1.10 就是退回了 <render_lag> 常量，两者对高速档的意义不同。
    RCLCPP_INFO(ros_node_->get_logger(),
      "[v2x_gate] 抓拍 %s @ %.1f m/s d=%.1f m lag=%.2f s -> %s",
      pc.capture_id.c_str(), pc.speed, _dist, frame_lag(), path.c_str());
    return true;
  }

  gazebo::sensors::CameraSensorPtr sensor_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr conn_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<v2x_msgs::msg::PlateCapture>::SharedPtr cap_pub_;

  std::string gate_id_, capture_dir_, prefix_, link_name_, fmt_;
  ignition::math::Vector3d focus_;
  double near_ = 6.0, far_ = 12.0, frame_wait_ = 0.25, render_lag_ = 0.0;
  unsigned seq_ = 0;
  Pending pending_;
  bool rearm_on_fail_ = false;
  std::map<std::string, bool> armed_;
};

GZ_REGISTER_SENSOR_PLUGIN(GatePlugin)
}  // namespace v2x
