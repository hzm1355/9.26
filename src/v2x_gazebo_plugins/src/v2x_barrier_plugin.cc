// ============================================================================
//  v2x_barrier —— 任务五（五）3/4：道闸升降动力学 + 卡滞/抬杆失败故障模拟插件
//
//  挂在道闸模型的 boom 连杆上，运行于 gzserver。订阅目标角，按 P/D + 重力前馈
//  驱动 revolute joint，并把 joint 的**实测**角、角速度、施加力矩按 state_rate 发
//  BarrierState。parking_manager_node 只消费这些实测值，所以"抬杆失败""卡滞"是从
//  指令角与实测角的偏差里长出来的，不是节点自己声明的一个字符串。
//
//  为什么用 ModelPlugin 驱动 joint 而不是 diffdrive/关节位置控制器：
//    · 条目3 要的是"真实物理碰撞与升降动力学"。joint 力矩驱动下，杆压在车身上时
//      ODE 会给出反力，杆要么被顶住要么把车顶开——这才是碰撞；用 SetPosition 硬设
//      角度只是动画，砸到车也不会有一点阻力。
//    · 抬落时间、堵转力矩这些取证量必须是测出来的，才能和真实道闸的 1.5~6 s 对得上。
//
//  重力前馈不写死数值，而是从模型自己读：mass 取 boom 连杆的 Inertial，质心臂取
//  Inertial 位姿在连杆系里的偏移，关节轴用 GlobalAxis(0)（世界系）。这样改 yaml 里
//  的杆长/杆质量/配重段之后不用改插件，重生成 world 就自动跟上。推导一次即可放心：
//  入口杆轴 (0,-1,0) 杆向 +x、出口杆轴 (0,1,0) 杆向 -x，两者在 +角=抬起 下同给
//      τ_gravity = (r_com × m·g) · axis = -m·g·r_c·cos θ
//  所以抬到 90° 时前馈归零（竖直平衡），水平段最费力 —— 与真实拉簧配重道闸一致。
//  r_com 直接用 WorldCoGPose 现量（每步都随实际转动更新），不做任何小角度近似。
//
//  四种故障（fault_topic 收 "gate_id:KIND"）都是**物理机制层面的注入**，不是直接
//  改状态位，所以判据能从实测值里复核：
//    STUCK        机构卡滞：叠加一个极大的角向阻尼 + 干摩擦阻力矩，杆停在当前位置。
//                 施加力矩会顶到 effort 上限而角速度归零 → 堵转判据自然成立。
//    RAISE_FAIL   抬杆失效：抬杆方向的驱动力矩清零，杆起不来 → 抬杆超时判据成立。
//    NO_CLOSE     落杆失效：落杆方向的驱动力矩清零，竖直杆保持不住也不落 → 落杆超时。
//    SENSOR_FAIL  地感失效：loop_occupied/vehicle_id 冻结在注入瞬间的读数不上报变化，
//                 闸机本体照常动作。manager 见 FAULT_SENSOR_FAIL 后降级为计时放行。
//  清除用 "gate_id:NONE" 或 "gate_id:CLEAR"；清除后把当前目标角当作一次新动作重新
//  起表，否则旧的 elapsed 会立刻再次超时。注入的故障与**测出来**的故障分开存：
//  注入是"我要演这一出"，所以按上表改物理量；测出来的是真出了事，只 latch 一个
//  故障位，绝不再叠加注入的阻尼/断驱 —— 否则一次真堵转会让杆永远起不来。
//
//  安全侧：不管是哪个方向测出的故障，一旦成立就把目标角归到"抬起"并保持驱动，
//  杆从车身上退开（真实道闸的 fail-safe 就是这个方向）。落杆砸到车时如果只 latch
//  故障位、目标角仍留在 0，插件会以限幅力矩继续往下压 —— 那才是条目3 要防的事故。
//
//  SDF 参数（gen_parking_world.py 从 parking_zones.yaml 生成，不要手抄）：
//    <gate_id>         闸机 ID（必填；也是 cmd/state 话题名的片段）
//    <joint_name>      revolute joint 名（默认 boom_joint）
//    <boom_link>       杆连杆名（默认 boom）
//    <cmd_topic>       std_msgs/Float64 目标角 rad（默认 /parking/gate/<id>/cmd）
//    <state_topic>     BarrierState（默认 /parking/gate/<id>/state）
//    <fault_topic>     std_msgs/String "gate_id:KIND"（默认 /parking/fault_inject）
//    <state_rate>      状态发布频率 Hz（默认 20）
//    <open_angle>/<closed_angle>/<angle_tolerance>
//    <p_gain>/<d_gain> 位置环增益（N·m/rad、N·m·s/rad）
//    <raise_timeout_s>/<lower_timeout_s>  到位超时
//    <effort_limit>    施加力矩上限；也是堵转判据的基准（取 joint effort 限的较小者）
//    <jam_effort_ratio>顶到 effort_limit 的这个比例 + 角速度归零 → 判堵转
//    <gravity_comp>    true/false，重力前馈开关（关掉它是"拉簧断了"，用来对照）
//    <loop_id>/<loop_center>/<loop_size>  地感线圈矩形（world 系，轴向对齐）
//    <vehicle_length>/<vehicle_width>     车体包围尺寸，占用判定用
//    <vehicle_prefix>/<chassis_link>      参与占用判定的模型前缀与连杆名
// ============================================================================
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <v2x_msgs/msg/barrier_state.hpp>

namespace v2x
{
class BarrierPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model_ = _model;
    world_ = _model->GetWorld();
    gate_id_ = _sdf->Get<std::string>("gate_id", "gate").first;

    const std::string base = "/parking/gate/" + gate_id_;
    joint_name_ = _sdf->Get<std::string>("joint_name", "boom_joint").first;
    boom_name_ = _sdf->Get<std::string>("boom_link", "boom").first;
    cmd_topic_ = _sdf->Get<std::string>("cmd_topic", base + "/cmd").first;
    state_topic_ = _sdf->Get<std::string>("state_topic", base + "/state").first;
    fault_topic_ = _sdf->Get<std::string>("fault_topic", "/parking/fault_inject").first;

    rate_ = _sdf->Get<double>("state_rate", 20.0).first;
    open_ = _sdf->Get<double>("open_angle", M_PI / 2.0).first;
    closed_ = _sdf->Get<double>("closed_angle", 0.0).first;
    tol_ = _sdf->Get<double>("angle_tolerance", 0.08).first;
    kp_ = _sdf->Get<double>("p_gain", 50.0).first;
    kd_ = _sdf->Get<double>("d_gain", 5.0).first;
    raise_timeout_ = _sdf->Get<double>("raise_timeout_s", 5.0).first;
    lower_timeout_ = _sdf->Get<double>("lower_timeout_s", 5.0).first;
    effort_ = _sdf->Get<double>("effort_limit", 300.0).first;
    jam_ratio_ = _sdf->Get<double>("jam_effort_ratio", 0.9).first;
    gravity_comp_ = _sdf->Get<bool>("gravity_comp", true).first;

    loop_id_ = _sdf->Get<std::string>("loop_id", "").first;
    loop_c_ = _sdf->Get<ignition::math::Vector3d>(
      "loop_center", ignition::math::Vector3d(0, 0, 0)).first;
    loop_s_ = _sdf->Get<ignition::math::Vector3d>(
      "loop_size", ignition::math::Vector3d(0, 0, 0)).first;
    car_l_ = _sdf->Get<double>("vehicle_length", 1.6).first;
    car_w_ = _sdf->Get<double>("vehicle_width", 0.8).first;
    prefix_ = _sdf->Get<std::string>("vehicle_prefix", "car_").first;
    chassis_ = _sdf->Get<std::string>("chassis_link", "chassis").first;

    joint_ = model_->GetJoint(joint_name_);
    if (!joint_) {
      gzerr << "v2x_barrier[" << gate_id_ << "] 找不到 joint " << joint_name_
            << "，插件不工作（杆会自由躺着）" << std::endl;
      return;
    }
    boom_ = model_->GetLink(boom_name_);
    if (!boom_) {
      gzerr << "v2x_barrier[" << gate_id_ << "] 找不到 link " << boom_name_ << std::endl;
      return;
    }

    // 力矩上限取"SDF 里 joint 自己的 limit"与参数两者的较小者：world 里写 300 而
    // 限太松会把杆甩出惯性，真实道闸的电机扭矩与减速机就是一起封顶的。
    const double lim = joint_->GetEffortLimit(0);
    if (lim > 0.0) effort_ = std::min(effort_, lim);

    mass_ = boom_->GetInertial()->Mass();
    // 质心偏移（连杆系）。杆的碰撞段与视觉段都沿 x 铺开，所以它就在 x 上；
    // 这里只取模长，方向每步用 WorldCoGPose 现量，不改 world 也不用重新推。
    arm_ = std::fabs(boom_->GetInertial()->Pose().Pos().X());
    target_ = closed_;

    std::string nn = "v2x_barrier_" + gate_id_;
    for (auto & c : nn) {
      if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') c = '_';
    }
    ros_node_ = gazebo_ros::Node::Get(_sdf, nn);
    state_pub_ = ros_node_->create_publisher<v2x_msgs::msg::BarrierState>(
      state_topic_, rclcpp::QoS(50).keep_last(10));
    cmd_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
      cmd_topic_, 10, std::bind(&BarrierPlugin::on_cmd, this, std::placeholders::_1));
    fault_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      fault_topic_, 10, std::bind(&BarrierPlugin::on_fault, this, std::placeholders::_1));

    conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&BarrierPlugin::on_update, this));

    RCLCPP_INFO(ros_node_->get_logger(),
      "[v2x_barrier] %s joint=%s mass=%.2f kg 臂=%.2f m open=%.2f rad "
      "effort=%.0f N·m 重力前馈=%s loop=%s -> %s",
      gate_id_.c_str(), joint_name_.c_str(), mass_, arm_, open_, effort_,
      gravity_comp_ ? "on" : "off", loop_id_.c_str(), state_topic_.c_str());
  }

private:
  // ── 话题回调 ────────────────────────────────────────────────────────────
  void on_cmd(const std_msgs::msg::Float64::SharedPtr _msg)
  {
    double t = std::clamp(_msg->data, std::min(closed_, open_), std::max(closed_, open_));
    std::lock_guard<std::mutex> g(mtx_);
    if (std::fabs(t - target_) < 1e-6) return;         // 重复目标角不重起计时
    target_ = t;
    new_cmd_ = true;                                    // 下一拍用实测角起表
  }

  void on_fault(const std_msgs::msg::String::SharedPtr _msg)
  {
    const std::string & s = _msg->data;
    const size_t colon = s.find(':');
    if (colon == std::string::npos) return;
    if (s.substr(0, colon) != gate_id_) return;         // 别的闸机的事
    const std::string kind = s.substr(colon + 1);
    unsigned f = v2x_msgs::msg::BarrierState::FAULT_NONE;
    if (kind == "STUCK") f = v2x_msgs::msg::BarrierState::FAULT_STUCK;
    else if (kind == "RAISE_FAIL") f = v2x_msgs::msg::BarrierState::FAULT_RAISE_FAIL;
    else if (kind == "NO_CLOSE") f = v2x_msgs::msg::BarrierState::FAULT_NO_CLOSE;
    else if (kind == "SENSOR_FAIL") f = v2x_msgs::msg::BarrierState::FAULT_SENSOR_FAIL;
    else if (kind != "NONE" && kind != "CLEAR") {
      RCLCPP_WARN(ros_node_->get_logger(),
        "[v2x_barrier] %s 收到未知故障类型 %s，忽略", gate_id_.c_str(), kind.c_str());
      return;
    }
    std::lock_guard<std::mutex> g(mtx_);
    fault_ = f;
    if (f == v2x_msgs::msg::BarrierState::FAULT_NONE) {
      // 清除要同时清注入位和测出来自持的故障位，只清一个的话另一半还会继续挂 FAULT。
      det_fault_ = v2x_msgs::msg::BarrierState::FAULT_NONE;
      // 按当前目标角重新起表：旧 elapsed 一定已经超过阈值，不清零的话
      // 下一拍就会又判一次故障，看起来像"故障清不掉"。
      new_cmd_ = true;
      RCLCPP_INFO(ros_node_->get_logger(), "[v2x_barrier] %s 故障已清除，目标角 %.2f rad",
                  gate_id_.c_str(), target_);
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "[v2x_barrier] %s 注入故障 %s",
                  gate_id_.c_str(), kind.c_str());
    }
  }

  // ── 每物理拍 ────────────────────────────────────────────────────────────
  void on_update()
  {
    if (!joint_ || !boom_ || !ros_node_) return;
    std::lock_guard<std::mutex> g(mtx_);

    const gazebo::common::Time now = world_->SimTime();
    const double ang = joint_->Position(0);
    const double vel = joint_->GetVelocity(0);
    const double err = target_ - ang;

    if (new_cmd_) {
      new_cmd_ = false;
      action_start_ = now;
      action_dir_ = (target_ >= ang) ? 1 : -1;
      prog_since_ = now;
      prog_ref_ = ang;
      completed_ = false;
    }

    double drive = kp_ * err - kd_ * vel;
    double ff = 0.0;
    if (gravity_comp_) {
      // τ = (r_com × m·g) · axis，全部现量：r_com 是世界系里质心相对关节原点的偏移，
      // 关节原点按构造等于 boom 连杆原点（joint 没写 <pose>，父链被 fixed 到 world）。
      const ignition::math::Vector3d r =
        boom_->WorldCoGPose().Pos() - boom_->WorldPose().Pos();
      // 重力现量 World::Gravity()：gazebo-11 的 World 没有 GetPhysicsEngine()，
      // 而且每步现读比 Load 时抄一份更稳（世界可以运行期改重力）。
      const ignition::math::Vector3d gv = world_->Gravity();
      const ignition::math::Vector3d F(r.X() * mass_ * gv.X(),
                                       r.Y() * mass_ * gv.Y(),
                                       r.Z() * mass_ * gv.Z());
      const ignition::math::Vector3d tau(r.Y() * F.Z() - r.Z() * F.Y(),
                                        r.Z() * F.X() - r.X() * F.Z(),
                                        r.X() * F.Y() - r.Y() * F.X());
      const ignition::math::Vector3d ax = joint_->GlobalAxis(0);
      ff = -(tau.X() * ax.X() + tau.Y() * ax.Y() + tau.Z() * ax.Z());
    }
    // 抬杆方向的故障：只掐驱动，重力前馈照给 —— 真实"电磁铁不吸合"就是没有驱动力，
    // 但配重还在，杆不会因此砸下去。
    if (fault_ == v2x_msgs::msg::BarrierState::FAULT_RAISE_FAIL && action_dir_ > 0) {
      drive = 0.0;
    } else if (fault_ == v2x_msgs::msg::BarrierState::FAULT_NO_CLOSE && action_dir_ < 0) {
      drive = 0.0;
    }
    double applied = drive + ff;
    if (fault_ == v2x_msgs::msg::BarrierState::FAULT_STUCK) {
      // 卡滞：大阻尼 + 干摩擦。阻尼取 4000 N·m·s/rad，是正常值(0.1)的四万倍量级里
      // 挑的一个不会让 ODE 发散的数；干摩擦 120 N·m 大于正常保持力矩，杆就定住了。
      double resist = -4000.0 * vel;
      if (std::fabs(vel) > 1e-3) resist -= 120.0 * (vel > 0 ? 1.0 : -1.0);
      applied += resist;
    }
    applied = std::clamp(applied, -effort_, effort_);
    joint_->SetForce(0, applied);

    // ── 占用判定（地感）─────────────────────────────────────────────────
    bool occ = false;
    std::string vid;
    if (!loop_id_.empty() && loop_s_.X() > 0.0 && loop_s_.Y() > 0.0) {
      for (auto & m : world_->Models()) {
        if (m->GetName().compare(0, prefix_.size(), prefix_) != 0) continue;
        gazebo::physics::LinkPtr l = m->GetLink(chassis_);
        if (!l) continue;
        const auto p = l->WorldPose();
        // 车是长方体、线圈是轴向矩形：把车包围成它的外接轴向盒，宁可多判 0.4 m
        // 也不漏判 —— 漏判会落杆砸车，多判只是晚一点落杆。
        const double hl = 0.5 * (car_l_ + car_w_);
        const double hw = hl;
        if (std::fabs(p.Pos().X() - loop_c_.X()) <= hl + 0.5 * loop_s_.X() &&
            std::fabs(p.Pos().Y() - loop_c_.Y()) <= hw + 0.5 * loop_s_.Y()) {
          occ = true;
          vid = m->GetName();
          break;
        }
      }
    }
    if (fault_ == v2x_msgs::msg::BarrierState::FAULT_SENSOR_FAIL) {
      occ = occ_frozen_;          // 地感失效：读数冻结在注入瞬间
      vid = vid_frozen_;
    } else {
      occ_frozen_ = occ;
      vid_frozen_ = vid;
    }

    // ── 到位与故障判定：全部从实测偏差里长出来 ─────────────────────────
    const bool at = std::fabs(err) <= tol_;
    if (at && !completed_) {
      completed_ = true;
      if (action_dir_ > 0) ++raise_count_;
      else ++lower_count_;
    }
    const double elapsed = (now - action_start_).Double();
    unsigned det = v2x_msgs::msg::BarrierState::FAULT_NONE;
    // 机构类注入（STUCK/RAISE_FAIL/NO_CLOSE）期间不跑判据：那三种本来就是"让杆动不了"，
    // 判据再跑一遍只会把演出重判成堵转并把目标角抢回抬起，演的是什么就不清楚了。
    // SENSOR_FAIL 是地感的事，杆该照常受判据保护。
    const bool mech_injected = fault_ == v2x_msgs::msg::BarrierState::FAULT_STUCK ||
      fault_ == v2x_msgs::msg::BarrierState::FAULT_RAISE_FAIL ||
      fault_ == v2x_msgs::msg::BarrierState::FAULT_NO_CLOSE;
    if (!at && action_active() && !mech_injected &&
        det_fault_ == v2x_msgs::msg::BarrierState::FAULT_NONE) {
      // 判据 A 堵转：力矩顶满、角速度归零、还差得远
      if (std::fabs(applied) >= jam_ratio_ * effort_ && std::fabs(vel) < 0.02) {
        det = v2x_msgs::msg::BarrierState::FAULT_STUCK;
      } else if ((now - prog_since_).Double() >= 1.0) {
        // 判据 B 无进展：1 s 窗口内实测角位移不足 0.05 rad。
        // 底数是 tools/test_barrier_dyn.py 的实测：正常单程 0.70~0.85 s（≈2 rad/s），
        // 杆压在车身上时只剩蠕变 v=-0.03 rad/s（1 s 走 0.03 rad）—— 相差两个量级，
        // 所以这条能在 1 s 里判出砸车，而不是等 5 s 超时兜（超时是判据 C）。
        if (std::fabs(ang - prog_ref_) < 0.05) {
          det = v2x_msgs::msg::BarrierState::FAULT_STUCK;
        } else {
          prog_since_ = now;
          prog_ref_ = ang;
        }
      }
      // 判据 C 超时
      if (det == v2x_msgs::msg::BarrierState::FAULT_NONE) {
        const double lim = action_dir_ > 0 ? raise_timeout_ : lower_timeout_;
        if (elapsed > lim) {
          det = action_dir_ > 0 ? v2x_msgs::msg::BarrierState::FAULT_RAISE_FAIL
                               : v2x_msgs::msg::BarrierState::FAULT_NO_CLOSE;
        }
      }
    }
    // 注入的故障优先于测出来的：注入就是"我要演这一出"，别让判据把它覆盖成别的。
    if (det != v2x_msgs::msg::BarrierState::FAULT_NONE &&
        det_fault_ == v2x_msgs::msg::BarrierState::FAULT_NONE) {
      det_fault_ = det;           // 自持：下一次发布前不用再算，避免抖回非故障
      const double want = target_;   // 回落前的下发值，日志要点名它
      // 安全侧：把目标角归到"抬起"并按这个方向重新起表，杆从车身上退开。
      // completed_ 置真是因为这次动作没做成，不该在事后补一个 raise/lower 计数。
      target_ = open_;
      action_dir_ = 1;
      action_start_ = now;
      prog_since_ = now;
      prog_ref_ = ang;
      completed_ = true;
      new_cmd_ = false;
      RCLCPP_WARN(ros_node_->get_logger(),
        "[v2x_barrier] %s 测出%s（下发目标 %.2f rad，实测角 %.2f rad 无进展，"
        "τ=%.0f N·m v=%+.3f rad/s）→ 安全侧保持抬起 %.2f rad",
        gate_id_.c_str(), det == v2x_msgs::msg::BarrierState::FAULT_STUCK ? "堵转 STUCK"
          : (det == v2x_msgs::msg::BarrierState::FAULT_RAISE_FAIL ? "抬杆超时" : "落杆超时"),
        want, ang, applied, vel, open_);
    }
    const unsigned fault = (fault_ != v2x_msgs::msg::BarrierState::FAULT_NONE)
      ? fault_ : det_fault_;

    if (rate_ > 0.0 && (now - last_pub_).Double() >= 1.0 / rate_) {
      last_pub_ = now;
      publish(at, ang, err, vel, applied, elapsed, occ, vid, fault);
    }
  }

  bool action_active() const { return action_start_.Double() > 0.0; }

  void publish(bool at, double ang, double err, double vel, double applied,
               double elapsed, bool occ, const std::string & vid, unsigned fault)
  {
    v2x_msgs::msg::BarrierState s;
    const gazebo::common::Time & st = world_->SimTime();
    s.header.stamp.sec = st.sec;
    s.header.stamp.nanosec = static_cast<uint32_t>(st.nsec);
    s.header.frame_id = "world";
    s.gate_id = gate_id_;

    if (fault != v2x_msgs::msg::BarrierState::FAULT_NONE &&
        fault != v2x_msgs::msg::BarrierState::FAULT_SENSOR_FAIL) {
      s.state = v2x_msgs::msg::BarrierState::STATE_FAULT;
    } else if (at) {
      // 手动停在中间角时按"是否已让开车道"归边，界面只用 OPEN/CLOSED 两态。
      s.state = (std::fabs(target_ - open_) <= tol_ || target_ > 0.5 * (open_ + closed_))
        ? v2x_msgs::msg::BarrierState::STATE_OPEN
        : v2x_msgs::msg::BarrierState::STATE_CLOSED;
    } else {
      s.state = (target_ >= ang) ? v2x_msgs::msg::BarrierState::STATE_OPENING
                                 : v2x_msgs::msg::BarrierState::STATE_CLOSING;
    }
    s.source = v2x_msgs::msg::BarrierState::SRC_AUTO;
    s.fault = fault;
    s.target_rad = target_;
    s.angle_rad = ang;
    s.angle_error_rad = err;
    s.at_target = at;
    s.angvel_rps = vel;
    s.effort_nm = applied;
    s.action_elapsed_s = elapsed;
    s.loop_id = loop_id_;
    s.loop_occupied = occ;
    s.vehicle_id = vid;
    s.raise_count = raise_count_;
    s.lower_count = lower_count_;

    // 状态迁移时打一行日志：取证时要能从 gzserver 日志里独立复核"什么时候到位"，
    // 不能只信 JSONL（那是下游写的，链条断在中游就看不见了）。
    if (s.state != last_state_ || s.fault != last_fault_) {
      RCLCPP_INFO(ros_node_->get_logger(),
        "[v2x_barrier] %s state=%u fault=%u tgt=%.2f ang=%.3f err=%.3f "
        "v=%.2f τ=%.0f t=%.2f s loop=%d",
        gate_id_.c_str(), s.state, s.fault, s.target_rad, s.angle_rad,
        s.angle_error_rad, s.angvel_rps, s.effort_nm, s.action_elapsed_s,
        s.loop_occupied ? 1 : 0);
      last_state_ = s.state;
      last_fault_ = s.fault;
    }
    state_pub_->publish(s);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::LinkPtr boom_;
  gazebo::event::ConnectionPtr conn_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<v2x_msgs::msg::BarrierState>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_sub_;

  std::mutex mtx_;
  std::string gate_id_, joint_name_, boom_name_, cmd_topic_, state_topic_, fault_topic_;
  std::string loop_id_, prefix_, chassis_;
  double rate_ = 20.0, open_ = M_PI / 2.0, closed_ = 0.0, tol_ = 0.08;
  double kp_ = 50.0, kd_ = 5.0, raise_timeout_ = 5.0, lower_timeout_ = 5.0;
  double effort_ = 300.0, jam_ratio_ = 0.9, mass_ = 0.0, arm_ = 0.0;
  double car_l_ = 1.6, car_w_ = 0.8;
  bool gravity_comp_ = true;
  ignition::math::Vector3d loop_c_, loop_s_;

  double target_ = 0.0;
  bool new_cmd_ = false;
  int action_dir_ = 1;
  bool completed_ = true;
  gazebo::common::Time action_start_{0, 0}, prog_since_{0, 0}, last_pub_{0, 0};
  double prog_ref_ = 0.0;                       // 判据 B 的 1 s 窗口起点角
  // fault_ 只放注入的故障（要改物理量那一套），det_fault_ 只放测出来的（latch + 安全侧）。
  // 两者混用的话，一次真堵转会套上注入 STUCK 的四万倍阻尼，杆再也起不来。
  unsigned fault_ = v2x_msgs::msg::BarrierState::FAULT_NONE;
  unsigned det_fault_ = v2x_msgs::msg::BarrierState::FAULT_NONE;
  bool occ_frozen_ = false;
  std::string vid_frozen_;
  unsigned raise_count_ = 0, lower_count_ = 0;
  uint8_t last_state_ = 255, last_fault_ = 255;
};

GZ_REGISTER_MODEL_PLUGIN(BarrierPlugin)
}  // namespace v2x
