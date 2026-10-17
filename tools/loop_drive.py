import math, time, csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# 西南净空矩形环路：x[-17,-6] y[-22,-17]
# 已核实无障碍——school 在其北 3m(y>=-14)，speed_cam(0,-23) 在其东 6m，
# intersection_cam(20,-20)/monitor_cam(-20,20) 及各建筑均远离，全程在 slab 路面内
RING = [(-17, -17), (-6, -17), (-6, -22), (-17, -22)]
LAPS = 2
SAFE_Z = 5.0          # z 低于此视为掉落
MAX_Y = 24.5          # 越界保护
WP_RADIUS = 3.0       # 到点判定半径
WP_TIMEOUT = 60.0     # 单个路点最长耗时，超时强制推进，杜绝原地打转卡死
TOTAL_TIMEOUT = 420.0
PATH_CSV = '/tmp/loop_path.csv'


class LoopDriver(Node):
    def __init__(self):
        super().__init__('loop_driver')
        self.pub = self.create_publisher(Twist, '/car_1/cmd_vel', 10)
        self.create_subscription(Odometry, '/car_1/odom', self.cb, 10)
        self.x = self.y = self.yaw = 0.0
        self.z = 6.7
        self.wz = 0.0
        self.vx = 0.0
        self.got = False
        self.stamp = 0.0

    def cb(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.wz = msg.twist.twist.angular.z
        self.vx = msg.twist.twist.linear.x
        self.got = True
        self.stamp = time.time()


def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    rclpy.init()
    node = LoopDriver()
    t_wait = time.time()
    while rclpy.ok() and not node.got and time.time() - t_wait < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.got:
        print('ERROR: no odom received', flush=True)
        node.destroy_node(); rclpy.shutdown(); return

    print('start pose x=%.2f y=%.2f z=%.2f yaw=%.3f' % (node.x, node.y, node.z, node.yaw), flush=True)
    t0 = time.time()
    idx = 0
    wps = RING * LAPS + [RING[0]]
    last_log = 0.0
    last_wp_t = t0
    fell = False
    stuck = 0
    mode = 0          # 0=前进 1=原地转；滞回带避免拐角来回切换抖动
    lap_marks = []
    csvf = open(PATH_CSV, 'w', newline='')
    writer = csv.writer(csvf)
    writer.writerow(['t', 'x', 'y', 'yaw', 'idx', 'mode', 'err', 'wz'])

    while rclpy.ok() and time.time() - t0 < TOTAL_TIMEOUT and idx < len(wps):
        rclpy.spin_once(node, timeout_sec=0.03)

        # 掉落 / 越界保护
        if node.z < SAFE_Z or abs(node.y) > MAX_Y or abs(node.x) > MAX_Y:
            print('ABORT: fell/off-road z=%.2f x=%.2f y=%.2f' % (node.z, node.x, node.y), flush=True)
            fell = True
            break

        wx, wy = wps[idx]
        dx, dy = wx - node.x, wy - node.y
        d = math.hypot(dx, dy)
        now = time.time()
        if d < WP_RADIUS or now - last_wp_t > WP_TIMEOUT:
            if d >= WP_RADIUS:
                stuck += 1
                print('STUCK: wp %d timeout at pos(%.1f,%.1f) d=%.1f -> skip' % (idx, node.x, node.y, d), flush=True)
            idx += 1
            last_wp_t = now
            mode = 0
            if idx < len(wps):
                if idx % len(RING) == 0:
                    lap_marks.append((idx // len(RING), now - t0, node.x, node.y))
                    print('LAP %d done at t=%.1fs pos(%.1f,%.1f)' % (idx // len(RING), now - t0, node.x, node.y), flush=True)
                else:
                    print('wp %d -> (%.0f,%.0f) at pos(%.1f,%.1f) d=%.1f' % (idx, wps[idx][0], wps[idx][1], node.x, node.y, d), flush=True)
            continue

        err = wrap(math.atan2(dy, dx) - node.yaw)
        t = Twist()
        if abs(err) > 1.6:
            # 大角度：先刹停再慢转，避免带速急转甩尾打滑
            mode = 1
            t.linear.x = 0.0
            t.angular.z = 0.0 if abs(node.vx) > 0.25 else clamp(0.7 * err, -0.35, 0.35)
        else:
            mode = 0
            t.linear.x = 0.7 if abs(err) < 0.3 else 0.3
            t.angular.z = clamp(1.1 * err, -0.45, 0.45)
        node.pub.publish(t)

        writer.writerow(['%.2f' % (now - t0), '%.2f' % node.x, '%.2f' % node.y,
                         '%.3f' % node.yaw, idx, mode, '%.3f' % err, '%.3f' % node.wz])

        if now - last_log > 1.0:
            print('t=%.1f idx=%d/%d lap=%d pos=(%.1f,%.1f,%.1f) yaw=%+.2f err=%+.2f v=%+.2f wz=%+.2f mode=%d d=%.1f' %
                  (now - t0, idx, len(wps), idx // len(RING), node.x, node.y, node.z,
                   node.yaw, err, node.vx, node.wz, mode, d), flush=True)
            last_log = now
        time.sleep(0.05)

    csvf.close()
    for _ in range(10):
        node.pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    print('DONE fell=%s stuck=%s idx=%d/%d laps=%d final pos=(%.1f,%.1f,%.1f)' %
          (fell, stuck, idx, len(wps), idx // len(RING), node.x, node.y, node.z), flush=True)
    node.destroy_node()
    rclpy.shutdown()


main()
