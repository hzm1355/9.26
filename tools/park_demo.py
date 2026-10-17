import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


class ParkDemo(Node):
    def __init__(self):
        super().__init__('park_demo')
        self.pub = self.create_publisher(Twist, '/car_1/cmd_vel', 10)
        self.create_subscription(Odometry, '/car_1/odom', self.cb, 10)
        self.x = self.y = self.yaw = 0.0
        self.got = False

    def cb(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.got = True


def main():
    rclpy.init()
    n = ParkDemo()
    t0 = time.time()
    while rclpy.ok() and not n.got and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)

    A = (0.0, -16.5)   # J2_ns 停车线前
    B = (0.0, -9.0)    # 通过 J2 后、J1_ns 停车线前
    for cycle in range(2):
        for target, brake in ((A, 10.0), (B, 10.0)):
            # 驶向目标
            while rclpy.ok():
                rclpy.spin_once(n, timeout_sec=0.03)
                dx, dy = target[0] - n.x, target[1] - n.y
                d = math.hypot(dx, dy)
                if d < 1.2:
                    break
                err = wrap(math.atan2(dy, dx) - n.yaw)
                t = Twist()
                t.linear.x = 0.6 if abs(err) < 0.5 else 0.25
                t.angular.z = max(-0.4, min(0.4, 1.0 * err))
                n.pub.publish(t)
                time.sleep(0.05)
            # 刹停等待（红灯期间计入排队）
            print('cycle %d brake at (%.1f,%.1f)' % (cycle, n.x, n.y), flush=True)
            tb = time.time()
            while rclpy.ok() and time.time() - tb < brake:
                n.pub.publish(Twist())
                rclpy.spin_once(n, timeout_sec=0.1)
    for _ in range(10):
        n.pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.05)
    print('park demo done', flush=True)
    n.destroy_node()
    rclpy.shutdown()


main()
