# ROS 2 适配层的最小集成冒烟测试（手动跑，不进 CI）
#
# 用法（VM 里，先 colcon build）：
#   source /opt/ros/humble/setup.bash && source install/setup.bash
#   ./install/foray_navigation/lib/foray_navigation/nav_node --ros-args \
#       -p params_file:=$PWD/config/shared.yaml -p robot_file:=$PWD/config/rmul_2027.yaml \
#       -p regions_file:=$PWD/config/rmul_2027.regions.txt &
#   python3 ros/smoke_test.py
#
# 它做的事：发一张 4x3 m 带墙地图 + 固定位姿 + 目标，订阅 cmd_vel，
# 5 秒内收到 |v| > 0.1 m/s 即 PASS（2026-11-10 实测：规划 6 点、2.5 m/s、120 帧）。
#
"""nav_node 的最小集成冒烟测试（在 VM 里跑）。

做四件事：
  1. 发一张 4x3 m、中间一堵墙的地图（transient_local，和节点订阅的 QoS 对齐）；
  2. 以 20 Hz 发一个固定位姿（假装车没动）；
  3. 发一个目标点；
  4. 订阅 cmd_vel，看有没有真的发出速度指令。

判定：5 秒内收到过 |vx| 或 |vy| > 0.1 m/s 的指令 → PASS。
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Smoke(Node):
    def __init__(self):
        super().__init__("nav_node_smoke")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.map_pub = self.create_publisher(OccupancyGrid, "map", qos)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", 1)
        self.max_speed = 0.0
        self.samples = 0
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_timer(0.05, self.tick)
        self.start = time.time()
        self.goal_sent = False
        self.map_sent = False

    def on_cmd(self, msg: Twist):
        speed = math.hypot(msg.linear.x, msg.linear.y)
        self.max_speed = max(self.max_speed, speed)
        self.samples += 1

    def make_map(self) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.05
        grid.info.width = 80
        grid.info.height = 60
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        data = []
        for iy in range(grid.info.height):
            for ix in range(grid.info.width):
                x = 0.025 + 0.05 * ix
                # 一堵竖墙 x∈[1.9,2.1]，y<2.2 —— 和 nav_demo 的内置图一样
                if 1.9 <= x <= 2.1 and (0.025 + 0.05 * iy) < 2.2:
                    data.append(100)
                else:
                    data.append(0)
        grid.data = data
        return grid

    def tick(self):
        if not self.map_sent:
            self.map_pub.publish(self.make_map())
            self.map_sent = True
            self.get_logger().info("map sent")
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = 0.5
        odom.pose.pose.position.y = 0.5
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        if self.map_sent and not self.goal_sent and time.time() - self.start > 0.5:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = 3.0
            goal.pose.position.y = 1.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.goal_sent = True
            self.get_logger().info("goal sent")


def main():
    rclpy.init()
    node = Smoke()
    deadline = time.time() + 6.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("samples=%d max_speed=%.3f" % (node.samples, node.max_speed))
    node.destroy_node()
    rclpy.shutdown()
    ok = node.max_speed > 0.1 and node.samples > 10
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
