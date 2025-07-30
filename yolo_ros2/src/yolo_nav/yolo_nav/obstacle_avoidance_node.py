import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # cmd_vel発行用パブリッシャー
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # LiDARデータ購読
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info("障害物回避ノードが起動しました")

    def scan_callback(self, msg):
        # 前方（±15度）の範囲の距離データ
        front_ranges = msg.ranges[0:15] + msg.ranges[-15:]
        min_dist = min(front_ranges)

        twist = Twist()

        if min_dist < 0.5:  # 0.5m以内に障害物
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # 右に回避（左回転でもOK）
            self.get_logger().info("障害物検知：回避行動")
        else:
            twist.linear.x = 0.2  # 前進
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
