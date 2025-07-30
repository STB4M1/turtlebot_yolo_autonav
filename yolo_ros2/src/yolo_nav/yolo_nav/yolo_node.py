import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloNavNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # OpenCV <-> ROS 画像変換用
        self.bridge = CvBridge()

        # YOLOv8 モデルの読み込み（軽量モデル）
        self.model = YOLO('yolov8n.pt')  # 初回起動時に自動DLされます

        self.get_logger().info("YOLOv8ノードが起動しました")

        # Gazebo仮想カメラの画像トピックを購読
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',   # 必要に応じてトピック名調整
            self.image_callback,
            10
        )

        # 可視化用に加工した画像を配信するトピック
        self.image_pub = self.create_publisher(Image, '/yolo/image_annotated', 10)

    def image_callback(self, msg):
        try:
            # ROS → OpenCV画像に変換
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"画像変換失敗: {e}")
            return

        # YOLOで物体検出（Ultralytics v8）
        try:
            results = self.model(cv_img)[0]
            annotated = results.plot()  # 検出結果付き画像
        except Exception as e:
            self.get_logger().error(f"YOLO推論失敗: {e}")
            return

        try:
            # OpenCV画像 → ROSメッセージに変換して配信
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"画像送信失敗: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
