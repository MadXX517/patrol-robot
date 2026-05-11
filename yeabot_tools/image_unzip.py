import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DecompressImage(Node):
    def __init__(self):
        super().__init__('decompress_image')

        self.bridge = CvBridge()

        # 订阅压缩图像
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.callback,
            10
        )

        # 发布解压后的原始图像，RViz 可以显示
        self.pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

    def callback(self, msg):
        # 解压 JPEG
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 转 Image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # 发布给 RViz2
        self.pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DecompressImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
