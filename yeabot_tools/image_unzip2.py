import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class H264ImageDecompressor(Node):
    def __init__(self):
        super().__init__('h264_image_decomp')

        self.bridge = CvBridge()

        # 订阅 H.264 压缩图像话题（与发布端话题一致）
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/h264',
            self.compressed_image_callback,
            10  # 队列大小，根据需求调整
        )
        self.subscription  # 防止未使用变量警告

        # 发布解压后的原始图像（话题名可自定义）
        self.publisher = self.create_publisher(
            Image,
            '/camera/h264/uncompressed',
            10
        )

        # 解码配置：原程序用 JPEG 占位，这里默认适配；若后续改为真正 H.264 需修改解码逻辑
        self.decoder_type = "h264"  # 可选: "jpeg" (适配原程序) / "h264" (真正 H.264 解码)

        self.get_logger().info(f"H264 Image Decompressor started. Decoder type: {self.decoder_type}")
        self.get_logger().info(f"Subscribed to: /camera/h264")
        self.get_logger().info(f"Publishing uncompressed image to: /camera/h264/uncompressed")

    def compressed_image_callback(self, msg: CompressedImage):
        try:
            # 1. 提取压缩数据
            compressed_data = np.frombuffer(msg.data, dtype=np.uint8)
            if compressed_data.size == 0:
                self.get_logger().warn("Received empty compressed image data")
                return

            # 2. 根据解码类型处理（核心解码逻辑）
            if self.decoder_type == "jpeg":
                # 适配原程序的 JPEG 占位编码（原程序标记为 h264 但实际是 JPEG）
                cv_image = cv2.imdecode(compressed_data, cv2.IMREAD_COLOR)
            elif self.decoder_type == "h264":
                # 真正的 H.264 解码（需安装相应依赖，如 GStreamer）
                cv_image = self.decode_h264(compressed_data)
            else:
                self.get_logger().error(f"Unsupported decoder type: {self.decoder_type}")
                return

            # 3. 检查解码结果
            if cv_image is None:
                self.get_logger().error("Failed to decode compressed image")
                return

            # 4. 转换为 ROS2 Image 消息（保持与原图像一致的 BGR8 编码）
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            
            # 5. 沿用原始消息的头部信息（时间戳、坐标系），保证数据同步
            ros_image.header = msg.header

            # 6. 发布解压后的图像
            self.publisher.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f"Error processing compressed image: {str(e)}")

    def decode_h264(self, h264_data: np.ndarray) -> np.ndarray:
        """
        真正的 H.264 解码实现（需提前安装依赖）
        依赖：sudo apt-get install gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad python3-gst-1.0
        """
        try:
            # 使用 GStreamer 构建 H.264 解码管道
            pipeline = (
                "appsrc is-live=true do-timestamp=true caps=application/x-rtp,media=video,encoding-name=H264 "
                "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR "
                "! appsink emit-signals=true sync=false max-buffers=1 drop=true"
            )

            # 初始化 GStreamer 管道
            import gi
            gi.require_version('Gst', '1.0')
            from gi.repository import Gst, GObject

            Gst.init(None)
            GObject.threads_init()

            gst_pipeline = Gst.parse_launch(pipeline)
            appsrc = gst_pipeline.get_by_name('appsrc')
            appsink = gst_pipeline.get_by_name('appsink')

            # 设置输入数据
            buffer = Gst.Buffer.new_wrapped(h264_data.tobytes())
            appsrc.emit('push-buffer', buffer)
            appsrc.emit('end-of-stream')

            # 启动管道并获取解码后的数据
            gst_pipeline.set_state(Gst.State.PLAYING)
            sample = appsink.emit('pull-sample')
            gst_pipeline.set_state(Gst.State.NULL)

            if not sample:
                self.get_logger().error("GStreamer H.264 decoding failed: no sample received")
                return None

            # 转换为 OpenCV 图像
            buffer = sample.get_buffer()
            caps = sample.get_caps()
            width = caps.get_structure(0).get_value('width')
            height = caps.get_structure(0).get_value('height')
            
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                self.get_logger().error("Failed to map GStreamer buffer")
                return None

            cv_image = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=map_info.data
            ).copy()

            buffer.unmap(map_info)
            return cv_image

        except ImportError:
            self.get_logger().fatal("GStreamer dependencies not found! Install with: sudo apt-get install python3-gst-1.0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad")
            return None
        except Exception as e:
            self.get_logger().error(f"H.264 decoding error: {str(e)}")
            return None

def main(args=None):
    rclpy.init(args=args)
    decompressor_node = H264ImageDecompressor()
    rclpy.spin(decompressor_node)
    decompressor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()