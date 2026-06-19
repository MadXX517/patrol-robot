import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StringPublisher(Node):
    def __init__(self):
        # 初始化节点，名称为"simple_string_publisher"
        super().__init__("simple_string_publisher")
        # 创建发布者：话题名"string_topic"，消息类型String，队列大小10
        self.tts_pub = self.create_publisher(String, "/tts_node/tts_text", 10)

        self.publish_string_msg()

    def publish_string_msg(self):
        msg = String()
        msg.data = f"好的,我会按照你的要求执行动作。"  # 可修改
        self.tts_pub.publish(msg)

def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    # 创建节点实例
    node = StringPublisher()
    # 运行节点（循环发布）
    rclpy.spin(node)
    # 关闭节点
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()