import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
from car_msg.msg import Beep
import time

class Nav2Goals(Node):
    def __init__(self):
        super().__init__('nav2_goals')
        self.nav2_state = 'wait'
        self.goal_state = 1
        self.navigator = BasicNavigator()
        self.beep_pub = self.create_publisher(Beep, '/beep_states', 1)
        self.timer = self.create_timer(1.0, self.main)  # 创建一个每秒触发的定时器

    def beep_open(self):
        msg = Beep()
        msg.times = 1
        msg.on_time = 0.3
        msg.off_time = 0.1
        self.beep_pub.publish(msg)

    def navigate_to_goal(self, goal):
        if goal is not None:
            self.get_logger().info(f'发送目标: {goal.pose.position.x}, {goal.pose.position.y}')
            self.navigator.goToPose(goal)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                # self.get_logger().info(f'剩余距离: {feedback.distance_remaining:.2f} 米')
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('导航成功！')
                self.nav2_state = 'success'
                self.beep_open()
            else:
                self.get_logger().info('导航失败！')
        else:
            self.get_logger().error('目标为空！')

    def main(self):
        if self.nav2_state == 'wait':
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            if self.goal_state == 1:
                self.get_logger().info('>>>>>> go a')
                pose.pose.position.x = 2.0
                pose.pose.position.y = 1.0
                pose.pose.orientation.w = 1.0
                self.navigate_to_goal(pose)
            elif self.goal_state == 2:
                self.get_logger().info('>>>>>> go b')
                pose.pose.position.x = 2.0
                pose.pose.position.y = -2.0
                pose.pose.orientation.w = 0.7071
                pose.pose.orientation.z = -0.7071
                self.navigate_to_goal(pose)
            elif self.goal_state == 3:
                self.get_logger().info('>>>>>> go c')
                pose.pose.position.x = 0.0
                pose.pose.position.y = -2.0
                pose.pose.orientation.w = 0.0
                pose.pose.orientation.z = 1.0
                self.navigate_to_goal(pose)
            elif self.goal_state == 4:
                self.get_logger().info('>>>>>> go origin')
                pose.pose.position.x = 0.0
                pose.pose.position.y = 0.0
                pose.pose.orientation.w = 1.0
                self.navigate_to_goal(pose)
        elif self.nav2_state == 'success':
            self.goal_state += 1
            self.nav2_state = 'wait'

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Goals()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
