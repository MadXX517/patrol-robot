import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from car_msg.msg import Beep


class Nav2Waypoints(Node):
    def __init__(self):
        super().__init__('nav2_waypoints')

        self.navigator = BasicNavigator()
        self.waypoints = []
        self.active_waypoints = []
        self.is_navigating = False
        self.last_feedback_log_time = self.get_clock().now()

        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10,
        )
        self.command_sub = self.create_subscription(
            String,
            '/nav2_waypoints/command',
            self.command_callback,
            10,
        )

        self.beep_pub = self.create_publisher(Beep, '/beep_states', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.timer = self.create_timer(0.2, self.check_navigation)

        self.get_logger().info(
            'Ready. Use RViz Publish Point to add waypoints, then publish '
            '"start" to /nav2_waypoints/command.'
        )

    def beep_open(self):
        msg = Beep()
        msg.times = 1
        msg.on_time = 0.3
        msg.off_time = 0.1
        self.beep_pub.publish(msg)

    def clicked_point_callback(self, msg):
        if self.is_navigating:
            self.get_logger().warn('Ignoring clicked point while navigation is running.')
            return

        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id or 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        self.waypoints.append(pose)
        self.publish_waypoint_markers()
        self.get_logger().info(
            f'Added waypoint #{len(self.waypoints)}: '
            f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}'
        )

    def command_callback(self, msg):
        command = msg.data.strip().lower()

        if command == 'start':
            self.start_navigation()
        elif command == 'clear':
            self.clear_waypoints()
        elif command == 'undo':
            self.undo_waypoint()
        elif command == 'cancel':
            self.cancel_navigation()
        else:
            self.get_logger().warn(
                f'Unknown command "{msg.data}". Use start, clear, undo, or cancel.'
            )

    def start_navigation(self):
        if self.is_navigating:
            self.get_logger().warn('Navigation is already running.')
            return

        if not self.waypoints:
            self.get_logger().warn('No waypoints available. Add points in RViz first.')
            return

        now = self.get_clock().now().to_msg()
        self.active_waypoints = []
        for waypoint in self.waypoints:
            active = PoseStamped()
            active.header.frame_id = waypoint.header.frame_id or 'map'
            active.header.stamp = now
            active.pose = waypoint.pose
            self.active_waypoints.append(active)

        self.get_logger().info(f'Starting navigation with {len(self.active_waypoints)} waypoints.')
        try:
            self.navigator.followWaypoints(self.active_waypoints)
        except Exception as exc:
            self.active_waypoints = []
            self.get_logger().error(f'Failed to start waypoint navigation: {exc}')
            return

        self.is_navigating = True
        self.last_feedback_log_time = self.get_clock().now()

    def clear_waypoints(self):
        if self.is_navigating:
            self.get_logger().warn('Cancel navigation before clearing waypoints.')
            return

        self.waypoints.clear()
        self.active_waypoints = []
        self.publish_waypoint_markers()
        self.get_logger().info('Cleared all waypoints.')

    def undo_waypoint(self):
        if self.is_navigating:
            self.get_logger().warn('Cancel navigation before removing waypoints.')
            return

        if not self.waypoints:
            self.get_logger().warn('No waypoint to remove.')
            return

        removed = self.waypoints.pop()
        self.publish_waypoint_markers()
        self.get_logger().info(
            f'Removed waypoint: x={removed.pose.position.x:.3f}, '
            f'y={removed.pose.position.y:.3f}'
        )

    def cancel_navigation(self):
        if not self.is_navigating:
            self.get_logger().warn('No active navigation task to cancel.')
            return

        self.navigator.cancelTask()
        self.is_navigating = False
        self.active_waypoints = []
        self.get_logger().info('Canceled active waypoint navigation.')

    def check_navigation(self):
        if not self.is_navigating:
            return

        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            self.is_navigating = False
            self.active_waypoints = []

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Waypoint navigation succeeded.')
                self.beep_open()
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Waypoint navigation was canceled.')
            else:
                self.get_logger().error('Waypoint navigation failed.')
            return

        now = self.get_clock().now()
        if (now - self.last_feedback_log_time).nanoseconds < 5_000_000_000:
            return

        self.last_feedback_log_time = now
        feedback = self.navigator.getFeedback()
        if feedback is not None and hasattr(feedback, 'current_waypoint'):
            self.get_logger().info(f'Current waypoint index: {feedback.current_waypoint}')

    def publish_waypoint_markers(self):
        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        now = self.get_clock().now().to_msg()
        for index, waypoint in enumerate(self.waypoints, start=1):
            point_marker = self.make_point_marker(index, waypoint, now)
            text_marker = self.make_text_marker(index, waypoint, now)
            markers.markers.extend([point_marker, text_marker])

        self.marker_pub.publish(markers)

    def make_point_marker(self, index, waypoint, stamp):
        marker = Marker()
        marker.header.frame_id = waypoint.header.frame_id or 'map'
        marker.header.stamp = stamp
        marker.ns = 'nav2_waypoints_points'
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = waypoint.pose.position.x
        marker.pose.position.y = waypoint.pose.position.y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def make_text_marker(self, index, waypoint, stamp):
        marker = Marker()
        marker.header.frame_id = waypoint.header.frame_id or 'map'
        marker.header.stamp = stamp
        marker.ns = 'nav2_waypoints_labels'
        marker.id = index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = waypoint.pose.position.x
        marker.pose.position.y = waypoint.pose.position.y
        marker.pose.position.z = 0.35
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.28
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = str(index)
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Waypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
