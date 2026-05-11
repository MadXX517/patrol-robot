#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class Nav2GoalSetter : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2GoalSetter()
        : Node("send_goal")
    {
        // 创建一个 Action 客户端
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // 等待服务器启动
        while (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for NavigateToPose action server...");
        }
        RCLCPP_INFO(this->get_logger(), "NavigateToPose action server available.");

        // 设置导航目标点
        set_goal();
    }

private:
    void set_goal()
    {
        auto goal_msg = NavigateToPose::Goal();

        // 设置目标点坐标和方向 (x, y, z, orientation)
        goal_msg.pose.pose.position.x = 1.0;
        goal_msg.pose.pose.position.y = 1.0;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();

        // 发送目标点
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const GoalHandleNavigateToPose::WrappedResult &result) {
                switch (result.code)
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Navigation succeeded.");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Navigation aborted.");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Navigation canceled.");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                    break;
                }
            };

        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2GoalSetter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
