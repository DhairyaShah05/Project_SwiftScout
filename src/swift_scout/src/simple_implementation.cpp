#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class SimpleGoalTester : public rclcpp::Node {
public:
    SimpleGoalTester() : Node("simple_goal_tester") {
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/tb1/navigate_to_pose");

        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = 1.0;
        goal_msg.pose.pose.position.y = 1.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending a test goal.");
        action_client_->async_send_goal(goal_msg);
    }

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleGoalTester>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
