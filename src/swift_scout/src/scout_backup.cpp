#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <vector>

struct GoalPosition {
    double x;
    double y;
};

class GoalPublisherNode : public rclcpp::Node {
public:
    explicit GoalPublisherNode(const std::string& robot_namespace)
        : Node("goal_publisher_" + robot_namespace),
          current_goal_index_(0),
          robot_namespace_(robot_namespace),
          goal_in_progress_(false) {
        // Initialize action client
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "/" + robot_namespace + "/navigate_to_pose");

        // Wait for the action server to be available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available for robot %s",
                         robot_namespace.c_str());
            rclcpp::shutdown();
            return;
        }

        // Set predefined goals
        goals_list_ = set_goals_for_robot(robot_namespace_);

        // Start sending goals
        send_next_goal();
    }

private:
    std::vector<GoalPosition> set_goals_for_robot(const std::string& robot_namespace) {
        if (robot_namespace == "tb1") {
            return {{-0.5, -1.5}, {-0.5, -2}, {0.5, -2}, {2, -0.4}, {1.5, 0.5},
                    {1.5, 2}, {-0.5, 2}, {-1.5, 0.9}};
        } else if (robot_namespace == "tb2") {
            return {{-0.5, 0.4}, {-0.5, 2}, {1.5, 5}, {1.0, -2}, {-0.5, -2},
                    {-0.5, -0.4}, {-1.5, -0.9}};
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown robot namespace: %s", robot_namespace.c_str());
            return {};
        }
    }

    void send_next_goal() {
        if (current_goal_index_ < goals_list_.size() && !goal_in_progress_) {
            goal_in_progress_ = true;

            auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.pose.position.x = goals_list_[current_goal_index_].x;
            goal_msg.pose.pose.position.y = goals_list_[current_goal_index_].y;
            goal_msg.pose.pose.position.z = 0.0;
            goal_msg.pose.pose.orientation.w = 1.0;

            auto goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            goal_options.result_callback = std::bind(&GoalPublisherNode::result_callback, this, std::placeholders::_1);

            RCLCPP_INFO(this->get_logger(), "[%s] Sending goal %zu: x = %.2f, y = %.2f",
                        robot_namespace_.c_str(), current_goal_index_,
                        goals_list_[current_goal_index_].x,
                        goals_list_[current_goal_index_].y);

            action_client_->async_send_goal(goal_msg, goal_options);
        } else if (current_goal_index_ >= goals_list_.size()) {
            RCLCPP_INFO(this->get_logger(), "[%s] All goals have been achieved!", robot_namespace_.c_str());
        }
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
        goal_in_progress_ = false;

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "[%s] Goal %zu succeeded!", robot_namespace_.c_str(), current_goal_index_);
            current_goal_index_++;
        } else {
            RCLCPP_ERROR(this->get_logger(), "[%s] Goal %zu failed with code %d.", robot_namespace_.c_str(),
                         current_goal_index_, static_cast<int>(result.code));
        }

        // Send the next goal
        send_next_goal();
    }

    std::vector<GoalPosition> goals_list_;
    size_t current_goal_index_;
    std::string robot_namespace_;
    bool goal_in_progress_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: goal_publisher <robot_namespace>");
        return -1;
    }

    auto node = std::make_shared<GoalPublisherNode>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
