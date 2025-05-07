#ifndef GOAL_RECEIVER_HPP_
#define GOAL_RECEIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "custom_interfaces/srv/path_from_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class GoalReceiver : public rclcpp::Node
{
    public:
        GoalReceiver();

    private:
        // Client
        rclcpp::Client<custom_interfaces::srv::PathFromGoal>::SharedPtr client_;
        
        // Subscribers
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;

        void goal_callback(
            const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

#endif  // MINIMAL_SERVICE_SERVER_CLASS_HPP_