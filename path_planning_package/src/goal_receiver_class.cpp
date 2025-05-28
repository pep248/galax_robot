#include <path_planning_package/goal_receiver_class.hpp>


GoalReceiver::GoalReceiver() : Node("goal_receiver")
{
  client_ = this->create_client<custom_interfaces::srv::PathFromGoal>(
    "path_planning_server");

  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
  }

  // Create a subscriber to the /goal_pose topic
  goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(
        &GoalReceiver::goalCallback, 
        this, 
        std::placeholders::_1));
}


void GoalReceiver::goalCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received goal pose: x = %f, y = %f", msg->pose.position.x, msg->pose.position.y);
  
  // Create a request for the service
  auto request = std::make_shared<custom_interfaces::srv::PathFromGoal::Request>();
  request->goal.point.x = msg->pose.position.x;
  request->goal.point.y = msg->pose.position.y;
  request->goal.point.z = msg->pose.position.z;

  // Send the service request asynchronously
  auto future = client_->async_send_request(request, 
    [this](rclcpp::Client<custom_interfaces::srv::PathFromGoal>::SharedFuture response) {
      try {
        RCLCPP_INFO(this->get_logger(), "Path received with %ld poses", response.get()->path.poses.size());
        for (const auto &pose : response.get()->path.poses) {
          RCLCPP_INFO(this->get_logger(), "Pose: x = %f, y = %f", pose.pose.position.x, pose.pose.position.y);
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      }
    });
}