#ifndef MINIMAL_ACTION_SERVER_HPP_
#define MINIMAL_ACTION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "custom_interfaces/srv/path_from_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"



class PathPlanningServer : public rclcpp::Node
{
    public:
        PathPlanningServer();

    private:
        rclcpp::Service<custom_interfaces::srv::PathFromGoal>::SharedPtr service_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

        void handle_request(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<custom_interfaces::srv::PathFromGoal::Request> request,
            const std::shared_ptr<custom_interfaces::srv::PathFromGoal::Response> response);

        void map_callback(
            const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        geometry_msgs::msg::PointStamped map_to_world(
            const int x_index, 
            const int y_index, 
            const float resolution, 
            const float x_offset, 
            const float y_offset);

        geometry_msgs::msg::PointStamped world_to_map(
            const float x_pose, 
            const float y_pose, 
            const float resolution, 
            const float x_offset, 
            const float y_offset);

        void update_cost_map(
            const int x_index,
            const int y_index);

        std::vector<std::vector<int>> find_path_in_costpam(
            const int start_x_index,
            const int start_y_index,
            const int goal_x_index,
            const int goal_y_index);

        // Class member to store the map data
        std::vector<std::vector<int>> map;
        std::vector<std::vector<int>> cost_map;
        float resolution;
        float x_offset;
        float y_offset;
        

        bool is_point_in_polygon(
            double x, 
            double y, 
            const std::vector<std::pair<double, double>>& vertices);


};

#endif  // MINIMAL_SERVICE_SERVER_CLASS_HPP_