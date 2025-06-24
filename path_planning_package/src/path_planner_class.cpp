#include <path_planning_package/path_planner_class.hpp>
#include <queue>
#include <unordered_map>
#include <tuple>
#include <cmath>

// Helper for hashing a pair of ints (for unordered_map)
struct PairHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};


PathPlanningServer::PathPlanningServer() : Node("path_planning_server")
{
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  this->robot_pose_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PathPlanningServer::robotPoseCallback, this));

  this->service_ = this->create_service<custom_interfaces::srv::PathFromGoal>(
    "path_planning_server",
    std::bind(
      &PathPlanningServer::handle_request,
      this, 
      std::placeholders::_1, 
      std::placeholders::_2, 
      std::placeholders::_3));

  // Create the subscriber with a custom QoS profile
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Match the publisher's durability

  this->map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos_profile,
      std::bind(
        &PathPlanningServer::map_callback, 
        this, 
        std::placeholders::_1));

  // Create a publisher to visualize the path
  this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

}


// Robot pose callback
void PathPlanningServer::robotPoseCallback()
{
    // Get the robot pose from the transform listener
    
    while (!tf_buffer_->canTransform("map", "galax_base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for transform map -> galax_base_link...");
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer_->lookupTransform("map", "galax_base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
    this->robot_pose.x = transform.transform.translation.x;
    this->robot_pose.y = transform.transform.translation.y;

    this->pose_received = true;
}



void PathPlanningServer::handle_request(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<custom_interfaces::srv::PathFromGoal::Request> request,
  const std::shared_ptr<custom_interfaces::srv::PathFromGoal::Response> response)
{
  (void)request_header;

    // Check if "map" has been populated with data
    if (map.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Map data not available. Please wait for the map callback to populate the map.");
        return;
    }
    else if( this->pose_received == false)
    {
      RCLCPP_ERROR(this->get_logger(), "Robot pose not received yet.");
      return;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Map data is available.");
      // Extract the goal from the request
      geometry_msgs::msg::PointStamped goal = request->goal;

    //   // Define the polygon vertices that represent the valid area
    //   std::vector<std::pair<double, double>> vertices = {
    //     {27.97408676147461, 0.036153316497802734},
    //     {27.88904571533203, -1.3325400352478027},
    //     {-19.40673828125, -0.9108824133872986},
    //     {-19.16746711730957, 13.490667343139648},
    //     {-17.890111923217773, 13.597461700439648},
    //     {-17.834320068359375, 1.0934691429138184}
    // };

    //   // if(!this->is_point_in_polygon(goal.point.x, goal.point.y, vertices))
    //   // {
    //   //   RCLCPP_ERROR(this->get_logger(), "Goal is not in the valid area.");
    //   //   return;
    //   // }


      geometry_msgs::msg::PointStamped goal_index = this->world_to_map(
        goal.point.x, 
        goal.point.y, 
        this->resolution, 
        this->x_offset, 
        this->y_offset);

      RCLCPP_INFO(this->get_logger(), "Goal index = %f, %f", goal_index.point.x, goal_index.point.y);
      
      geometry_msgs::msg::PointStamped init_index = this->world_to_map(
        this->robot_pose.x, 
        this->robot_pose.y, 
        this->resolution, 
        this->x_offset, 
        this->y_offset);

      RCLCPP_INFO(this->get_logger(), "Init index = %f, %f", init_index.point.x, init_index.point.y);

      if (!is_in_bounds(goal_index.point.x, goal_index.point.y)) {
        RCLCPP_ERROR(this->get_logger(), "Goal index (%f, %f) is out of map bounds!", goal_index.point.x, goal_index.point.y);
        return;
      }
      if (!is_in_bounds(init_index.point.x, init_index.point.y)) {
        RCLCPP_ERROR(this->get_logger(), "Start index (%f, %f) is out of map bounds!", init_index.point.x, init_index.point.y);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Computing cost_map");

      this->update_cost_map(
        goal_index.point.x,
        goal_index.point.y);

      std::vector<std::vector<int>> index_path = this->find_path_in_costpam(
        init_index.point.x,
        init_index.point.y,
        goal_index.point.x,
        goal_index.point.y);

      // Convert the index path to a world path
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";  // Example, make sure this is correct for your setup
      path.header.stamp = rclcpp::Clock().now();
      for (auto index : index_path)
      {
          geometry_msgs::msg::PoseStamped pose;
          geometry_msgs::msg::PointStamped point = this->map_to_world(
            index[0], 
            index[1], 
            this->resolution, 
            this->x_offset, 
            this->y_offset);
          pose.pose.position = point.point;
          path.poses.push_back(pose);
      }

      // Take only 1 out of each 10 entries in the path
      nav_msgs::msg::Path reduced_path;
      reduced_path.header = path.header;
      for (size_t i = 0; i < path.poses.size(); i += 10)
      {
          reduced_path.poses.push_back(path.poses[i]);
      }
      // Make sure that the goal is in the path (even if repeated)
      reduced_path.poses.push_back(path.poses[path.poses.size()-1]);
      reduced_path.poses.erase(reduced_path.poses.begin());
      
      // Publish the path
      path_publisher_->publish(reduced_path);

      // Fill the response with the generated path
      response->path = reduced_path;
      RCLCPP_INFO(this->get_logger(), "Path found with %ld entries", reduced_path.poses.size());
      
    }
}


void PathPlanningServer::map_callback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Map data received.");
  int width = msg->info.width;
  int height = msg->info.height;

  this->resolution = msg->info.resolution;
  this->x_offset = msg->info.origin.position.x;
  this->y_offset = msg->info.origin.position.y;

  // Resize the class member "map" to the required dimensions
  this->map.resize(width, std::vector<int>(height));

  // Fill the 2D grid with map data from the OccupancyGrid message
  // Fill the 2D grid with map data from the OccupancyGrid message
  for (int i = 0; i < width; ++i)
  {
      for (int j = 0; j < height; ++j)
      {
          // Calculate the 1D index in the map data
          int index = j * width + i;
          this->map[i][j] = msg->data[index];  // Store in the class member "map"
      }
  }

  RCLCPP_INFO(this->get_logger(), "Map data received and stored.");

  // Loop over the map and adjust the state of points within 0.5 meters of a "100" cell
  int radius_in_cells = static_cast<int>(0.5 / resolution);  // Convert 0.5 meters to grid cells

  for (int i = 0; i < width; ++i)
  {
      for (int j = 0; j < height; ++j)
      {
          if (this->map[i][j] == 100) // If the current point is occupied (100)
          {
              // Loop over the surrounding cells within the 0.5-meter radius
              for (int di = -radius_in_cells; di <= radius_in_cells; ++di)
              {
                  for (int dj = -radius_in_cells; dj <= radius_in_cells; ++dj)
                  {
                      int ni = i + di;
                      int nj = j + dj;

                      // Check if the new indices are within bounds of the map
                      if (ni >= 0 && ni < width && nj >= 0 && nj < height)
                      {
                          // If the cell is free (0), change its state to 1
                          if (this->map[ni][nj] == 0)
                          {
                            this->map[ni][nj] = 1;
                          }
                      }
                  }
              }
          }
      }
  }


  // // Print the map data
  // for (int i = 0; i < height; ++i)
  // {
  //   for (int j = 0; j < width; ++j)
  //   {
  //       if (i % 10 == 0 && j % 10 == 0)
  //       { // Print every 10th row and column
  //           RCLCPP_INFO(this->get_logger(), "map[%d][%d] = %d", i, j, this->map[i][j]);
  //       }
  //   }
  // }   

}


geometry_msgs::msg::PointStamped PathPlanningServer::world_to_map(
  const float x_pose, 
  const float y_pose, 
  const float resolution, 
  const float x_offset, 
  const float y_offset)
{
  geometry_msgs::msg::PointStamped point;
  point.point.x = int((x_pose - x_offset) / resolution);
  point.point.y = int((y_pose - y_offset) / resolution);
  point.point.z = 0.0;
  return point;
}


geometry_msgs::msg::PointStamped PathPlanningServer::map_to_world(
  const int x_index, 
  const int y_index, 
  const float resolution, 
  const float x_offset, 
  const float y_offset)
{
  geometry_msgs::msg::PointStamped point;
  point.point.x = x_index * resolution + x_offset;
  point.point.y = y_index * resolution + y_offset;
  point.point.z = 0.0;
  return point;
}


void PathPlanningServer::update_cost_map(
  const int x_index,
  const int y_index)
{
  int width = this->map.size();
  int height = this->map[0].size();

  // Resize the class member "cost_map" to the required dimensions
  this->cost_map.resize(width, std::vector<int>(height));

  // Fill the cost_map with the map data
  for (int i = 0; i < width; ++i)
  {
      for (int j = 0; j < height; ++j)
      {
        this->cost_map[i][j] = this->map[i][j];
      }
  }

  // Wall proximity penalty

  
  // Add goal distance cost as before
  for (int i = 0; i < width; ++i)
  {
      for (int j = 0; j < height; ++j)
      {
          int distance = std::abs(i - x_index) + std::abs(j - y_index);
          this->cost_map[i][j] = this->cost_map[i][j] + distance*2;
      }
  }
}


std::vector<std::vector<int>> PathPlanningServer::find_path_in_costpam(
  const int start_x_index,
  const int start_y_index,
  const int goal_x_index,
  const int goal_y_index)
{
    using Node = std::pair<int, int>;
    using PQElement = std::tuple<int, int, int>; // (priority, x, y)

    int width = this->map.size();
    int height = this->map[0].size();

    auto in_bounds = [&](int x, int y) {
        return x >= 0 && y >= 0 && x < width && y < height;
    };

    auto heuristic = [&](int x, int y) {
        // Manhattan distance
        return std::abs(x - goal_x_index) + std::abs(y - goal_y_index);
    };

    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open_set;
    std::unordered_map<Node, Node, PairHash> came_from;
    std::unordered_map<Node, int, PairHash> cost_so_far;

    Node start = {start_x_index, start_y_index};
    Node goal = {goal_x_index, goal_y_index};

    open_set.emplace(heuristic(start_x_index, start_y_index), start_x_index, start_y_index);
    cost_so_far[start] = 0;

    std::vector<Node> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };

    while (!open_set.empty()) {
        auto [priority, x, y] = open_set.top();
        open_set.pop();
        Node current = {x, y};

        if (current == goal) {
            // Reconstruct path
            std::vector<std::vector<int>> path;
            Node node = goal;
            while (node != start) {
                path.push_back({node.first, node.second});
                node = came_from[node];
            }
            path.push_back({start.first, start.second});
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& dir : directions) {
            int nx = x + dir.first;
            int ny = y + dir.second;
            Node neighbor = {nx, ny};

            if (!in_bounds(nx, ny))
                continue;
            if (this->map[nx][ny] == 100) // skip obstacles
                continue;

            int new_cost = cost_so_far[current] + this->cost_map[nx][ny];
            if (!cost_so_far.count(neighbor) || new_cost < cost_so_far[neighbor]) {
                cost_so_far[neighbor] = new_cost;
                int priority = new_cost + heuristic(nx, ny);
                open_set.emplace(priority, nx, ny);
                came_from[neighbor] = current;
            }
        }
    }

    // If we get here, no path was found
    RCLCPP_WARN(this->get_logger(), "A* failed to find a path.");
    return {};
}


// Add this helper function to your class
bool PathPlanningServer::is_point_in_polygon(
  double x,
  double y, 
  const std::vector<std::pair<double, double>>& vertices) 
{
  bool inside = false;
  int j = vertices.size() - 1;
  
  for (int i = 0; i < int(vertices.size()); i++) {
      if ((vertices[i].second > y) != (vertices[j].second > y) &&
          (x < (vertices[j].first - vertices[i].first) * (y - vertices[i].second) /
                  (vertices[j].second - vertices[i].second) + vertices[i].first)) {
          inside = !inside;
      }
      j = i;
  }
  
  return inside;
}


bool PathPlanningServer::is_in_bounds(
  int x, 
  int y)
{
    int width = this->map.size();
    int height = this->map[0].size();
    return x >= 0 && y >= 0 && x < width && y < height;
}

