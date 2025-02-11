#include <minimal_service/path_planner_class.hpp>


PathPlanningServer::PathPlanningServer()
  : Node("path_planning_server")
{
  service_ = this->create_service<custom_interfaces::srv::PathFromGoal>(
    "path_planning_server",
    std::bind(&PathPlanningServer::handle_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Create the subscriber with a custom QoS profile
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Match the publisher's durability

  subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos_profile,
      std::bind(&PathPlanningServer::map_callback, this, std::placeholders::_1));

  // Create a publisher to visualize the path
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
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
    else
    {
      RCLCPP_INFO(this->get_logger(), "Map data is available.");
      // Extract the goal from the request
      geometry_msgs::msg::PointStamped goal = request->goal;

      geometry_msgs::msg::PointStamped goal_index = this->world_to_map(
        goal.point.x, 
        goal.point.y, 
        this->resolution, 
        this->x_offset, 
        this->y_offset);
      
      geometry_msgs::msg::PointStamped init_index = this->world_to_map(
        0.0, 
        0.0, 
        this->resolution, 
        this->x_offset, 
        this->y_offset);


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

      // Publish the path
      path_publisher_->publish(path);

      // Fill the response with the generated path
      response->path = path;
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
  this->map.resize(height, std::vector<int>(width));

  // Fill the 2D grid with map data from the OccupancyGrid message
  for (int i = 0; i < height; ++i)
  {
      for (int j = 0; j < width; ++j)
      {
          // Calculate the 1D index in the map data
          int index = i * width + j;
          this->map[i][j] = msg->data[index];  // Store in the class member "map"
      }
  }
  RCLCPP_INFO(this->get_logger(), "Map data received and stored.");

  // Loop over the map and adjust the state of points within 0.5 meters of a "100" cell
  int radius_in_cells = static_cast<int>(0.5 / resolution);  // Convert 0.5 meters to grid cells

  for (int i = 0; i < height; ++i)
  {
      for (int j = 0; j < width; ++j)
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
                      if (ni >= 0 && ni < height && nj >= 0 && nj < width)
                      {
                          // If the cell is free (0), change its state to 80
                          if (this->map[ni][nj] == 0)
                          {
                            this->map[ni][nj] = 10;
                          }
                      }
                  }
              }
          }
      }
  }



  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
        if (i % 10 == 0 && j % 10 == 0)
        { // Print every 10th row and column
            RCLCPP_INFO(this->get_logger(), "map[%d][%d] = %d", i, j, this->map[i][j]);
        }
    }
  }   

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
  int height = this->map.size();
  int width = this->map[0].size();
 
  // Resize the class member "cost_map" to the required dimensions
  cost_map.resize(height, std::vector<int>(width));

  // Fill the cost_map with the map data
  for (int i = 0; i < height; ++i)
  {
      for (int j = 0; j < width; ++j)
      {
        this->cost_map[i][j] = this->map[i][j];  // Store in the class member "cost_map"
      }
  }

  // iterate through each cell of the costmap and add the distance between the current cell and the goal cell
  for (int i = 0; i < height; ++i)
  {
      for (int j = 0; j < width; ++j)
      {
          // Calculate the distance between the current cell and the goal cell
          int distance = std::abs(i - x_index) + std::abs(j - y_index);
          this->cost_map[i][j] = this->cost_map[i][j] + distance;  // Store in the class member "cost_map"
      }
  }
}


std::vector<std::vector<int>> PathPlanningServer::find_path_in_costpam(
  const int start_x_index,
  const int start_y_index,
  const int goal_x_index,
  const int goal_y_index)
  {
    std::vector<std::vector<int>> index_path;
    std::vector<std::vector<int>> queue;

    // Add the start cell to the queue
    queue.push_back({start_x_index, start_y_index});


    // While goal not in index_path
    while (std::find(index_path.begin(), index_path.end(), {goal_x_index, goal_y_index}) == index_path.end())
    {
      // Get the first element from the queue
      std::vector<int> current_cell = queue[0];
      queue.erase(queue.begin());

      // Add the current cell to the index_path
      index_path.push_back(current_cell);

      // Get the neighbors of the current cell
      std::vector<std::vector<int>> neighbors = {
          {current_cell[0] - 1, current_cell[1]},
          {current_cell[0] + 1, current_cell[1]},
          {current_cell[0], current_cell[1] - 1},
          {current_cell[0], current_cell[1] + 1}
      };

      // Loop over the neighbors
      for (auto neighbor : neighbors)
      {
          // Check if the neighbor is within the bounds of the map
          if (neighbor[0] >= 0 && neighbor[0] < this->map.size() && neighbor[1] >= 0 && neighbor[1] < this->map[0].size())
          {
              // Check if the neighbor is not an obstacle
              if (this->map[neighbor[0]][neighbor[1]] != 100)
              {
                  // Check if the neighbor is not already in the index_path
                  if (std::find(index_path.begin(), index_path.end(), neighbor) == index_path.end())
                  {
                      // Check if the neighbor is not already in the queue
                      if (std::find(queue.begin(), queue.end(), neighbor) == queue.end())
                      {
                          // Add the neighbor to the queue
                          queue.push_back(neighbor);
                      }
                  }
              }
          }
      }

      // sort the queue putting at the beggining the cell with the lowest cost in the cost_map
      std::sort(
        queue.begin(), queue.end(), [this](std::vector<int> a, std::vector<int> b)
        {
          return this->cost_map[a[0]][a[1]] < this->cost_map[b[0]][b[1]];
        }
      );
    }

    return index_path;

  }


