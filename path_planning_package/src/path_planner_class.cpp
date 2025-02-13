#include <minimal_service/path_planner_class.hpp>


PathPlanningServer::PathPlanningServer() : Node("path_planning_server")
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

      // Define the polygon vertices that represent the valid area
      std::vector<std::pair<double, double>> vertices = {
        {27.97408676147461, 0.036153316497802734},
        {27.88904571533203, -1.3325400352478027},
        {-19.40673828125, -0.9108824133872986},
        {-19.16746711730957, 13.490667343139648},
        {-17.890111923217773, 13.597461700439648},
        {-17.834320068359375, 1.0934691429138184}
    };

      if(!this->is_point_in_polygon(goal.point.x, goal.point.y, vertices))
      {
        RCLCPP_ERROR(this->get_logger(), "Goal is not in the valid area.");
        return;
      }


      geometry_msgs::msg::PointStamped goal_index = this->world_to_map(
        goal.point.x, 
        goal.point.y, 
        this->resolution, 
        this->x_offset, 
        this->y_offset);

      RCLCPP_INFO(this->get_logger(), "Goal index = %f, %f", goal_index.point.x, goal_index.point.y);
      
      geometry_msgs::msg::PointStamped init_index = this->world_to_map(
        0.0, 
        0.0, 
        this->resolution, 
        this->x_offset, 
        this->y_offset);

      RCLCPP_INFO(this->get_logger(), "Init index = %f, %f", init_index.point.x, init_index.point.y);

      RCLCPP_INFO(this->get_logger(), "Computing cost_map");

      this->update_cost_map(
        goal_index.point.x,
        goal_index.point.y);

      // int width = this->map.size();
      // int height = this->map[0].size();

      // Print the map data
      // for (int i = 0; i < height; ++i)
      // {
      //   for (int j = 0; j < width; ++j)
      //   {
      //       if (i % 10 == 0 && j % 10 == 0)
      //       { // Print every 10th row and column
      //           RCLCPP_INFO(this->get_logger(), "cost_map[%d][%d] = %d", i, j, this->cost_map[i][j]);
      //       }
      //   }
      // }   

      // int height = this->map.size();
      // int width = this->map[0].size();
      // for (int i = 0; i < height; ++i)
      // {
      //   for (int j = 0; j < width; ++j)
      //   {
      //       if (i % 10 == 0 && j % 10 == 0)
      //       { // Print every 10th row and column
      //           RCLCPP_INFO(this->get_logger(), "cost_map[%d][%d] = %d", i, j, this->cost_map[i][j]);
      //       }
      //   }
      // }   

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

      // Take only 1 out of each 50 entries in the path
      nav_msgs::msg::Path reduced_path;
      reduced_path.header = path.header;
      for (size_t i = 0; i < path.poses.size(); i += 50)
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
  int width = this->map.size(); //TEST
  int height = this->map[0].size(); //TEST
 
  // Resize the class member "cost_map" to the required dimensions
  this->cost_map.resize(width, std::vector<int>(height));

  // Fill the cost_map with the map data
  for (int i = 0; i < width; ++i)
  {
      for (int j = 0; j < height; ++j)
      {
        this->cost_map[i][j] = this->map[i][j];  // Store in the class member "cost_map"
      }
  }

  // iterate through each cell of the costmap and add the distance between the current cell and the goal cell
  for (int i = 0; i < width; ++i)
  {
      for (int j = 0; j < height; ++j)
      {
          // Calculate the distance between the current cell and the goal cell
          int distance = std::abs(i - x_index) + std::abs(j - y_index);
          this->cost_map[i][j] = this->cost_map[i][j] + distance*2;  // Store in the class member "cost_map"
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

    // Add the start cell to the queue
    std::vector<int> start_cell = {start_x_index, start_y_index};

    index_path.push_back(start_cell);

    // Create goal cell vector for comparison
    std::vector<int> goal_cell = {goal_x_index, goal_y_index};

    RCLCPP_INFO(this->get_logger(), "start_cell = %d, %d", start_cell[0], start_cell[1]);
    RCLCPP_INFO(this->get_logger(), "goal_cell = %d, %d", goal_cell[1], goal_cell[1]);

    int width = this->map.size();
    int height = this->map[0].size();

    // While goal not in index_path
    while (std::find(index_path.begin(), index_path.end(), goal_cell) == index_path.end())
    {
        // Get the first element from the queue
        std::vector<int> current_cell = index_path.back();

        
        // Get the neighbors of the current cell
        std::vector<std::vector<int>> neighbors = {
            {current_cell[0] - 1, current_cell[1]},
            {current_cell[0] + 1, current_cell[1]},
            {current_cell[0], current_cell[1] - 1},
            {current_cell[0], current_cell[1] + 1}
        };

        std::vector<int> min_neighbor = {};
        int min_cost = std::numeric_limits<int>::max();
        // RCLCPP_INFO(this->get_logger(), "width: %d, height: %d", width, height);
        for (const auto& neighbor : neighbors)
        {
            // Check if the neighbor is already in the index_path
            if (std::find(index_path.begin(), index_path.end(), neighbor) != index_path.end())
            {
                continue;
            }
            // RCLCPP_INFO(this->get_logger(), "cost: %d, neighbor: %d, %d", this->cost_map[neighbor[0]][neighbor[1]], neighbor[0], neighbor[1]);
            // Find cheapest neighbor in the cost_map
            if (neighbor[0] >= 0 && neighbor[1] >= 0 && 
                neighbor[0] < width && neighbor[1] < height &&
                this->cost_map[neighbor[0]][neighbor[1]] < min_cost)
            {
                // RCLCPP_INFO(this->get_logger(), "min_neighbor %d, %d", neighbor[0], neighbor[1]);
                min_cost = this->cost_map[neighbor[0]][neighbor[1]];
                min_neighbor = neighbor;
            }
        }

        // Add the neighbor to the index_path
        index_path.push_back(min_neighbor);
        // RCLCPP_INFO(this->get_logger(), "Neighbor added= %d, %d", min_neighbor[0], min_neighbor[1]);

        // // Loop over the neighbors
        // for (auto neighbor : neighbors)
        // {
        //   int row = neighbor[0];
        //   int col = neighbor[1];
        //   // RCLCPP_INFO(this->get_logger(), "Evaluated Neighbor = %d, %d", neighbor[0], neighbor[1]);
        //   // Check if the neighbor is within the bounds of the map
        //   // RCLCPP_INFO(this->get_logger(), "width = %d, height = %d", width, height);
        //   if (row >= 0 && col >= 0 && row < width && col < height)
        //   {
        //     // RCLCPP_INFO(this->get_logger(), "Neighbor is in the map");
        //     // Check if the neighbor is not an obstacle
        //     // RCLCPP_INFO(this->get_logger(), "map = %d", this->map[col][row]);
        //     if (this->map[col][row] != 100)
        //     {
        //       // RCLCPP_INFO(this->get_logger(), "Neighbor is not an obstacle");
        //       // Check if the neighbor is not already in the index_path
        //       if (std::find(index_path.begin(), index_path.end(), neighbor) == index_path.end())
        //       {
        //         // RCLCPP_INFO(this->get_logger(), "Neighbor not already in path");
        //         // Check if the neighbor is not already in the queue
        //         if (std::find(queue.begin(), queue.end(), neighbor) == queue.end())
        //         {
        //           // RCLCPP_INFO(this->get_logger(), "Neighbor not already in queue");
        //           // Add the neighbor to the queue
        //           queue.push_back(neighbor);
        //           RCLCPP_INFO(this->get_logger(), "Neighbor added= %d, %d", neighbor[0], neighbor[1]);
        //         }
        //       }
        //     }
        //   }
        // }

        // // sort the queue putting at the beggining the cell with the lowest cost in the cost_map
        // std::sort(queue.begin(), queue.end(), 
        //     [this](const std::vector<int>& a, const std::vector<int>& b) {
        //         return this->cost_map[a[1]][a[0]] < this->cost_map[b[1]][b[0]];
        //     }
        // );
        // Print the queue cost_map cost
        // for (auto cell : queue)
        // {
        //   RCLCPP_INFO(this->get_logger(), "Queue cost_map cost = %d", this->cost_map[cell[1]][cell[0]]);
        // }

    }

    return index_path;
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



