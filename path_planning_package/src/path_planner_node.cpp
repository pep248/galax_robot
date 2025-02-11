#include <minimal_service/path_planner_class.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto path_planning_server = std::make_shared<PathPlanningServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(path_planning_server);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}