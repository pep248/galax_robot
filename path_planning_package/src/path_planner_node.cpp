#include <path_planning_package/path_planner_class.hpp>
#include <path_planning_package/goal_receiver_class.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto path_planning_server = std::make_shared<PathPlanningServer>();
    auto goal_receiver = std::make_shared<GoalReceiver>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(path_planning_server);
    executor.add_node(goal_receiver);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}