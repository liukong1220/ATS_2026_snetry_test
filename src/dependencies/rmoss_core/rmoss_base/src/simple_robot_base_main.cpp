 
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rmoss_base/simple_robot_base_node.hpp"

int main(int argc, char * argv[])
{
  // create ros2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rmoss_base::SimpleRobotBaseNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
