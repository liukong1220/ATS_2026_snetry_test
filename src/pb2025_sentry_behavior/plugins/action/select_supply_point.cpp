#include "pb2025_sentry_behavior/plugins/action/select_supply_point.hpp"

namespace pb2025_sentry_behavior
{

SelectSupplyPointAction::SelectSupplyPointAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  const auto node = decision::getNodeFromBlackboard(*this);
  node->get_parameter("decision.rmuc.supply_point.x", supply_x_);
  node->get_parameter("decision.rmuc.supply_point.y", supply_y_);
  node->get_parameter("decision.rmuc.supply_point.z", supply_z_);
}

BT::NodeStatus SelectSupplyPointAction::tick()
{
  geometry_msgs::msg::Point point;
  point.x = supply_x_;
  point.y = supply_y_;
  point.z = supply_z_;

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(decision::makePoseStamped(point));

  setOutput("path", path);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SelectSupplyPointAction::providedPorts()
{
  return {BT::OutputPort<nav_msgs::msg::Path>(
    "path", "{decision_path}", "Single-pose path to the RMUC supply point")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::SelectSupplyPointAction>("SelectSupplyPoint");
}
