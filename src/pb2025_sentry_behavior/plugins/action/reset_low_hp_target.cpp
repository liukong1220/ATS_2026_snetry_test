#include "pb2025_sentry_behavior/plugins/action/reset_low_hp_target.hpp"

namespace pb2025_sentry_behavior
{

BT::NodeStatus ResetLowHpTargetAction::tick()
{
  setOutput("selected_index", -1);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList ResetLowHpTargetAction::providedPorts()
{
  return {
    BT::OutputPort<int>(
      "selected_index", "{decision_low_hp_target_index}", "Reset low HP target index")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::ResetLowHpTargetAction>("ResetLowHpTarget");
}
