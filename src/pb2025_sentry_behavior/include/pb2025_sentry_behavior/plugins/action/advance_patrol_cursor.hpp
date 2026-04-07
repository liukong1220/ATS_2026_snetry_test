#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__ADVANCE_PATROL_CURSOR_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__ADVANCE_PATROL_CURSOR_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace pb2025_sentry_behavior
{

class AdvancePatrolCursorAction : public BT::SyncActionNode
{
public:
  AdvancePatrolCursorAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__ADVANCE_PATROL_CURSOR_HPP_
