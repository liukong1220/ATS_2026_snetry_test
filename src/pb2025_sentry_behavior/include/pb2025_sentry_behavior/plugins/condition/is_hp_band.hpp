#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HP_BAND_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HP_BAND_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsHpBandCondition : public BT::SimpleConditionNode
{
public:
  IsHpBandCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tickCondition();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsHpBandCondition");
  decision::TimeThresholds time_thresholds_;
  decision::HpThresholds hp_thresholds_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HP_BAND_HPP_
