#include "ats_sentry_behavior/plugins/condition/is_vision_target_valid.hpp"

#include <cmath>

#include "ats_rm_interfaces/msg/robot_status.hpp"

namespace ats_sentry_behavior
{

IsVisionTargetValidCondition::IsVisionTargetValidCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsVisionTargetValidCondition::tickCondition, this), config),
  node_(decision::getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
  node_->get_parameter("decision.vision.timeout_s", default_timeout_s_);
  node_->get_parameter("decision.vision.override_hold_s", override_hold_s_);
  node_->get_parameter("decision.vision.activation_hold_s", activation_hold_s_);
  node_->get_parameter("decision.vision.switch_target_hold_s", switch_target_hold_s_);
}

void IsVisionTargetValidCondition::resetVisionLatchState()
{
  has_latched_target_ = false;
  hold_until_.reset();
  activation_started_at_.reset();
  pending_switch_started_at_.reset();
  pending_target_id_ = -1;
}

BT::NodeStatus IsVisionTargetValidCondition::keepLatchedTargetIfAllowed(const char * reason)
{
  if (
    override_hold_s_ > 0.0 && has_latched_target_ && hold_until_ &&
    node_->now() <= *hold_until_)
  {
    setOutput("gimbal_yaw", last_gimbal_yaw_);
    setOutput("gimbal_pitch", last_gimbal_pitch_);
    setOutput("target_id", last_target_id_);
    RCLCPP_DEBUG_THROTTLE(
      logger_, *node_->get_clock(), 1000,
      "Keep vision override latched for %.2fs after transient invalid sample: %s",
      override_hold_s_, reason);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsVisionTargetValidCondition::failWithReason(const char * reason)
{
  if (keepLatchedTargetIfAllowed(reason) == BT::NodeStatus::SUCCESS) {
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *node_->get_clock(), 2000, "Vision override rejected: %s", reason);
  resetVisionLatchState();
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsVisionTargetValidCondition::failImmediately(const char * reason)
{
  RCLCPP_INFO_THROTTLE(
    logger_, *node_->get_clock(), 2000, "Vision override rejected: %s", reason);
  resetVisionLatchState();
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsVisionTargetValidCondition::tickCondition()
{
  auto vision_target = getInput<sp_msgs::msg::VisionTargetMsg>("key_port");
  double timeout_s = default_timeout_s_;
  getInput("timeout_s", timeout_s);

  bool require_nav_hold = true;
  getInput("require_nav_hold", require_nav_hold);

  bool valid_for_new_sample = false;
  int incoming_target_id = -1;
  float incoming_gimbal_yaw = 0.0F;
  float incoming_gimbal_pitch = 0.0F;

  if (vision_target) {
    const auto target_type = decision::visionTargetTypeFromMsg(*vision_target);
    if (!decision::isVisionFollowAllowed(target_type)) {
      std::string reason = "vision follow disabled for target_type=";
      reason += decision::visionTargetTypeToString(target_type);
      return failImmediately(reason.c_str());
    }

    const bool gimbal_valid =
      std::isfinite(vision_target->target_yaw) && std::isfinite(vision_target->target_pitch);
    bool stamp_valid = false;
    bool message_fresh = false;
    const rclcpp::Time stamp(vision_target->timestamp);
    if (stamp.nanoseconds() > 0) {
      stamp_valid = true;
      const auto age_s = (node_->now() - stamp).seconds();
      if (timeout_s <= 0.0 || age_s <= timeout_s) {
        message_fresh = true;
      } else {
        RCLCPP_DEBUG(
          logger_, "Vision target expired: age=%.3fs timeout=%.3fs", age_s, timeout_s);
      }
    }

    valid_for_new_sample =
      vision_target->tracking && (!require_nav_hold || vision_target->nav_hold) &&
      gimbal_valid && stamp_valid && message_fresh;

    if (valid_for_new_sample) {
      const auto now = node_->now();
      incoming_target_id = static_cast<int>(vision_target->target_id);
      incoming_gimbal_yaw = vision_target->target_yaw;
      incoming_gimbal_pitch = vision_target->target_pitch;

      if (!has_latched_target_) {
        if (!activation_started_at_) {
          activation_started_at_ = now;
          pending_target_id_ = incoming_target_id;
        } else if (pending_target_id_ != incoming_target_id) {
          // 初次进入视觉跟随前，若目标 id 还在跳，重新计时，避免一出现就来回跟错目标。
          activation_started_at_ = now;
          pending_target_id_ = incoming_target_id;
        }

        const double activation_elapsed_s =
          activation_started_at_ ? (now - *activation_started_at_).seconds() : 0.0;
        if (activation_hold_s_ > 0.0 && activation_elapsed_s < activation_hold_s_) {
          RCLCPP_INFO_THROTTLE(
            logger_, *node_->get_clock(), 2000,
            "Vision override warming up: target_id=%d elapsed=%.2fs hold=%.2fs",
            incoming_target_id, activation_elapsed_s, activation_hold_s_);
          return BT::NodeStatus::FAILURE;
        } else {
          last_gimbal_yaw_ = incoming_gimbal_yaw;
          last_gimbal_pitch_ = incoming_gimbal_pitch;
          last_target_id_ = incoming_target_id;
          has_latched_target_ = true;
          activation_started_at_.reset();
          pending_switch_started_at_.reset();
          pending_target_id_ = incoming_target_id;

          if (override_hold_s_ > 0.0) {
            hold_until_ = now + rclcpp::Duration::from_seconds(override_hold_s_);
          } else {
            hold_until_.reset();
          }

          setOutput("gimbal_yaw", last_gimbal_yaw_);
          setOutput("gimbal_pitch", last_gimbal_pitch_);
          setOutput("target_id", last_target_id_);
          return BT::NodeStatus::SUCCESS;
        }
      } else if (incoming_target_id != last_target_id_) {
        if (!pending_switch_started_at_ || pending_target_id_ != incoming_target_id) {
          pending_switch_started_at_ = now;
          pending_target_id_ = incoming_target_id;
        }

        const double switch_elapsed_s =
          pending_switch_started_at_ ? (now - *pending_switch_started_at_).seconds() : 0.0;
        if (switch_target_hold_s_ > 0.0 && switch_elapsed_s < switch_target_hold_s_) {
          // 这里继续沿用旧目标，专门抑制“遮挡一下就切另一个敌人”的抖动。
          setOutput("gimbal_yaw", last_gimbal_yaw_);
          setOutput("gimbal_pitch", last_gimbal_pitch_);
          setOutput("target_id", last_target_id_);
          if (override_hold_s_ > 0.0) {
            hold_until_ = now + rclcpp::Duration::from_seconds(override_hold_s_);
          }
          RCLCPP_DEBUG_THROTTLE(
            logger_, *node_->get_clock(), 1000,
            "Delay switching vision target: from=%d to=%d elapsed=%.2fs hold=%.2fs",
            last_target_id_, incoming_target_id, switch_elapsed_s, switch_target_hold_s_);
          return BT::NodeStatus::SUCCESS;
        } else {
          last_gimbal_yaw_ = incoming_gimbal_yaw;
          last_gimbal_pitch_ = incoming_gimbal_pitch;
          last_target_id_ = incoming_target_id;
          has_latched_target_ = true;
          activation_started_at_.reset();
          pending_switch_started_at_.reset();
          pending_target_id_ = incoming_target_id;

          if (override_hold_s_ > 0.0) {
            hold_until_ = now + rclcpp::Duration::from_seconds(override_hold_s_);
          } else {
            hold_until_.reset();
          }

          setOutput("gimbal_yaw", last_gimbal_yaw_);
          setOutput("gimbal_pitch", last_gimbal_pitch_);
          setOutput("target_id", last_target_id_);
          return BT::NodeStatus::SUCCESS;
        }
      } else {
        pending_switch_started_at_.reset();
        pending_target_id_ = incoming_target_id;
        last_gimbal_yaw_ = incoming_gimbal_yaw;
        last_gimbal_pitch_ = incoming_gimbal_pitch;
        if (override_hold_s_ > 0.0) {
          hold_until_ = now + rclcpp::Duration::from_seconds(override_hold_s_);
        } else {
          hold_until_.reset();
        }
        setOutput("gimbal_yaw", last_gimbal_yaw_);
        setOutput("gimbal_pitch", last_gimbal_pitch_);
        setOutput("target_id", last_target_id_);
        return BT::NodeStatus::SUCCESS;
      }
    } else {
      activation_started_at_.reset();
      pending_switch_started_at_.reset();
      pending_target_id_ = -1;
      if (!vision_target->tracking) {
        return failWithReason("tracking=false");
      }
      if (require_nav_hold && !vision_target->nav_hold) {
        return failWithReason("nav_hold=false");
      }
      if (!gimbal_valid) {
        return failWithReason("target_yaw or target_pitch is not finite");
      }
      if (!stamp_valid) {
        return failWithReason("timestamp is zero or invalid");
      }
      if (!message_fresh) {
        return failWithReason("timestamp expired relative to behavior clock");
      }
    }
  } else {
    return failWithReason("vision target message is unavailable on blackboard");
  }

  if (override_hold_s_ > 0.0 && has_latched_target_ && hold_until_ && node_->now() <= *hold_until_) {
    setOutput("gimbal_yaw", last_gimbal_yaw_);
    setOutput("gimbal_pitch", last_gimbal_pitch_);
    setOutput("target_id", last_target_id_);
    RCLCPP_DEBUG_THROTTLE(
      logger_, *node_->get_clock(), 1000,
      "Keep vision override latched for %.2fs to smooth short message dropouts",
      override_hold_s_);
    return BT::NodeStatus::SUCCESS;
  }

  return failWithReason("target warmup/switch hold is not satisfied");
}

BT::PortsList IsVisionTargetValidCondition::providedPorts()
{
  return {
    BT::InputPort<sp_msgs::msg::VisionTargetMsg>(
      "key_port", "{@sp_vision_target}", "Vision fusion message on blackboard"),
    BT::InputPort<double>(
      "timeout_s", "{@decision_vision_timeout_s}", "Maximum allowed message age in seconds"),
    BT::InputPort<bool>(
      "require_nav_hold", true, "Require nav_hold=true before considering the target valid"),
    BT::OutputPort<float>("gimbal_yaw", "{vision_gimbal_yaw}", "Vision gimbal yaw command"),
    BT::OutputPort<float>("gimbal_pitch", "{vision_gimbal_pitch}", "Vision gimbal pitch command"),
    BT::OutputPort<int>("target_id", "{vision_target_id}", "Current vision target id")};
}

}  // namespace ats_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ats_sentry_behavior::IsVisionTargetValidCondition>(
    "IsVisionTargetValid");
}
