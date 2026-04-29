#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_ROBOT_MODE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_ROBOT_MODE_HPP_

#include <cstdint>
#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "example_interfaces/msg/u_int8.hpp"

namespace pb2025_sentry_behavior
{

/**
 * @brief 发布哨兵姿态模式的 BT Action。
 *
 * 这个节点不是简单地把 XML 中传入的 mode 直接发出去，
 * 而是会在真正发布前统一处理以下约束：
 *
 * 1. 姿态切换冷却时间
 * 2. 单局比赛内每种姿态的累计使用时长限制
 * 3. 当前请求姿态不可用时的回退策略
 *
 * 最终结果会发布到 `decision/robot_mode`，再由下位机串口节点继续转发。
 */
class PublishRobotModeAction
: public BT::RosTopicPubStatefulActionNode<example_interfaces::msg::UInt8>
{
public:
  PublishRobotModeAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  /**
   * @brief 定义 BT 可配置端口。
   *
   * 主要端口含义：
   * - mode: 当前行为树分支想要申请的姿态
   * - game_status: 用于识别比赛是否进入 RUNNING，从而决定是否累计时长/是否开新局
   * - cooldown_s: 姿态切换最小间隔
   * - max_cumulative_s: 单局内每个姿态允许累计存在的最大时长
   */
  static BT::PortsList providedPorts();

  /// @brief 生成本次 tick 要发布的姿态模式消息，内部会做模式字符串解析和约束裁决。
  bool setMessage(example_interfaces::msg::UInt8 & msg) override;

  /// @brief 节点 halt 时回发 move，避免保留在攻击/防御模式。
  bool setHaltMessage(example_interfaces::msg::UInt8 & msg) override;

private:
  /**
   * @brief 根据比赛状态、冷却时间和累计时长规则，裁决最终允许发送的姿态。
   * @param requested_mode 行为树当前分支请求的目标姿态
   * @return uint8_t 最终允许发送给下位机的姿态枚举值。
   *
   * 注意返回值可能与 requested_mode 不同，常见原因包括：
   * 1. 距离上一次成功切换未超过 cooldown_s
   * 2. requested_mode 已达到单局累计时长上限
   * 3. 当前 active_mode 也已超限，触发回退姿态选择
   */
  uint8_t resolveModeWithConstraints(uint8_t requested_mode);
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_ROBOT_MODE_HPP_
