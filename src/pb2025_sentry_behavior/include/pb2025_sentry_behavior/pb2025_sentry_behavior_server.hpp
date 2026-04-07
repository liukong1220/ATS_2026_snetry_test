 

#ifndef PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_SERVER_HPP_
#define PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_SERVER_HPP_

#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class SentryBehaviorServer : public BT::TreeExecutionServer
{
public:
  explicit SentryBehaviorServer(const rclcpp::NodeOptions & options);
  bool shouldExportTreeModels() const { return export_tree_models_on_shutdown_; }
  const std::string & treeModelsOutputPath() const { return tree_models_output_path_; }

  /**
   * @brief Callback invoked when a goal is received and before the tree is created.
   * If it returns false, the goal will be rejected.
  */
  bool onGoalReceived(const std::string & tree_name, const std::string & payload) override;

  /**
   * @brief Callback invoked after the tree is created.
   * It can be used, for instance, to initialize a logger or the global blackboard.
   *
   * @param tree The tree that was created
  */
  void onTreeCreated(BT::Tree & tree) override;

  /**
   * @brief onLoopAfterTick invoked at each loop, after tree.tickOnce().
   * If it returns a valid NodeStatus, the tree will stop and return that status.
   * Return std::nullopt to continue the execution.
   *
   * @param status The status of the tree after the last tick
  */
  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override;

  /**
   * @brief onTreeExecutionCompleted is a callback invoked after the tree execution is completed,
   * i.e. if it returned SUCCESS/FAILURE or if the action was cancelled by the Action Client.
   *
   * @param status The status of the tree after the last tick
   * @param was_cancelled True if the action was cancelled by the Action Client
   *
   * @return if not std::nullopt, the string will be sent as [return_message] to the Action Client.
  */
  std::optional<std::string> onTreeExecutionCompleted(
    BT::NodeStatus status, bool was_cancelled) override;

private:
  template <typename T>
  void subscribe(
    const std::string & topic, const std::string & bb_key,
    const rclcpp::QoS & qos = rclcpp::QoS(10));
  void declareDecisionParameters();
  void initializeDecisionBlackboard();

  std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
  std::shared_ptr<BT::StdCoutLogger> logger_cout_;
  uint32_t tick_count_;
  bool use_cout_logger_;
  bool export_tree_models_on_shutdown_;
  std::string tree_models_output_path_;
  std::string decision_input_source_;
  std::string decision_sim_mode_;
  std::string decision_sim_mode_topic_;
  std::string decision_gimbal_topic_;
  std::string decision_vision_topic_;
  double decision_vision_timeout_s_ = 0.5;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_SERVER_HPP_
