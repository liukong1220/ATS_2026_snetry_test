 
#ifndef RMOSS_PROJECTILE_MOTION__GRAVITY_PROJECTILE_SOLVER_HPP_
#define RMOSS_PROJECTILE_MOTION__GRAVITY_PROJECTILE_SOLVER_HPP_

#include <string>
#include <memory>

#include "rmoss_projectile_motion/projectile_solver_interface.hpp"
#include "rmoss_projectile_motion/iterative_projectile_tool.hpp"

namespace rmoss_projectile_motion
{

// 考虑重力的弹道重力修正工具.
class GravityProjectileSolver : public ProjectileSolverInterface
{
public:
  explicit GravityProjectileSolver(double initial_vel);

  void set_initial_vel(double vel) {initial_vel_ = vel;}
  std::shared_ptr<IterativeProjectileTool> get_iterative_tool() {return iterative_tool_;}
  bool solve(double target_x, double target_h, double & angle) override;
  std::string error_message() override;

private:
  std::shared_ptr<IterativeProjectileTool> iterative_tool_;
  double initial_vel_;
};

}  // namespace rmoss_projectile_motion

#endif  // RMOSS_PROJECTILE_MOTION__GRAVITY_PROJECTILE_SOLVER_HPP_
