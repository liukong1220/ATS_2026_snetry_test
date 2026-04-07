 
#include "rmoss_projectile_motion/gimbal_transform_tool.hpp"

namespace rmoss_projectile_motion
{

bool GimbalTransformTool::solve(double x, double y, double z, double & pitch, double & yaw)
{
  if (!solver_) {
    // if model is nullptr, use line model.
    pitch = -atan2(z, x);
  } else {
    double angle;
    if (solver_->solve(x, z, angle)) {
      pitch = -angle;
    } else {
      error_message_ = solver_->error_message();
      return false;
    }
  }
  yaw = atan2(y, x);
  return true;
}

}  // namespace rmoss_projectile_motion
