 
#include "rmoss_util/time_utils.hpp"

namespace rmoss_util
{
std::chrono::steady_clock::time_point get_curr_time() {return std::chrono::steady_clock::now();}

int64_t count_time_duration(
  const std::chrono::steady_clock::time_point & begin,
  const std::chrono::steady_clock::time_point & end, TimeUnit unit)
{
  if (unit == TimeUnit::MICROSECONDS) {
    std::chrono::microseconds duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
    return duration.count();
  } else if (unit == TimeUnit::MILLISECONDS) {
    std::chrono::milliseconds duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
    return duration.count();
  }
  return 0;
}

}  // namespace rmoss_util
