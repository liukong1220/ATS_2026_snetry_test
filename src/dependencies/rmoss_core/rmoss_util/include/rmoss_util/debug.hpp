 
#ifndef RMOSS_UTIL__DEBUG_HPP_
#define RMOSS_UTIL__DEBUG_HPP_

#define RMOSS_DEBUG(text) \
  if (rmoss_util::get_debug()) text

namespace rmoss_util
{
// debug config
bool get_debug();
void set_debug(bool get_debug);

}  // namespace rmoss_util

#endif  // RMOSS_UTIL__DEBUG_HPP_
