 
#include "rmoss_util/debug.hpp"

namespace rmoss_util
{
bool g_is_debug = false;

bool get_debug() {return g_is_debug;}
void set_debug(bool get_debug) {g_is_debug = get_debug;}

}  // namespace rmoss_util
