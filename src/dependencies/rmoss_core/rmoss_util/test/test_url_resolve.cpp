 

#include <string>

#include "gtest/gtest.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rmoss_util/url_resolver.hpp"

TEST(URLResolver, package)
{
  std::string url = "package://rmoss_util/test";
  std::string gt_path = ament_index_cpp::get_package_share_directory("rmoss_util") + "/test";

  EXPECT_STREQ(rmoss_util::URLResolver::get_resolved_path(url).c_str(), gt_path.c_str());
}

TEST(URLResolver, file)
{
  std::string url = "file:///test_dir/test_file";
  std::string gt_path = "/test_dir/test_file";

  EXPECT_STREQ(rmoss_util::URLResolver::get_resolved_path(url).c_str(), gt_path.c_str());
}
