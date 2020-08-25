#include <rct_ros_tools/target_loaders.h>
#include <rct_image_tools/modified_circle_grid_target.h>
#include <rct_image_tools/charuco_grid_target.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

const static std::string TARGET_FILENAME_PARAM = "target_file";
const static std::string TARGET_DEFINITION_PARAM = "target_definition";

template <typename T>
class TargetLoaderTest : public ::testing::Test
{
public:
  using ::testing::Test::Test;

  /** @brief Create a nominal target */
  T create();

  static const std::string target_type_parameter;
};

template <>
rct_image_tools::ModifiedCircleGridTarget TargetLoaderTest<rct_image_tools::ModifiedCircleGridTarget>::create()
{
  return rct_image_tools::ModifiedCircleGridTarget(7, 5, 0.025);
}

template <>
const std::string TargetLoaderTest<rct_image_tools::ModifiedCircleGridTarget>::target_type_parameter = "modified_circle_grid_target";

template <>
rct_image_tools::CharucoGridTarget TargetLoaderTest<rct_image_tools::CharucoGridTarget>::create()
{
  return rct_image_tools::CharucoGridTarget(7, 5, 0.036195, 0.018256);
}

template <>
const std::string TargetLoaderTest<rct_image_tools::CharucoGridTarget>::target_type_parameter = "charuco_grid_target";

using Implementations = ::testing::Types<rct_image_tools::ModifiedCircleGridTarget,
                                         rct_image_tools::CharucoGridTarget>;

TYPED_TEST_CASE(TargetLoaderTest, Implementations);

TYPED_TEST(TargetLoaderTest, test)
{
  ros::NodeHandle nh("~");
  const std::string filename_param = this->target_type_parameter + "/" + TARGET_FILENAME_PARAM;
  const std::string target_definition_param = this->target_type_parameter + "/" + TARGET_DEFINITION_PARAM;
  TypeParam nominal_target = this->create();
  std::string filename;
  ASSERT_TRUE(nh.getParam(filename_param, filename));

  // Method 1 (From ROS parameter, throwing)
  {
    TypeParam target;
    ASSERT_NO_THROW(target = rct_ros_tools::TargetLoader<TypeParam>::load(nh, target_definition_param));
    EXPECT_EQ(target, nominal_target);
  }

  // Method 2 (From ROS parameter, non-throwing
  {
    TypeParam target;
    ASSERT_TRUE(rct_ros_tools::TargetLoader<TypeParam>::load(nh, target_definition_param, target));
    EXPECT_EQ(target, nominal_target);
  }

  // Method 3 (From file, throwing)
  {
    TypeParam target;
    ASSERT_NO_THROW(target = rct_ros_tools::TargetLoader<TypeParam>::load(filename));
    EXPECT_EQ(target, nominal_target);
  }

  // Method 4 (From file, non-throwing)
  {
    TypeParam target;
    ASSERT_TRUE(rct_ros_tools::TargetLoader<TypeParam>::load(filename, target));
    EXPECT_EQ(target, nominal_target);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "target_loader_utest");
  return RUN_ALL_TESTS();
}
