#include <rct_ros_tools/target_loaders.h>
#include <rct_image_tools/modified_circle_grid_target.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

const static std::string TARGET_PARAM = "target_definition";
const static std::string TARGET_FILENAME_PARAM = "target_file";

template <typename T>
class TargetLoaderTest : public ::testing::Test
{
public:
  using ::testing::Test::Test;

  T create();

  bool equals(const T& lhs, const T& rhs);
};

template <>
rct_image_tools::ModifiedCircleGridTarget TargetLoaderTest<rct_image_tools::ModifiedCircleGridTarget>::create()
{
  return rct_image_tools::ModifiedCircleGridTarget(7, 5, 0.025);
}

using Implementations = ::testing::Types<rct_image_tools::ModifiedCircleGridTarget>;

TYPED_TEST_CASE(TargetLoaderTest, Implementations);

TYPED_TEST(TargetLoaderTest, test)
{
  ros::NodeHandle nh("~");
  TypeParam nominal_target = this->create();
  std::string filename;
  ASSERT_TRUE(nh.getParam(TARGET_FILENAME_PARAM, filename));

  // Method 1 (From ROS parameter, throwing)
  {
    TypeParam target;
    EXPECT_NO_THROW(target = rct_ros_tools::TargetLoader<TypeParam>::load(nh, "target_definition"));
    EXPECT_EQ(target, nominal_target);
  }

  // Method 2 (From ROS parameter, non-throwing
  {
    TypeParam target;
    EXPECT_TRUE(rct_ros_tools::TargetLoader<TypeParam>::load(nh, "target_definition", target));
    EXPECT_EQ(target, nominal_target);
  }

  // Method 3 (From file, throwing)
  {
    TypeParam target;
    EXPECT_NO_THROW(target = rct_ros_tools::TargetLoader<TypeParam>::load(filename));
    EXPECT_EQ(target, nominal_target);
  }

  // Method 4 (From file, non-throwing)
  {
    TypeParam target;
    EXPECT_TRUE(rct_ros_tools::TargetLoader<TypeParam>::load(filename, target));
    EXPECT_EQ(target, nominal_target);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "target_loader_utest");
  return RUN_ALL_TESTS();
}
