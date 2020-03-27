#include <gtest/gtest.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/eigen_conversions.h>

using namespace rct_optimizations;

TEST(CeresMathUtilities, transformPose6d1)
{
  auto e_pose1 = Eigen::Isometry3d::Identity();
  e_pose1 = e_pose1 * Eigen::Translation3d(0.5, 0.75, 1.0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());

  auto e_pose2 = Eigen::Isometry3d::Identity();
  e_pose2 = e_pose2 * Eigen::Translation3d(0.25, 1.0, 0.5) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());

  auto result = e_pose1 * e_pose2;
  auto result_angle_axis = Eigen::AngleAxisd(result.rotation());
  auto result_position = result.translation();

  // Test that ceres utils produce same results
  Pose6d e_pose6d1 = poseEigenToCal(e_pose1);
  Pose6d e_pose6d2 = poseEigenToCal(e_pose2);

  double t_angle_axes[3];
  double t_position[3];

  transformPose6d<double>(e_pose6d1, e_pose6d2, t_angle_axes, t_position);
  Eigen::Vector3d a = result_angle_axis.axis() * result_angle_axis.angle();
  EXPECT_NEAR(t_angle_axes[0], a.x(), 1e-5);
  EXPECT_NEAR(t_angle_axes[1], a.y(), 1e-5);
  EXPECT_NEAR(t_angle_axes[2], a.z(), 1e-5);

  EXPECT_NEAR(t_position[0], result_position.x(), 1e-5);
  EXPECT_NEAR(t_position[1], result_position.y(), 1e-5);
  EXPECT_NEAR(t_position[2], result_position.z(), 1e-5);
}

TEST(CeresMathUtilities, transformPose6d2)
{
  auto e_pose1 = Eigen::Isometry3d::Identity();
  e_pose1 = e_pose1 * Eigen::Translation3d(0.5, 0.75, 1.0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());

  auto e_pose2 = Eigen::Isometry3d::Identity();
  e_pose2 = e_pose2 * Eigen::Translation3d(0.25, 1.0, 0.5) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
  auto e_pose2_angle_axis = Eigen::AngleAxisd(e_pose2.rotation());
  auto e_pose2_position = e_pose2.translation();

  auto result = e_pose1 * e_pose2;
  auto result_angle_axis = Eigen::AngleAxisd(result.rotation());
  auto result_position = result.translation();

  // Test that ceres utils produce same results
  Pose6d e_pose6d1 = poseEigenToCal(e_pose1);

  double e_pose6d2_angle_axes[3];
  double e_pose6d2_position[3];
  e_pose6d2_angle_axes[0] = e_pose2_angle_axis.axis()(0) * e_pose2_angle_axis.angle();
  e_pose6d2_angle_axes[1] = e_pose2_angle_axis.axis()(1) * e_pose2_angle_axis.angle();
  e_pose6d2_angle_axes[2] = e_pose2_angle_axis.axis()(2) * e_pose2_angle_axis.angle();

  e_pose6d2_position[0] = e_pose2_position.x();
  e_pose6d2_position[1] = e_pose2_position.y();
  e_pose6d2_position[2] = e_pose2_position.z();

  double t_angle_axes[3];
  double t_position[3];

  transformPose6d<double>(e_pose6d1, e_pose6d2_angle_axes, e_pose6d2_position, t_angle_axes, t_position);
  Eigen::Vector3d a = result_angle_axis.axis() * result_angle_axis.angle();
  EXPECT_NEAR(t_angle_axes[0], a.x(), 1e-5);
  EXPECT_NEAR(t_angle_axes[1], a.y(), 1e-5);
  EXPECT_NEAR(t_angle_axes[2], a.z(), 1e-5);

  EXPECT_NEAR(t_position[0], result_position.x(), 1e-5);
  EXPECT_NEAR(t_position[1], result_position.y(), 1e-5);
  EXPECT_NEAR(t_position[2], result_position.z(), 1e-5);
}

TEST(CeresMathUtilities, transformPose6d3)
{
  auto e_pose1 = Eigen::Isometry3d::Identity();
  e_pose1 = e_pose1 * Eigen::Translation3d(0.5, 0.75, 1.0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
  auto e_pose1_angle_axis = Eigen::AngleAxisd(e_pose1.rotation());
  auto e_pose1_position = e_pose1.translation();

  auto e_pose2 = Eigen::Isometry3d::Identity();
  e_pose2 = e_pose2 * Eigen::Translation3d(0.25, 1.0, 0.5) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());


  auto result = e_pose1 * e_pose2;
  auto result_angle_axis = Eigen::AngleAxisd(result.rotation());
  auto result_position = result.translation();

  // Test that ceres utils produce same results
  Pose6d e_pose6d2 = poseEigenToCal(e_pose2);

  double e_pose6d1_angle_axes[3];
  double e_pose6d1_position[3];
  e_pose6d1_angle_axes[0] = e_pose1_angle_axis.axis()(0) * e_pose1_angle_axis.angle();
  e_pose6d1_angle_axes[1] = e_pose1_angle_axis.axis()(1) * e_pose1_angle_axis.angle();
  e_pose6d1_angle_axes[2] = e_pose1_angle_axis.axis()(2) * e_pose1_angle_axis.angle();

  e_pose6d1_position[0] = e_pose1_position.x();
  e_pose6d1_position[1] = e_pose1_position.y();
  e_pose6d1_position[2] = e_pose1_position.z();

  double t_angle_axes[3];
  double t_position[3];

  transformPose6d<double>(e_pose6d1_angle_axes, e_pose6d1_position, e_pose6d2,t_angle_axes, t_position);
  Eigen::Vector3d a = result_angle_axis.axis() * result_angle_axis.angle();
  EXPECT_NEAR(t_angle_axes[0], a.x(), 1e-5);
  EXPECT_NEAR(t_angle_axes[1], a.y(), 1e-5);
  EXPECT_NEAR(t_angle_axes[2], a.z(), 1e-5);

  EXPECT_NEAR(t_position[0], result_position.x(), 1e-5);
  EXPECT_NEAR(t_position[1], result_position.y(), 1e-5);
  EXPECT_NEAR(t_position[2], result_position.z(), 1e-5);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
