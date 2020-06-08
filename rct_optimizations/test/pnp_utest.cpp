#include <gtest/gtest.h>
#include <rct_optimizations/pnp.h>
#include <rct_optimizations_tests/observation_creator.h>
#include <rct_optimizations_tests/utilities.h>

using namespace rct_optimizations;

TEST(PNP_2D, PerfectInitialConditions)
{
  test::Camera camera = test::makeKinectCamera();
  test::Target target(5, 7, 0.025);

  Eigen::Isometry3d camera_to_target(Eigen::Isometry3d::Identity());
  camera_to_target.translate(Eigen::Vector3d(0.0, 0.0, 1.0));
  camera_to_target.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  PnPProblem problem;
  problem.camera_to_target_guess = camera_to_target;
  problem.intr = camera.intr;
  problem.correspondences = test::getCorrespondences(camera_to_target,
                                                     Eigen::Isometry3d::Identity(),
                                                     camera,
                                                     target,
                                                     true);

  PnPResult result = optimize(problem);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.camera_to_target.isApprox(camera_to_target));
  EXPECT_LT(result.initial_cost_per_obs, 1.0e-15);
  EXPECT_LT(result.final_cost_per_obs, 1.0e-15);
}

TEST(PNP_2D, PerturbedInitialCondition)
{
  test::Camera camera = test::makeKinectCamera();
  test::Target target(5, 7, 0.025);

  Eigen::Isometry3d camera_to_target(Eigen::Isometry3d::Identity());
  camera_to_target.translate(Eigen::Vector3d(0.0, 0.0, 1.0));
  camera_to_target.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  PnPProblem problem;
  problem.camera_to_target_guess = test::perturbPose(camera_to_target, 0.05, 0.05);
  problem.intr = camera.intr;
  problem.correspondences = test::getCorrespondences(camera_to_target,
                                                     Eigen::Isometry3d::Identity(),
                                                     camera,
                                                     target,
                                                     true);

  PnPResult result = optimize(problem);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.camera_to_target.isApprox(camera_to_target, 1.0e-7));
  EXPECT_LT(result.final_cost_per_obs, 1.0e-10);
}

TEST(PNP_2D, BadIntrinsicParameters)
{
  test::Camera camera = test::makeKinectCamera();
  test::Target target(5, 7, 0.025);

  Eigen::Isometry3d camera_to_target(Eigen::Isometry3d::Identity());
  camera_to_target.translate(Eigen::Vector3d(0.0, 0.0, 1.0));
  camera_to_target.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  PnPProblem problem;
  problem.camera_to_target_guess = camera_to_target;
  problem.correspondences = test::getCorrespondences(camera_to_target,
                                                     Eigen::Isometry3d::Identity(),
                                                     camera,
                                                     target,
                                                     true);

  // Tweak the camera intrinsics such that we are optimizing with different values (+/- 1%)
  // than were used to generate the observations
  problem.intr = camera.intr;
  problem.intr.fx() *= 1.01;
  problem.intr.fy() *= 0.99;
  problem.intr.cx() *= 1.01;
  problem.intr.cy() *= 0.99;

  PnPResult result = optimize(problem);

  // The optimization should still converge by moving the camera further away from the target,
  // but the residual error and error in the resulting transform should be much higher
  EXPECT_TRUE(result.converged);
  EXPECT_FALSE(result.camera_to_target.isApprox(camera_to_target, 1.0e-3));
  EXPECT_GT(result.final_cost_per_obs, 1.0e-3);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
