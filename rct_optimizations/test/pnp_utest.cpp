#include <gtest/gtest.h>
#include <rct_optimizations/pnp.h>
#include <rct_optimizations_tests/observation_creator.h>
#include <rct_optimizations_tests/utilities.h>

using namespace rct_optimizations;

TEST(PNP_2D, PerfectInitialConditions)
{
  test::Camera camera = test::makeKinectCamera();

  unsigned target_rows = 5;
  unsigned target_cols = 7;
  double spacing = 0.025;
  test::Target target(target_rows, target_cols, spacing);

  Eigen::Isometry3d target_to_camera(Eigen::Isometry3d::Identity());
  double x = static_cast<double>(target_rows - 1) * spacing / 2.0;
  double y = static_cast<double>(target_cols - 1) * spacing / 2.0;
  target_to_camera.translate(Eigen::Vector3d(x, y, 1.0));
  target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  PnPProblem problem;
  problem.camera_to_target_guess = target_to_camera.inverse();
  problem.intr = camera.intr;
  EXPECT_NO_THROW(problem.correspondences =
                    test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), camera, target, true));

  PnPResult result = optimize(problem);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.camera_to_target.isApprox(target_to_camera.inverse()));
  EXPECT_LT(result.initial_cost_per_obs, 1.0e-15);
  EXPECT_LT(result.final_cost_per_obs, 1.0e-15);
}

TEST(PNP_2D, PerturbedInitialCondition)
{
  test::Camera camera = test::makeKinectCamera();

  unsigned target_rows = 5;
  unsigned target_cols = 7;
  double spacing = 0.025;
  test::Target target(target_rows, target_cols, spacing);

  Eigen::Isometry3d target_to_camera(Eigen::Isometry3d::Identity());
  double x = static_cast<double>(target_rows) * spacing / 2.0;
  double y = static_cast<double>(target_cols) * spacing / 2.0;
  target_to_camera.translate(Eigen::Vector3d(x, y, 1.0));
  target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  PnPProblem problem;
  problem.camera_to_target_guess = test::perturbPose(target_to_camera.inverse(), 0.05, 0.05);
  problem.intr = camera.intr;
  problem.correspondences = test::getCorrespondences(target_to_camera,
                                                     Eigen::Isometry3d::Identity(),
                                                     camera,
                                                     target,
                                                     true);

  PnPResult result = optimize(problem);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.camera_to_target.isApprox(target_to_camera.inverse(), 1.0e-8));
  EXPECT_LT(result.final_cost_per_obs, 1.0e-14);
}

TEST(PNP_2D, BadIntrinsicParameters)
{
  test::Camera camera = test::makeKinectCamera();

  unsigned target_rows = 5;
  unsigned target_cols = 7;
  double spacing = 0.025;
  test::Target target(target_rows, target_cols, spacing);

  Eigen::Isometry3d target_to_camera(Eigen::Isometry3d::Identity());
  double x = static_cast<double>(target_rows) * spacing / 2.0;
  double y = static_cast<double>(target_cols) * spacing / 2.0;
  target_to_camera.translate(Eigen::Vector3d(x, y, 1.0));
  target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  PnPProblem problem;
  problem.camera_to_target_guess = target_to_camera.inverse();
  problem.correspondences = test::getCorrespondences(target_to_camera,
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
  EXPECT_FALSE(result.camera_to_target.isApprox(target_to_camera.inverse(), 1.0e-3));
  EXPECT_GT(result.final_cost_per_obs, 1.0e-3);
}

TEST(PNP_3D, PerfectInitialConditions)
{
  //setup params
  PnPProblem3D setup;

  //testing with only one observation of a 3x3 target

  //make target
  test::Target target(5, 5, 0.025);

  Eigen::Isometry3d target_point = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d camera_point = target_point;

  camera_point.translate(Eigen::Vector3d(0.0,0.0,1.0));
  camera_point.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));

  setup.correspondences = getCorrespondences(camera_point,
                                             target_point,
                                             target);

  setup.camera_to_target_guess = camera_point;

  PnPResult res = rct_optimizations::optimize(setup);

  EXPECT_TRUE(res.converged);
  //costs are oddly high for 3d, perfect guess
  std::cout << res.initial_cost_per_obs << " , " << res.final_cost_per_obs << ";\n";
  EXPECT_TRUE(res.initial_cost_per_obs < 1.0e-2);
  EXPECT_TRUE(res.final_cost_per_obs < 1.0e-2);
  EXPECT_TRUE(res.camera_to_target.isApprox(setup.camera_to_target_guess));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
