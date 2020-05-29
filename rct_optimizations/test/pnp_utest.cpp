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
//TODO: add a 2d vaildation (requires intrinsics for projection)

TEST(PnPTest, 3DValidation)
{
  //correspondences: do easy offset between target & camera (ie make set translate 0.5 +x)
  //setup params
  PnPProblem3D setup;

  //testing with only one observation
  Correspondence3D3D::Set corrs;
  corrs.reserve(9);

  Eigen::Vector3d origin(1.0,1.0,1.0);
  //what is the origin?
  for(std::size_t i=0; i < 3; ++i)
  {
    for (std::size_t j =0; j < 3; j++)
    {
      Correspondence3D3D c;
      Eigen::Vector3d world_point =Eigen::Vector3d(double<i>, double<j>, double<(i*j)%3>);
      Eigen::Vector3d camera_point = world_point - Eigen::Vector3d(1.0,1.0,1.0);
      c.in_target = world_point;
      c.in_image = camera_point;

      corrs.push_back(c);
    }
  }

  setup.correspondences = corrs;
  setup.camera_to_target_guess = Eigen::Isometry3d::Identity();

  PnPResult res = rct_optimizations::optimize(setup);
  EXPECT_TRUE(res.converged);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
