#include <gtest/gtest.h>
#include <rct_optimizations/pnp.h>
#include <rct_optimizations_tests/observation_creator.h>
#include <rct_optimizations_tests/utilities.h>

using namespace rct_optimizations;

static const unsigned TARGET_ROWS = 10;
static const unsigned TARGET_COLS = 14;
static const double SPACING = 0.025;
static const double CORRELATION_COEFFICIENT_THRESHOLD = 0.8;

TEST(PNP_2D, PerfectInitialConditions)
{
  test::Camera camera = test::makeKinectCamera();
  test::Target target(TARGET_ROWS, TARGET_COLS, SPACING);

  Eigen::Isometry3d target_to_camera(Eigen::Isometry3d::Identity());
  double x = static_cast<double>(TARGET_ROWS - 1) * SPACING / 2.0;
  double y = static_cast<double>(TARGET_COLS - 1) * SPACING / 2.0;
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

  for (Eigen::Index row = 0; row < result.camera_to_target_covariance.rows(); ++row)
  {
    for (Eigen::Index col = 0; col < result.camera_to_target_covariance.cols(); ++col)
    {
      if(row != col)
        EXPECT_LT(std::abs(result.camera_to_target_covariance(row, col)), CORRELATION_COEFFICIENT_THRESHOLD);
    }
  }

//  Eigen::IOFormat fmt(4, 0, " | ", "\n", "|", "|");
//  std::cout << "Covariance:\n" << result.camera_to_target_covariance.format(fmt) << std::endl;
}

TEST(PNP_2D, PerturbedInitialCondition)
{
  test::Camera camera = test::makeKinectCamera();
  test::Target target(TARGET_ROWS, TARGET_COLS, SPACING);

  Eigen::Isometry3d target_to_camera(Eigen::Isometry3d::Identity());
  double x = static_cast<double>(TARGET_ROWS) * SPACING / 2.0;
  double y = static_cast<double>(TARGET_COLS) * SPACING / 2.0;
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

  for (Eigen::Index row = 0; row < result.camera_to_target_covariance.rows(); ++row)
  {
    for (Eigen::Index col = 0; col < result.camera_to_target_covariance.cols(); ++col)
    {
      if (row != col)
        EXPECT_LT(std::abs(result.camera_to_target_covariance(row, col)), CORRELATION_COEFFICIENT_THRESHOLD);
    }
  }

//  Eigen::IOFormat fmt(4, 0, " | ", "\n", "|", "|");
//  std::cout << "Covariance:\n" << result.camera_to_target_covariance.format(fmt) << std::endl;
}

TEST(PNP_2D, BadIntrinsicParameters)
{
  test::Camera camera = test::makeKinectCamera();
  test::Target target(TARGET_ROWS, TARGET_COLS, SPACING);

  Eigen::Isometry3d target_to_camera(Eigen::Isometry3d::Identity());
  double x = static_cast<double>(TARGET_ROWS) * SPACING / 2.0;
  double y = static_cast<double>(TARGET_COLS) * SPACING / 2.0;
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

  for (Eigen::Index row = 0; row < result.camera_to_target_covariance.rows(); ++row)
  {
    for (Eigen::Index col = 0; col < result.camera_to_target_covariance.cols(); ++col)
    {
      if (row != col)
        EXPECT_LT(std::abs(result.camera_to_target_covariance(row, col)), CORRELATION_COEFFICIENT_THRESHOLD);
    }
  }

//  Eigen::IOFormat fmt(4, 0, " | ", "\n", "|", "|");
//  std::cout << "Covariance:\n" << result.camera_to_target_covariance.format(fmt) << std::endl;
}

TEST(PNP_3D, PerfectInitialConditions)
{
  unsigned target_rows = 5;
  unsigned target_cols = 7;
  double spacing = 0.025;
  test::Target target(target_rows, target_cols, spacing);

  Eigen::Isometry3d target_to_camera(Eigen::Isometry3d::Identity());
  double x = static_cast<double>(target_rows - 1) * spacing / 2.0;
  double y = static_cast<double>(target_cols - 1) * spacing / 2.0;
  target_to_camera.translate(Eigen::Vector3d(x, y, 1.0));
  target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  PnPProblem3D problem;
  problem.camera_to_target_guess = target_to_camera.inverse();
  EXPECT_NO_THROW(problem.correspondences =
                    test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), target));

  PnPResult result = optimize(problem);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.camera_to_target.isApprox(target_to_camera.inverse()));
  EXPECT_LT(result.initial_cost_per_obs, 1.0e-15);
  EXPECT_LT(result.final_cost_per_obs, 1.0e-15);
}

TEST(PNP_3D, PerturbedInitialCondition)
{

  unsigned target_rows = 5;
  unsigned target_cols = 7;
  double spacing = 0.025;
  test::Target target(target_rows, target_cols, spacing);

  Eigen::Isometry3d target_to_camera(Eigen::Isometry3d::Identity());
  double x = static_cast<double>(target_rows) * spacing / 2.0;
  double y = static_cast<double>(target_cols) * spacing / 2.0;
  target_to_camera.translate(Eigen::Vector3d(x, y, 1.0));
  target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  PnPProblem3D problem;
  problem.camera_to_target_guess = test::perturbPose(target_to_camera.inverse(), 0.05, 0.05);
  problem.correspondences = test::getCorrespondences(target_to_camera,
                                                     Eigen::Isometry3d::Identity(),
                                                     target);

  PnPResult result = optimize(problem);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.camera_to_target.isApprox(target_to_camera.inverse(), 0.1));
  EXPECT_LT(result.final_cost_per_obs, 1.0e-14);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
