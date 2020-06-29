#include <gtest/gtest.h>
#include <rct_optimizations/validation/homography_check.h>
#include <rct_optimizations_tests/observation_creator.h>
#include <rct_optimizations_tests/utilities.h>

using namespace rct_optimizations;

/*plan:
 * each homography sample needs:
 * camera intr
 * target (row, col, spacing)
 * observation set
 * Threshold for error (max residual)
 *
 * A run returns:
 * a bool that it worked
*/
TEST(Homography, PerfectInitialConditions)
{
  test::Camera camera = test::makeKinectCamera();

  const double max_residual = 0.5;
  const unsigned target_rows = 5;
  const unsigned target_cols = 7;
  double spacing = 0.025;
  test::Target target(target_rows, target_cols, spacing);

  Eigen::Isometry3d target_to_camera(Eigen::Isometry3d::Identity());
  double x = static_cast<double>(target_rows - 1) * spacing / 2.0;
  double y = static_cast<double>(target_cols - 1) * spacing / 2.0;
  target_to_camera.translate(Eigen::Vector3d(x, y, 1.0));
  target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  rct_optimizations::Observation2D3D obs;
  EXPECT_NO_THROW(obs.correspondence_set=
                    test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), camera, target, true));

  auto ob = std::make_shared<rct_optimizations::Observation2D3D>(obs);
  bool res = checkObservationSequence(target_rows, target_cols, obs, max_residual);
  // bool res = checkObservationSequence(target_rows, target_cols, obs, max_residual);
  //bool res = true;
  EXPECT_TRUE(res);
}

//TEST(Homography, BadIntrinsicParameters)
//{
//  test::Camera camera = test::makeKinectCamera();

//  unsigned target_rows = 5;
//  unsigned target_cols = 7;
//  double spacing = 0.025;
//  test::Target target(target_rows, target_cols, spacing);

//  Eigen::Isometry3d target_to_camera(Eigen::Isometry3d::Identity());
//  double x = static_cast<double>(target_rows) * spacing / 2.0;
//  double y = static_cast<double>(target_cols) * spacing / 2.0;
//  target_to_camera.translate(Eigen::Vector3d(x, y, 1.0));
//  target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

//  PnPProblem problem;
//  problem.camera_to_target_guess = target_to_camera.inverse();
//  problem.correspondences = test::getCorrespondences(target_to_camera,
//                                                     Eigen::Isometry3d::Identity(),
//                                                     camera,
//                                                     target,
//                                                     true);

//  // Tweak the camera intrinsics such that we are optimizing with different values (+/- 1%)
//  // than were used to generate the observations
//  problem.intr = camera.intr;
//  problem.intr.fx() *= 1.01;
//  problem.intr.fy() *= 0.99;
//  problem.intr.cx() *= 1.01;
//  problem.intr.cy() *= 0.99;

//  PnPResult result = optimize(problem);

//  // The optimization should still converge by moving the camera further away from the target,
//  // but the residual error and error in the resulting transform should be much higher
//  EXPECT_TRUE(result.converged);
//  EXPECT_FALSE(result.camera_to_target.isApprox(target_to_camera.inverse(), 1.0e-3));
//  EXPECT_GT(result.final_cost_per_obs, 1.0e-3);
//}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
