#include <gtest/gtest.h>
#include <rct_optimizations/extrinsic_camera_on_wrist.h>
#include <rct_optimizations/ceres_math_utilities.h>

// Test utilities
#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>
#include <rct_optimizations_tests/pose_generator.h>

using namespace rct_optimizations;

namespace
{
static void printResults(const ExtrinsicCameraOnWristResult& r)
{
  // Report results
  std::cout << "Did converge?: " << r.converged << "\n";
  std::cout << "Initial cost?: " << r.initial_cost_per_obs << "\n";
  std::cout << "Final cost?: " << r.final_cost_per_obs << "\n";

  Eigen::Isometry3d c = r.wrist_to_camera;
  Eigen::Isometry3d t = r.base_to_target;

  std::cout << "Wrist to Camera:\n";
  std::cout << c.matrix() << "\n";
  std::cout << "Base to Target:\n";
  std::cout << t.matrix() << "\n";
}

enum class InitialConditions
{
  PERFECT, IDENTITY_TARGET, RANDOM_AROUND_ANSWER
};

enum class CameraPattern
{
  HEMISPHERE, CONE, GRID
};

} // namespace anonymous

void run_test(InitialConditions condition, CameraPattern pattern)
{

  auto camera = test::makeKinectCamera();
  test::Target grid(5, 7, 0.025);

  // Define the "true" conditions
  auto true_base_to_target = Eigen::Isometry3d::Identity();
  true_base_to_target.translation() = Eigen::Vector3d(1.0, 0, 0);

  auto true_wrist_to_camera = Eigen::Isometry3d::Identity();
  true_wrist_to_camera.translation() = Eigen::Vector3d(0.05, 0, 0.1);
  true_wrist_to_camera.linear() <<  0,  0,  1,
                                   -1,  0,  0,
                                    0, -1,  0;

  // Create some number of "test" images...
  //    We'll take pictures in a grid above the origin of the target
  std::vector<Eigen::Isometry3d> wrist_poses;

  //std::vector<Eigen::Affine3d> wrist_poses;
  std::vector<rct_optimizations::CorrespondenceSet> correspondences;

  std::vector<Eigen::Isometry3d>camera_poses;

  if(pattern == CameraPattern::HEMISPHERE)
  {
    const double radius = 2.0;
    const unsigned int theta_cnt = 10;
    const unsigned int phi_cnt = 10;

    //hemisphere poses: radius 2, 10 rows of 10 observations
    camera_poses = rct_optimizations::test::genHemispherePose(true_base_to_target.translation(),
                                                             radius,
                                                             theta_cnt,
                                                             phi_cnt
                                                             );
  }
  if(pattern == CameraPattern::CONE)
  {
    const double radius = 1.0;
    const double h = 2.0;
    const unsigned int observations = 20;
    //conical poses: 20 poses in a 1 meter radius cone at a distance of 2 meters
    camera_poses = rct_optimizations::test::genConicalPose(true_base_to_target.translation(),
                                                           observations,
                                                           radius,
                                                           h
                                                           );
  }
  else //GRID
  {
    const double spacing = 0.2;
    const double h = 2.0;
    unsigned int grid_side = 10;
    //grid poses: 100 poses at a distance of 2 meters with 0.2 meter spacing
    camera_poses = rct_optimizations::test::genGridPose(true_base_to_target.translation(),
                                                           grid_side,
                                                           spacing,
                                                           h
                                                           );
  }
 for (auto& pose : camera_poses)
 {
     Eigen::Isometry3d wrist_pose = pose * true_wrist_to_camera.inverse();

       // Attempt to generate points
       try
       {
         CorrespondenceSet corr = getCorrespondences(pose,
                                                     true_base_to_target,
                                                     camera,
                                                     grid,
                                                     true);
         correspondences.push_back(corr);
         wrist_poses.push_back(wrist_pose);
       }
       catch (const std::exception& ex)
       {
         continue;
       }
  }


  // Fill out the calibration
  ExtrinsicCameraOnWristProblem problem;
  problem.intr = camera.intr;
  problem.wrist_poses = wrist_poses;
  problem.image_observations = correspondences;
  // Set the initial guess
  if (condition == InitialConditions::PERFECT)
  {
    problem.base_to_target_guess = true_base_to_target;
    problem.wrist_to_camera_guess = true_wrist_to_camera;
  }
  else if (condition == InitialConditions::IDENTITY_TARGET)
  {
    problem.base_to_target_guess.setIdentity();
    problem.wrist_to_camera_guess = true_wrist_to_camera;
  }
  else // condition == RANDOM
  {
    const double spatial_noise = 0.25; // +/- 0.25 meters
    const double angular_noise = 45. * M_PI / 180.0; // +/- 45 degrees
    problem.base_to_target_guess = test::perturbPose(true_base_to_target, spatial_noise, angular_noise);
    problem.wrist_to_camera_guess = test::perturbPose(true_wrist_to_camera, spatial_noise, angular_noise);
    std::cout << "initial base to target:\n" << problem.base_to_target_guess.matrix() << "\n";
    std::cout << "initial wrist to camera:\n" << problem.wrist_to_camera_guess.matrix() << "\n";
  }

  // Run the optimization
  auto result = optimize(problem);

  // Make sure it converged to the correct answer
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.final_cost_per_obs < 1.0);

  EXPECT_TRUE(result.base_to_target.isApprox(true_base_to_target, 1e-6));
  EXPECT_TRUE(result.wrist_to_camera.isApprox(true_wrist_to_camera, 1e-6));

  printResults(result);
}

TEST(CameraOnWrist, perfect_start_hemisphere)
{
  run_test(InitialConditions::PERFECT, CameraPattern::HEMISPHERE);
}

TEST(CameraOnWrist, perfect_start_cone)
{
  run_test(InitialConditions::PERFECT, CameraPattern::CONE);
}

TEST(CameraOnWrist, perfect_start_grid)
{
  run_test(InitialConditions::PERFECT, CameraPattern::GRID);
}

TEST(CameraOnWrist, identity_target_start_hemisphere)
{
  run_test(InitialConditions::IDENTITY_TARGET, CameraPattern::HEMISPHERE);
}

TEST(CameraOnWrist, identity_target_start_cone)
{
  run_test(InitialConditions::IDENTITY_TARGET, CameraPattern::CONE);
}

//TEST(CameraOnWrist, identity_target_start_grid)
//{
//  run_test(InitialConditions::IDENTITY_TARGET, CameraPattern::GRID);
//}


TEST(CameraOnWrist, perturbed_start)
{
  // Run 10 random tests
  for (int i = 0; i < 10; ++i)
    run_test(InitialConditions::RANDOM_AROUND_ANSWER, CameraPattern::HEMISPHERE);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
