#include <gtest/gtest.h>
#include <rct_optimizations/extrinsic_hand_eye.h>
#include <rct_optimizations/ceres_math_utilities.h>

// Test utilities
#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>

using namespace rct_optimizations;

namespace
{
static void printResults(const ExtrinsicHandEyeResult& r)
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

} // namespace anonymous

void run_test(InitialConditions condition)
{
  test::Target grid(5, 7, 0.025);

  // Define the "true" conditions
  auto true_base_to_target = Eigen::Isometry3d::Identity();
  true_base_to_target.translation() = Eigen::Vector3d(1.0, 0, 0);

  auto true_wrist_to_camera = Eigen::Isometry3d::Identity();
  true_wrist_to_camera.translation() = Eigen::Vector3d(0.05, 0, 0.1);
  true_wrist_to_camera.linear() <<  0,  0,  1,
                                   -1,  0,  0,
                                    0, -1,  0;

  // Set up the calibration problem
  ExtrinsicHandEyeProblem3D3D problem;

  // Create some number of "test" images...
  // We'll take pictures in a grid above the origin of the target
  for (int i = -5; i < 5; ++i)
  {
    for (int j = -5; j < 5; ++j)
    {
      Eigen::Vector3d center_point = true_base_to_target.translation() + Eigen::Vector3d(i * 0.025, j * 0.025, 1.0);
      Eigen::Isometry3d camera_pose = test::lookAt(center_point,
                                                   true_base_to_target.translation(),
                                                   Eigen::Vector3d(1, 0, 0));
      Eigen::Isometry3d wrist_pose = camera_pose * true_wrist_to_camera.inverse();

      // Attempt to generate points
      try
      {
        Observation3D3D obs;
        obs.correspondence_set = getCorrespondences(camera_pose, true_base_to_target, grid);
        obs.base_to_camera_mount = wrist_pose;
        problem.observations.push_back(obs);
      }
      catch (const std::exception& ex)
      {
        continue;
      }
    }
  }

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

TEST(HandEye3D, perfect_start)
{
  run_test(InitialConditions::PERFECT);
}

TEST(HandEye3D, identity_target_start)
{
  run_test(InitialConditions::IDENTITY_TARGET);
}

TEST(HandEye3D, perturbed_start)
{
  // Run 10 random tests
  for (int i = 0; i < 10; ++i)
    run_test(InitialConditions::RANDOM_AROUND_ANSWER);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
