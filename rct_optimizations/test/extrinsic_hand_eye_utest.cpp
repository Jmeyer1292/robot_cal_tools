#include <gtest/gtest.h>
#include <rct_optimizations/extrinsic_hand_eye.h>
#include <rct_optimizations/ceres_math_utilities.h>

// Test utilities
#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>
#include <memory>

using namespace rct_optimizations;

enum class InitialConditions
{
  PERFECT,
  RANDOM_AROUND_ANSWER
};

template<typename ProblemT>
struct ProblemCreator
{
  static Eigen::Isometry3d createPose(const Eigen::Isometry3d &pose, const InitialConditions &init)
  {
    Eigen::Isometry3d out(pose);
    switch (init)
    {
    case InitialConditions::RANDOM_AROUND_ANSWER:
    {
      const double spatial_noise = 0.25; // +/- 0.25 meters
      const double angular_noise = 45. * M_PI / 180.0; // +/- 45 degrees
      out = test::perturbPose(pose, spatial_noise, angular_noise);
      break;
    }
    default: // PERFECT
      out = pose;
    }

    return out;
  }

  static ProblemT createProblem(const Eigen::Isometry3d &true_target,
                                const Eigen::Isometry3d &true_camera,
                                const test::PoseGenerator& pose_generator,
                                const test::Target &target,
                                const InitialConditions &init);

  /** @brief The maximum allowable cost per observation for this problem
   * Note: this cost is squared (i.e. pixels^2 or m^2) */
  static double max_cost_per_obs;
};

template<>
ExtrinsicHandEyeProblem2D3D ProblemCreator<ExtrinsicHandEyeProblem2D3D>::createProblem(
  const Eigen::Isometry3d &true_target,
  const Eigen::Isometry3d &true_camera,
  const test::PoseGenerator& pose_generator,
  const test::Target &target,
  const InitialConditions &init)
{
  test::Camera camera = test::makeKinectCamera();

  ExtrinsicHandEyeProblem2D3D problem;
  problem.intr = camera.intr;
  problem.target_mount_to_target_guess = createPose(true_target, init);
  problem.camera_mount_to_camera_guess = createPose(true_camera, init);
  problem.observations = test::createObservations(camera,
                                                  target,
                                                  pose_generator,
                                                  true_target,
                                                  true_camera);

  return problem;
}

template<>
double ProblemCreator<ExtrinsicHandEyeProblem2D3D>::max_cost_per_obs = 1.0;

template<>
ExtrinsicHandEyeProblem3D3D ProblemCreator<ExtrinsicHandEyeProblem3D3D>::createProblem(
  const Eigen::Isometry3d &true_target,
  const Eigen::Isometry3d &true_camera,
  const test::PoseGenerator& pose_generator,
  const test::Target &target,
  const InitialConditions &init)
{
  ExtrinsicHandEyeProblem3D3D problem;
  problem.target_mount_to_target_guess = createPose(true_target, init);
  problem.camera_mount_to_camera_guess = createPose(true_camera, init);
  problem.observations = test::createObservations(target, pose_generator, true_target, true_camera);

  return problem;
}

template<>
double ProblemCreator<ExtrinsicHandEyeProblem3D3D>::max_cost_per_obs = std::pow(1.0e-6, 2.0);

template<typename ProblemT>
class HandEyeTest : public ::testing::Test
{
public:
  HandEyeTest()
    : true_target_mount_to_target(Eigen::Isometry3d::Identity())
    , true_camera_mount_to_camera(Eigen::Isometry3d::Identity())
    , target(5, 7, 0.025)
  {
    true_target_mount_to_target.translate(Eigen::Vector3d(1.0, 0, 0.0));

    true_camera_mount_to_camera.translation() = Eigen::Vector3d(0.05, 0, 0.1);
    true_camera_mount_to_camera.linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  }

  static void printResults(const ExtrinsicHandEyeResult &r)
  {
    // Report results
    std::cout << "Did converge?: " << r.converged << "\n";
    std::cout << "Initial cost?: " << r.initial_cost_per_obs << "\n";
    std::cout << "Final cost?: " << r.final_cost_per_obs << "\n";

    Eigen::Isometry3d c = r.camera_mount_to_camera;
    Eigen::Isometry3d t = r.target_mount_to_target;

    std::cout << "Wrist to Camera:\n";
    std::cout << c.matrix() << "\n";
    std::cout << "Base to Target:\n";
    std::cout << t.matrix() << "\n";
  }

  Eigen::Isometry3d true_target_mount_to_target;
  Eigen::Isometry3d true_camera_mount_to_camera;
  test::Target target;
};

using Implementations = ::testing::Types<ExtrinsicHandEyeProblem2D3D, ExtrinsicHandEyeProblem3D3D>;

TYPED_TEST_CASE(HandEyeTest, Implementations);

TYPED_TEST(HandEyeTest, PerfectInitialConditions)
{
  test::HemispherePoseGenerator pg;
  TypeParam prob = ProblemCreator<TypeParam>::createProblem(this->true_target_mount_to_target,
                                                            this->true_camera_mount_to_camera,
                                                            pg,
                                                            this->target,
                                                            InitialConditions::PERFECT);
  // Run the optimization
  // Expect the function not to throw an exception because it is starting with perfect initial conditions
  ExtrinsicHandEyeResult result;
  EXPECT_NO_THROW(result = optimize(prob));

  // Make sure it converged to the correct answer
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.final_cost_per_obs < ProblemCreator<TypeParam>::max_cost_per_obs);

  EXPECT_TRUE(result.target_mount_to_target.isApprox(this->true_target_mount_to_target, 1e-6));
  EXPECT_TRUE(result.camera_mount_to_camera.isApprox(this->true_camera_mount_to_camera, 1e-6));

  this->printResults(result);
}

TYPED_TEST(HandEyeTest, RandomAroundAnswerInitialConditions)
{
  const std::size_t n = 10;
  const std::size_t max_attempts = 2 * n;
  std::size_t count = 0;

  test::HemispherePoseGenerator pg;

  while(count < n && count < max_attempts)
  {
    TypeParam prob
      = ProblemCreator<TypeParam>::createProblem(this->true_target_mount_to_target,
                                                 this->true_camera_mount_to_camera,
                                                 pg,
                                                 this->target,
                                                 InitialConditions::RANDOM_AROUND_ANSWER);
    // Run the optimization
    // Catch exceptions during the optimization because there is a good chance that infeasible initial
    // transform guesses will be randomly generated
    ExtrinsicHandEyeResult result;
    try
    {
      result = optimize(prob);
    }
    catch (const std::exception &ex)
    {
      std::cout << "Exception from optimization; continuing: '" << ex.what() << "'" << std::endl;
      continue;
    }

    ++count;

    // Make sure it converged to the correct answer
    EXPECT_TRUE(result.converged);
    EXPECT_TRUE(result.final_cost_per_obs < ProblemCreator<TypeParam>::max_cost_per_obs);

    EXPECT_TRUE(result.target_mount_to_target.isApprox(this->true_target_mount_to_target, 1e-6));
    EXPECT_TRUE(result.camera_mount_to_camera.isApprox(this->true_camera_mount_to_camera, 1e-6));

    this->printResults(result);
  }
  EXPECT_EQ(count, n);
  EXPECT_LT(count, max_attempts);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
