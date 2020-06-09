#include <random>
#include <gtest/gtest.h>

#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>
#include <rct_optimizations/experimental/pnp.h>
#include <rct_optimizations/validation/noise_qualifier.h>

using namespace rct_optimizations;

TEST(NoiseTest, 2DPerfectTest)
{
  //make target
  test::Target target(4, 4, 0.025);

  //Load camera intrinsics
  //camera is a kinect
  auto camera = test::makeKinectCamera();

  //reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem> ideal_problem_set;
  ideal_problem_set.reserve(obs_cnt);

  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  target_loc.translate(Eigen::Vector3d(1.0,1.0,0.0));
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc = test::lookAt(camera_loc.translation(), target_loc.translation(), Eigen::Vector3d(1.0,0.0,0.0));

  //create observations
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    PnPProblem instance;
    //guess inital position
    instance.camera_to_target_guess= camera_loc;
    instance.intr = camera.intr;

    Correspondence2D3D::Set corr;
    EXPECT_NO_THROW
    (
    corr = getCorrespondences(camera_loc,
                              target_loc,
                              camera,
                              target,
                              true);
   );

   ideal_problem_set.push_back(instance);

  }

  std::vector<rct_optimizations::NoiseStatistics> output = rct_optimizations::qualifyNoise2D(ideal_problem_set);

  EXPECT_TRUE(output[0].std_dev < 1.0e-14);
  EXPECT_TRUE(output[1].std_dev < 1.0e-14);
  EXPECT_TRUE(output[2].std_dev < 1.0e-14);
  EXPECT_TRUE(output[3].std_dev < 1.0e-14);
  EXPECT_TRUE(output[4].std_dev < 1.0e-14);
  EXPECT_TRUE(output[5].std_dev < 1.0e-14);

  //absolute value of location mean should still be very close to 0
  EXPECT_TRUE(abs(output[0].mean) < 1.0e-15);
  EXPECT_TRUE(abs(output[1].mean) < 1.0e-15);
  EXPECT_TRUE(abs(output[2].mean) < 1.0e-15);
  //Euler angles are excluded because of singularities
}

TEST(NoiseTest, 2DNoiseTest)
{

  //make target
  test::Target target(4, 4, 0.025);

  //Load camera intrinsics
  //camera is a kinect
  auto camera = test::makeKinectCamera();

  //reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem> perturbed_problem_set;
  perturbed_problem_set.reserve(obs_cnt);

  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  target_loc.translate(Eigen::Vector3d(1.0,1.0,0.0));
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc = test::lookAt(camera_loc.translation(), target_loc.translation(), Eigen::Vector3d(1.0,0.0,0.0));

  //create observations
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    PnPProblem instance;
    //guess inital position
    instance.camera_to_target_guess = camera_loc;

    instance.intr = camera.intr;
    Correspondence2D3D::Set corr;

    EXPECT_NO_THROW(
      corr = getCorrespondences(camera_loc,
                                target_loc,
                                camera,
                                target,
                                true);
    );

    instance.correspondences = corr;

    //now add noise to correspondences
    const double mean = 0.0;
    const double stddev = 0.001;
    std::random_device rd{};
    std::mt19937 generator{rd()};
    std::normal_distribution<double> dist(mean, stddev);
    double wobblex = dist(generator);
    double wobbley = dist(generator);
    for (std::size_t j = 0;  i < perturbed_problem_set[j].correspondences.size(); ++j)
    {
      //all correspondences in one observation will have th same noise added
      instance.correspondences[j].in_image(0) += wobblex;
      instance.correspondences[j].in_image(1) += wobbley;
    }
    perturbed_problem_set.push_back(instance);

  }

  std::vector<rct_optimizations::NoiseStatistics> output = rct_optimizations::qualifyNoise2D(perturbed_problem_set);

  EXPECT_TRUE(output[0].std_dev < 0.0015);
  EXPECT_TRUE(output[1].std_dev < 0.0015);
  EXPECT_TRUE(output[2].std_dev < 0.0015);
  EXPECT_TRUE(output[3].std_dev < 0.0015);
  EXPECT_TRUE(output[4].std_dev < 0.0015);
  EXPECT_TRUE(output[5].std_dev < 0.0015);

  //absolute value of location mean should still be very close to 0
  EXPECT_TRUE(abs(output[0].mean) < 1.0e-10);
  EXPECT_TRUE(abs(output[1].mean) < 1.0e-10);
  EXPECT_TRUE(abs(output[2].mean) < 1.0e-10);
  //Euler angles are excluded because of singularities
}

TEST(NoiseTest, 3DPerfectTest)
{
  //make target
  test::Target target(4, 4, 0.025);


  //reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem3D> ideal_problem_set;
  ideal_problem_set.reserve(obs_cnt);

  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  target_loc.translate(Eigen::Vector3d(1.0,1.0,0.0));
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc = test::lookAt(camera_loc.translation(), target_loc.translation(), Eigen::Vector3d(1.0,0.0,0.0));


  //create observations
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    PnPProblem3D instance;
    //start with a perfect guess
    instance.camera_to_target_guess = camera_loc;

    Correspondence3D3D::Set corr;
    EXPECT_NO_THROW(
      corr = getCorrespondences(camera_loc,
                                target_loc,
                                target);
    );
    ideal_problem_set.push_back(instance);

  }

  std::vector<rct_optimizations::NoiseStatistics> output = rct_optimizations::qualifyNoise3D(ideal_problem_set);

  EXPECT_TRUE(output[0].std_dev < 1.0e-14);
  EXPECT_TRUE(output[1].std_dev < 1.0e-14);
  EXPECT_TRUE(output[2].std_dev < 1.0e-14);
  EXPECT_TRUE(output[3].std_dev < 1.0e-14);
  EXPECT_TRUE(output[4].std_dev < 1.0e-14);
  EXPECT_TRUE(output[5].std_dev < 1.0e-14);

  //absolute value of location mean should still be very close to 0
  EXPECT_TRUE(abs(output[0].mean) < 1.0e-15);
  EXPECT_TRUE(abs(output[1].mean) < 1.0e-15);
  EXPECT_TRUE(abs(output[2].mean) < 1.0e-15);
  //Euler angles are excluded because of singularities
}

TEST(NoiseTest, 3DNoiseTest)
{
  //make target
  test::Target target(4, 4, 0.025);


  //reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem3D> perturbed_problem_set;
  perturbed_problem_set.reserve(obs_cnt);

  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  target_loc.translate(Eigen::Vector3d(1.0,1.0,0.0));
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc = test::lookAt(camera_loc.translation(), target_loc.translation(), Eigen::Vector3d(1.0,0.0,0.0));


  //create observations
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    PnPProblem3D instance;

    //start with a perfect guess
    instance.camera_to_target_guess = camera_loc;

    Correspondence3D3D::Set corr;

    EXPECT_NO_THROW(
      corr = getCorrespondences(camera_loc,
                                target_loc,
                                target);
    );

    instance.correspondences = corr;


    //now add noise to correspondences
    const double mean = 0.0;
    const double stddev = 0.001;
    std::random_device rd{};
    std::mt19937 generator{rd()};
    std::normal_distribution<double> dist(mean, stddev);
    double wobblex = dist(generator);
    double wobbley = dist(generator);
    for (std::size_t j = 0;  i < perturbed_problem_set[j].correspondences.size(); ++j)
    {
      //all correspondences in one observation will have th same noise added
      instance.correspondences[j].in_image(0) += wobblex;
      instance.correspondences[j].in_image(1) += wobbley;
    }

    perturbed_problem_set.push_back(instance);

  }

  std::vector<rct_optimizations::NoiseStatistics> output = rct_optimizations::qualifyNoise3D(perturbed_problem_set);

  EXPECT_TRUE(output[0].std_dev < 0015);
  EXPECT_TRUE(output[1].std_dev < 0015);
  EXPECT_TRUE(output[2].std_dev < 0015);
  EXPECT_TRUE(output[3].std_dev < 0015);
  EXPECT_TRUE(output[4].std_dev < 0015);
  EXPECT_TRUE(output[5].std_dev < 0015);

  //absolute value of location mean should still be very close to 0
  EXPECT_TRUE(abs(output[0].mean) < 1.0e-10);
  EXPECT_TRUE(abs(output[1].mean) < 1.0e-10);
  EXPECT_TRUE(abs(output[2].mean) < 1.0e-10);
}
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
