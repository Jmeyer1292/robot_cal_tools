#include <random>
#include <gtest/gtest.h>

#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>
#include <rct_optimizations/experimental/pnp.h>
#include <rct_optimizations/validation/noise_qualifier.h>

using namespace rct_optimizations;

TEST(NoiseTest, 3DNoiseTest)
{
  //make target
  test::Target targeta(5, 7, 0.025);


  //reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem3D> perturbed_problem_set;
  perturbed_problem_set.reserve(obs_cnt);

  //create poses outside the loop; would be better to decalre pose, then look at from there
  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  target_loc.translate(Eigen::Vector3d(1.0,0.0,0.0));
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc = test::lookAt(camera_loc.translation(), target_loc.translation(), Eigen::Vector3d(1.0,0.0,0.0));


  //create observations
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    //start with a perfect guess
    Eigen::Isometry3d guess = camera_loc;

    perturbed_problem_set[i].camera_to_target_guess = guess;

    EXPECT_NO_THROW(
      perturbed_problem_set[i].correspondences = getCorrespondences(camera_loc,
                                                                    target_loc,
                                                                    targeta);
    );

    //now add noise to correspondences
    const double mean = 0.0;
    const double stddev = 0.001;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    double wobblex = dist(generator);
    double wobbley = dist(generator);
    for (std::size_t j = 0;  i < perturbed_problem_set[j].correspondences.size(); ++j)
    {
      //all correspondences in one observation will have th same noise added
      perturbed_problem_set[i].correspondences[j].in_image(0) += wobblex;
      perturbed_problem_set[i].correspondences[j].in_image(1) += wobbley;
    }

  }

  std::vector<rct_optimizations::NoiseStatistics> output = rct_optimizations::qualifyNoise3D(perturbed_problem_set);

  EXPECT_TRUE(output[0].std_dev < 0015);
  EXPECT_TRUE(output[1].std_dev < 0015);
  EXPECT_TRUE(output[2].std_dev < 0015);
  EXPECT_TRUE(output[3].std_dev < 0015);
  EXPECT_TRUE(output[4].std_dev < 0015);
  EXPECT_TRUE(output[5].std_dev < 0015);

  //absolute value of mean should still be very close to 0
//  EXPECT_TRUE(abs(output[0].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[1].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[2].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[3].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[4].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[5].mean) < 1.0e-10);
}

TEST(NoiseTest, 2DNoiseTest)
{

  //make target
  test::Target targetb(5, 7, 0.025);

  //Load camera intrinsics
  //camera is a kinect
  auto camera = test::makeKinectCamera();

  //reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem> perturbed_problem_set;
  perturbed_problem_set.reserve(obs_cnt);

  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  target_loc.translate(Eigen::Vector3d(1.0,0.0,0.0));
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc = test::lookAt(camera_loc.translation(), target_loc.translation(), Eigen::Vector3d(1.0,0.0,0.0));

  //create observations
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    //guess inital position
    Eigen::Isometry3d guess = camera_loc;

    perturbed_problem_set[i].intr = camera.intr;
    perturbed_problem_set[i].camera_to_target_guess = guess;
    EXPECT_NO_THROW(
      perturbed_problem_set[i].correspondences = getCorrespondences(camera_loc,
                                               target_loc,
                                               camera,
                                               targetb,
                                               true);
    );

    //now add noise to correspondences
    const double mean = 0.0;
    const double stddev = 0.001;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    double wobblex = dist(generator);
    double wobbley = dist(generator);
    for (std::size_t j = 0;  i < perturbed_problem_set[j].correspondences.size(); ++j)
    {
      //all correspondences in one observation will have th same noise added
      perturbed_problem_set[i].correspondences[j].in_image(0) += wobblex;
      perturbed_problem_set[i].correspondences[j].in_image(1) += wobbley;
    }

  }

  std::vector<rct_optimizations::NoiseStatistics> output = rct_optimizations::qualifyNoise2D(perturbed_problem_set);

  EXPECT_TRUE(output[0].std_dev < 0.0015);
  EXPECT_TRUE(output[1].std_dev < 0.0015);
  EXPECT_TRUE(output[2].std_dev < 0.0015);
  EXPECT_TRUE(output[3].std_dev < 0.0015);
  EXPECT_TRUE(output[4].std_dev < 0.0015);
  EXPECT_TRUE(output[5].std_dev < 0.0015);

  //absolute value of mean should still be very close to 0
//  EXPECT_TRUE(abs(output[0].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[1].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[2].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[3].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[4].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[5].mean) < 1.0e-10);
}

TEST(NoiseTest, 2DPerfectTest)
{
  //make target
  test::Target targetc(5, 7, 0.025);

  //Load camera intrinsics
  //camera is a kinect
  auto camera = test::makeKinectCamera();

  //reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem> ideal_problem_set;
  ideal_problem_set.reserve(obs_cnt);

  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  target_loc.translate(Eigen::Vector3d(1.0,0.0,0.0));
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc = test::lookAt(camera_loc.translation(), target_loc.translation(), Eigen::Vector3d(1.0,0.0,0.0));

  //create observations
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    //guess inital position
    Eigen::Isometry3d guess = camera_loc;

    EXPECT_NO_THROW
    (
    ideal_problem_set[i].intr = camera.intr;
    ideal_problem_set[i].camera_to_target_guess = guess;
    ideal_problem_set[i].correspondences = getCorrespondences(camera_loc,
                                             target_loc,
                                             camera,
                                             targetc,
                                             true);
   );
  }

  std::vector<rct_optimizations::NoiseStatistics> output = rct_optimizations::qualifyNoise2D(ideal_problem_set);

  EXPECT_TRUE(output[0].std_dev < 1.0e-14);//There is more error than expected here
  EXPECT_TRUE(output[1].std_dev < 1.0e-14);//There is more error than expected here
  EXPECT_TRUE(output[2].std_dev < 1.0e-14);//There is more error than expected here
  EXPECT_TRUE(output[3].std_dev < 1.0e-14);
  EXPECT_TRUE(output[4].std_dev < 1.0e-14);
  EXPECT_TRUE(output[5].std_dev < 1.0e-14);

  //absolute value of mean should still be very close to 0
  //EXPECT_TRUE(abs(output[0].mean) < 1.0e-5);
  //EXPECT_TRUE(abs(output[1].mean) < 1.0e-5);
  //EXPECT_TRUE(abs(output[2].mean) < 1.0e-5);
  //EXPECT_TRUE(abs(output[3].mean) < 1.0e-5);
  //EXPECT_TRUE(abs(output[4].mean) < 1.0e-5);
  //EXPECT_TRUE(abs(output[5].mean) < 1.0e-5);

}

TEST(NoiseTest, 3DPerfectTest)
{
  //make target
  test::Target targetd(5, 7, 0.025);


  //reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem3D> problem_set;
  problem_set.reserve(obs_cnt);

  //create poses outside the loop; would be better to decalre pose, then look at from there
  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  target_loc.translate(Eigen::Vector3d(1.0,0.0,0.0));
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc = test::lookAt(camera_loc.translation(), target_loc.translation(), Eigen::Vector3d(1.0,0.0,0.0));


  //create observations
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {
    //start with a perfect guess
    Eigen::Isometry3d guess = camera_loc;

    problem_set[i].camera_to_target_guess = guess;
    problem_set[i].correspondences = getCorrespondences(camera_loc,
                                             target_loc,
                                             targetd);

  }

  std::vector<rct_optimizations::NoiseStatistics> output = rct_optimizations::qualifyNoise3D(problem_set);

  EXPECT_TRUE(output[0].std_dev < 1.0e-14);
  EXPECT_TRUE(output[1].std_dev < 1.0e-14);
  EXPECT_TRUE(output[2].std_dev < 1.0e-14);
  EXPECT_TRUE(output[3].std_dev < 1.0e-14);
  EXPECT_TRUE(output[4].std_dev < 1.0e-14);
  EXPECT_TRUE(output[5].std_dev < 1.0e-14);

  //absolute value of mean should still be very close to 0
//  EXPECT_TRUE(abs(output[0].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[1].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[2].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[3].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[4].mean) < 1.0e-10);
//  EXPECT_TRUE(abs(output[5].mean) < 1.0e-10);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
