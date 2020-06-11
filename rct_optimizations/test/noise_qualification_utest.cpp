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
  target_loc.translate(Eigen::Vector3d(0.5,0.5,-1.0));
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

   instance.correspondences = corr;
   ideal_problem_set.push_back(instance);

  }

 PnPNoiseStat output = rct_optimizations::qualifyNoise2D(ideal_problem_set);

  EXPECT_TRUE(output.x.std_dev < 1.0e-14);
  EXPECT_TRUE(output.y.std_dev < 1.0e-14);
  EXPECT_TRUE(output.z.std_dev < 1.0e-14);
  EXPECT_TRUE(output.r.std_dev < 1.0e-14);
  EXPECT_TRUE(output.p.std_dev < 1.0e-14);
  EXPECT_TRUE(output.yw.std_dev < 1.0e-14);

  //absolute value of location mean should still be very close to 0
  std::cout << output.x.mean << " , " << output.y.mean << " , " << output.z.mean << ";\n";
  //EXPECT_TRUE(abs(output.x.mean) < 1.0e-15);
  //EXPECT_TRUE(abs(output.y.mean) < 1.0e-15);
  //EXPECT_TRUE(abs(output.z.mean) < 1.0e-15);
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
  target_loc.translate(Eigen::Vector3d(0.5,0.5,-1.0));
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc = test::lookAt(camera_loc.translation(), target_loc.translation(), Eigen::Vector3d(1.0,0.0,0.0));


  //now add noise boilerplate
  const double mean = 0.0;
  const double stddev = 0.001;
  std::random_device rd{};
  std::mt19937 generator{rd()};
  std::normal_distribution<double> dist(mean, stddev);
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

   instance.correspondences = corr;

   for (std::size_t j = 0;  j < instance.correspondences.size(); ++j)
   {
     double wobblex = dist(generator);
     double wobbley = dist(generator);
     std::cout << wobblex << " , " << wobbley << ";\n";
     instance.correspondences[j].in_image(0) += wobblex;
     instance.correspondences[j].in_image(1) += wobbley;
   }

   perturbed_problem_set.push_back(instance);
  }

 PnPNoiseStat output = rct_optimizations::qualifyNoise2D(perturbed_problem_set);

  EXPECT_TRUE(output.x.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.y.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.z.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.r.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.p.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.yw.std_dev < 1.5 * stddev);

  //absolute value of location mean should still be very close to 0
  std::cout << output.x.mean << " , " << output.y.mean << " , " << output.z.mean << ";\n";
  //EXPECT_TRUE(abs(output.x.mean) < 1.0e-15);
  //EXPECT_TRUE(abs(output.y.mean) < 1.0e-15);
  //EXPECT_TRUE(abs(output.z.mean) < 1.0e-15);
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
  target_loc.translate(Eigen::Vector3d(0.5,0.5,-1.0));
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
    ideal_problem_set.push_back(instance);

  }

   PnPNoiseStat output = rct_optimizations::qualifyNoise3D(ideal_problem_set);

   EXPECT_TRUE(output.x.std_dev < 1.0e-14);
   EXPECT_TRUE(output.y.std_dev < 1.0e-14);
   EXPECT_TRUE(output.z.std_dev < 1.0e-14);
   EXPECT_TRUE(output.r.std_dev < 1.0e-14);
   EXPECT_TRUE(output.p.std_dev < 1.0e-14);
   EXPECT_TRUE(output.yw.std_dev < 1.0e-14);

   std::cout << output.x.mean << " , " << output.y.mean << " , " << output.z.mean << ";\n";
   //absolute value of location mean should still be very close to 0
   //EXPECT_TRUE(abs(output.x.mean) < 1.0e-15);
   //EXPECT_TRUE(abs(output.y.mean) < 1.0e-15);
   //EXPECT_TRUE(abs(output.z.mean) < 1.0e-15);
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
  target_loc.translate(Eigen::Vector3d(0.5,0.5,-1.0));
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc = test::lookAt(camera_loc.translation(), target_loc.translation(), Eigen::Vector3d(1.0,0.0,0.0));

  //Noise boilerplate
  const double mean = 0.0;
  const double stddev = 0.001;
  std::random_device rd{};
  std::mt19937 generator{rd()};
  std::normal_distribution<double> dist(mean, stddev);

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
    for (std::size_t j = 0;  j < instance.correspondences.size(); ++j)
    {
      double wobblex = dist(generator);
      double wobbley = dist(generator);
      instance.correspondences[j].in_image(0) += wobblex;
      instance.correspondences[j].in_image(1) += wobbley;
    }

    perturbed_problem_set.push_back(instance);

  }

  PnPNoiseStat output = rct_optimizations::qualifyNoise3D(perturbed_problem_set);

  EXPECT_TRUE(output.x.std_dev < 1.5* stddev);
  EXPECT_TRUE(output.y.std_dev < 1.5* stddev);
  EXPECT_TRUE(output.z.std_dev < 1.5* stddev);
  EXPECT_TRUE(output.r.std_dev < 1.5* stddev);
  EXPECT_TRUE(output.p.std_dev < 1.5* stddev);
  EXPECT_TRUE(output.yw.std_dev < 1.5* stddev);

  //absolute value of location mean should still be very close to 0
  std::cout << output.x.mean << " , " << output.y.mean << " , " << output.z.mean << ";\n";
  //EXPECT_TRUE(abs(output.x.mean) < 1.0e-10);
  //EXPECT_TRUE(abs(output.y.mean) < 1.0e-10);
  //EXPECT_TRUE(abs(output.z.mean) < 1.0e-10);
  //Euler angles are excluded because of singularities

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
