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
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc.translate(Eigen::Vector3d(0.0,0.0,1.0));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));

  Eigen::AngleAxisd aa_ver;
  Eigen::Matrix3d m = camera_loc.rotation();
  aa_ver = m;

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
  //EXPECT_TRUE(output.r.std_dev < 1.0e-14);
  EXPECT_TRUE(output.p.std_dev < 1.0e-14);
  EXPECT_TRUE(output.yw.std_dev < 1.0e-14);

  //absolute value of location mean should still be very close to real
  std::cout << "angle values" << "\n";
  std::cout << abs(output.r.mean - aa_ver.axis()(0)) << " , " << abs(output.r.mean - aa_ver.axis()(1)) << " , " << abs(output.r.mean - aa_ver.axis()(2)) << ";\n";

  std::cout << "angle std_devs" << "\n";
  std::cout  << output.r.std_dev << ", "<< output.p.std_dev << ", " << output.yw.std_dev << ";\n";

  EXPECT_TRUE(abs(output.x.mean - camera_loc.translation()(0)) < 1.0e-14);
  EXPECT_TRUE(abs(output.y.mean - camera_loc.translation()(1)) < 1.0e-14);
  EXPECT_TRUE(abs(output.z.mean - camera_loc.translation()(2)) < 1.0e-14);
//  EXPECT_TRUE(abs(output.r.mean - aa_ver.axis()(0)) < 1.0e-14);
//  EXPECT_TRUE(abs(output.p.mean - aa_ver.axis()(1)) < 1.0e-14);
//  EXPECT_TRUE(abs(output.yw.mean - aa_ver.axis()(2)) < 1.0e-14);
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
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc.translate(Eigen::Vector3d(0.0,0.0,1.0));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));

  Eigen::AngleAxisd aa_ver;
  Eigen::Matrix3d m = camera_loc.rotation();
  aa_ver = m;

  std::cout << aa_ver.axis()(0) << " , "<< aa_ver.axis()(1) << " , " << aa_ver.axis()(2) << " ;\n";

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
     instance.correspondences[j].in_image(0) += wobblex;
     instance.correspondences[j].in_image(1) += wobbley;
   }

   perturbed_problem_set.push_back(instance);
  }

 PnPNoiseStat output = rct_optimizations::qualifyNoise2D(perturbed_problem_set);

  //Euler angle problems?
  std::cout << "angle values" << "\n";
  std::cout << abs(output.r.mean - aa_ver.axis()(0)) << " , " << abs(output.r.mean - aa_ver.axis()(1)) << " , " << abs(output.r.mean - aa_ver.axis()(2)) << ";\n";

  std::cout << "angle std_devs" << "\n";
  std::cout  << output.r.std_dev << ", "<< output.p.std_dev << ", " << output.yw.std_dev << ";\n";

  EXPECT_TRUE(output.x.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.y.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.z.std_dev < 1.5 * stddev);
//  EXPECT_TRUE(output.r.std_dev < 1.5 * stddev);
//  EXPECT_TRUE(output.p.std_dev < 1.5 * stddev);
//  EXPECT_TRUE(output.yw.std_dev < 1.5 * stddev);

  //absolute value of location mean should still be very close to 0
  EXPECT_TRUE(abs(output.x.mean - camera_loc.translation()(0)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.y.mean - camera_loc.translation()(1)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.z.mean - camera_loc.translation()(2)) < 1.5 * stddev);
//  EXPECT_TRUE(abs(output.r.mean - aa_ver.axis()(0)) < 1.5 * stddev);
//  EXPECT_TRUE(abs(output.p.mean - aa_ver.axis()(1)) < 1.5 * stddev);
//  EXPECT_TRUE(abs(output.yw.mean - aa_ver.axis()(2)) < 1.5 * stddev);
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
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc.translate(Eigen::Vector3d(0.0,0.0,1.0));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));

  Eigen::AngleAxisd aa_ver;
  Eigen::Matrix3d m = camera_loc.rotation();
  aa_ver = m;

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
   //EXPECT_TRUE(output.r.std_dev < 1.0e-14);
   EXPECT_TRUE(output.p.std_dev < 1.0e-14);
   EXPECT_TRUE(output.yw.std_dev < 1.0e-14);

   std::cout << "angle values" << "\n";
   std::cout << abs(output.r.mean - aa_ver.axis()(0)) << " , " << abs(output.r.mean - aa_ver.axis()(1)) << " , " << abs(output.r.mean - aa_ver.axis()(2)) << ";\n";

   std::cout << "angle std_devs" << "\n";
   std::cout  << output.r.std_dev << ", "<< output.p.std_dev << ", " << output.yw.std_dev << ";\n";

   EXPECT_TRUE(abs(output.x.mean - camera_loc.translation()(0)) < 1.0e-14);
   EXPECT_TRUE(abs(output.y.mean - camera_loc.translation()(1)) < 1.0e-14);
   EXPECT_TRUE(abs(output.z.mean - camera_loc.translation()(2)) < 1.0e-14);
//   EXPECT_TRUE(abs(output.r.mean - aa_ver.axis()(0)) < 1.0e-14);
//   EXPECT_TRUE(abs(output.p.mean - aa_ver.axis()(1)) < 1.0e-14);
//   EXPECT_TRUE(abs(output.yw.mean - aa_ver.axis()(2)) < 1.0e-14);
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
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  camera_loc.translate(Eigen::Vector3d(0.0,0.0,1.0));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));

  Eigen::AngleAxisd aa_ver;
  Eigen::Matrix3d m = camera_loc.rotation();
  aa_ver = m;

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

  EXPECT_TRUE(output.x.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.y.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.z.std_dev < 1.5 * stddev);
//  EXPECT_TRUE(output.r.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.p.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.yw.std_dev < 1.5 * stddev);

  //absolute value of location mean should still be very close to 0
  std::cout << "angle values" << "\n";
  std::cout << abs(output.r.mean - aa_ver.axis()(0)) << " , " << abs(output.r.mean - aa_ver.axis()(1)) << " , " << abs(output.r.mean - aa_ver.axis()(2)) << ";\n";

  std::cout << "angle std_devs" << "\n";
  std::cout  << output.r.std_dev << ", "<< output.p.std_dev << ", " << output.yw.std_dev << ";\n";

  EXPECT_TRUE(abs(output.x.mean - camera_loc.translation()(0)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.y.mean - camera_loc.translation()(1)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.z.mean - camera_loc.translation()(2)) < 1.5 * stddev);
//  EXPECT_TRUE(abs(output.r.mean - aa_ver.axis()(0)) < 1.5 * stddev);
//  EXPECT_TRUE(abs(output.p.mean - aa_ver.axis()(1)) < 1.5 * stddev);
//  EXPECT_TRUE(abs(output.yw.mean - aa_ver.axis()(2)) < 1.5 * stddev);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
