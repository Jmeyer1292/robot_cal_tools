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

  Eigen::Quaterniond q_ver;
  Eigen::Matrix3d m = camera_loc.rotation();
  q_ver = m;

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
  EXPECT_TRUE(output.i.std_dev < 1.0e-14);
  EXPECT_TRUE(output.j.std_dev < 1.0e-14);
  EXPECT_TRUE(output.k.std_dev < 1.0e-14);
  EXPECT_TRUE(output.w.std_dev < 1.0e-14);

  //absolute value of location mean should still be very close to real
  std::cout << "angle values" << "\n";
  std::cout << abs(output.i.mean - q_ver.x()) << " , " << abs(output.j.mean - q_ver.y()) << " , " << abs(output.k.mean - q_ver.z()) << " , " << abs(output.w.mean - q_ver.w())<< ";\n";

  std::cout << "angle std_devs" << "\n";
  std::cout  << output.i.std_dev << ", "<< output.j.std_dev << ", " << output.k.std_dev << ", " << output.w.std_dev << ";\n";

  EXPECT_TRUE(abs(output.x.mean - camera_loc.translation()(0)) < 1.0e-14);
  EXPECT_TRUE(abs(output.y.mean - camera_loc.translation()(1)) < 1.0e-14);
  EXPECT_TRUE(abs(output.z.mean - camera_loc.translation()(2)) < 1.0e-14);
  EXPECT_TRUE(abs(output.w.mean - q_ver.w()) < 1.0e-14);
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

  Eigen::Quaterniond q_ver;
  Eigen::Matrix3d m = camera_loc.rotation();
  q_ver = m;

  std::cout << q_ver.x() << " , " <<q_ver.y() << " , " << q_ver.z() << " , " << q_ver.w() << " ;\n";

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

  EXPECT_TRUE(output.x.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.y.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.z.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.i.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.j.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.k.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.w.std_dev < 1.5 * stddev);

  //absolute value of location mean should still be very close to real
  std::cout << "angle values" << "\n";
  std::cout << abs(output.i.mean - q_ver.x()) << " , " << abs(output.j.mean - q_ver.y()) << " , " << abs(output.k.mean - q_ver.z()) << " , " << abs(output.w.mean - q_ver.w())<< ";\n";

  std::cout << "angle std_devs" << "\n";
  std::cout  << output.i.std_dev << ", "<< output.j.std_dev << ", " << output.k.std_dev << ", " << output.w.std_dev << ";\n";

  EXPECT_TRUE(abs(output.x.mean - camera_loc.translation()(0)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.y.mean - camera_loc.translation()(1)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.z.mean - camera_loc.translation()(2)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.w.mean - q_ver.w()) < 1.5 * stddev);
}

TEST(NoiseTest, 2DTwistNoiseTest)
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

  target_loc.rotate(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));

  camera_loc.translate(Eigen::Vector3d(0.0,0.0,1.0));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));

  Eigen::Quaterniond q_ver;
  Eigen::Matrix3d m = camera_loc.rotation();
  q_ver = m;

  std::cout << q_ver.x() << " , " <<q_ver.y() << " , " << q_ver.z() << " , " << q_ver.w() << " ;\n";

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

  EXPECT_TRUE(output.x.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.y.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.z.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.i.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.j.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.k.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.w.std_dev < 1.5 * stddev);

  //absolute value of location mean should still be very close to real
  std::cout << "angle values" << "\n";
  std::cout << abs(output.i.mean - q_ver.x()) << " , " << abs(output.j.mean - q_ver.y()) << " , " << abs(output.k.mean - q_ver.z()) << " , " << abs(output.w.mean - q_ver.w())<< ";\n";

  std::cout << "angle std_devs" << "\n";
  std::cout  << output.i.std_dev << ", "<< output.j.std_dev << ", " << output.k.std_dev << ", " << output.w.std_dev << ";\n";

  EXPECT_TRUE(abs(output.x.mean - camera_loc.translation()(0)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.y.mean - camera_loc.translation()(1)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.z.mean - camera_loc.translation()(2)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.i.mean - q_ver.x()) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.j.mean - q_ver.y()) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.k.mean - q_ver.z()) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.w.mean - q_ver.w()) < 1.5 * stddev);
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

  Eigen::Quaterniond q_ver;
  Eigen::Matrix3d m = camera_loc.rotation();
  q_ver = m;

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
   EXPECT_TRUE(output.i.std_dev < 1.0e-14);
   EXPECT_TRUE(output.j.std_dev < 1.0e-14);
   EXPECT_TRUE(output.k.std_dev < 1.0e-14);
   EXPECT_TRUE(output.w.std_dev < 1.0e-14);

   //absolute value of location mean should still be very close to real
   std::cout << "angle values" << "\n";
   std::cout << abs(output.i.mean - q_ver.x()) << " , " << abs(output.j.mean - q_ver.y()) << " , " << abs(output.k.mean - q_ver.z()) << " , " << abs(output.w.mean - q_ver.w())<< ";\n";

   std::cout << "angle std_devs" << "\n";
   std::cout  << output.i.std_dev << ", "<< output.j.std_dev << ", " << output.k.std_dev << ", " << output.w.std_dev << ";\n";

   EXPECT_TRUE(abs(output.x.mean - camera_loc.translation()(0)) < 1.0e-14);
   EXPECT_TRUE(abs(output.y.mean - camera_loc.translation()(1)) < 1.0e-14);
   EXPECT_TRUE(abs(output.z.mean - camera_loc.translation()(2)) < 1.0e-14);
   EXPECT_TRUE(abs(output.w.mean - q_ver.w()) < 1.0e-14);
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

  Eigen::Quaterniond q_ver;
  Eigen::Matrix3d m = camera_loc.rotation();
  q_ver = m;

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
  EXPECT_TRUE(output.i.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.j.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.k.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.w.std_dev < 1.5 * stddev);

  //absolute value of location mean should still be very close to real
  std::cout << "angle values" << "\n";
  std::cout << abs(output.i.mean - q_ver.x()) << " , " << abs(output.j.mean - q_ver.y()) << " , " << abs(output.k.mean - q_ver.z()) << " , " << abs(output.w.mean - q_ver.w())<< ";\n";

  std::cout << "angle std_devs" << "\n";
  std::cout  << output.i.std_dev << ", "<< output.j.std_dev << ", " << output.k.std_dev << ", " << output.w.std_dev << ";\n";

  EXPECT_TRUE(abs(output.x.mean - camera_loc.translation()(0)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.y.mean - camera_loc.translation()(1)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.z.mean - camera_loc.translation()(2)) < 1.5 * stddev);
//  EXPECT_TRUE(abs(output.i.mean - q_ver.x()) < 1.5 * stddev);
//  EXPECT_TRUE(abs(output.j.mean - q_ver.y()) < 1.5 * stddev);
//  EXPECT_TRUE(abs(output.k.mean - q_ver.z()) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.w.mean - q_ver.w()) < 1.5 * stddev);

}

TEST(NoiseTest, 3DTwistNoiseTest)
{
  //make target
  test::Target target(4, 4, 0.025);


  //reserve observations
  std::size_t obs_cnt = 35;
  std::vector<PnPProblem3D> perturbed_problem_set;
  perturbed_problem_set.reserve(obs_cnt);

  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d camera_loc = Eigen::Isometry3d::Identity();

  target_loc.rotate(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));

  camera_loc.translate(Eigen::Vector3d(0.0,0.0,1.0));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));

  Eigen::Quaterniond q_ver;
  Eigen::Matrix3d m = camera_loc.rotation();
  q_ver = m;

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
  EXPECT_TRUE(output.i.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.j.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.k.std_dev < 1.5 * stddev);
  EXPECT_TRUE(output.w.std_dev < 1.5 * stddev);

  //absolute value of location mean should still be very close to real
  std::cout << "angle values" << "\n";
  std::cout << abs(output.i.mean - q_ver.x()) << " , " << abs(output.j.mean - q_ver.y()) << " , " << abs(output.k.mean - q_ver.z()) << " , " << abs(output.w.mean - q_ver.w())<< ";\n";

  std::cout << "angle std_devs" << "\n";
  std::cout  << output.i.std_dev << ", "<< output.j.std_dev << ", " << output.k.std_dev << ", " << output.w.std_dev << ";\n";

  EXPECT_TRUE(abs(output.x.mean - camera_loc.translation()(0)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.y.mean - camera_loc.translation()(1)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.z.mean - camera_loc.translation()(2)) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.i.mean - q_ver.x()) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.j.mean - q_ver.y()) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.k.mean - q_ver.z()) < 1.5 * stddev);
  EXPECT_TRUE(abs(output.w.mean - q_ver.w()) < 1.5 * stddev);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
