#include <random>
#include <gtest/gtest.h>

#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>
#include <rct_optimizations/experimental/pnp.h>
#include <rct_optimizations/validation/noise_qualifier.h>
#include <rct_optimizations/ceres_math_utilities.h>

//tolerance for oreintation difference, in radians
#define ANGULAR_THRESHOLD 10*M_PI/180

using namespace rct_optimizations;

TEST(NoiseTest, QuatMeanTest)
{
  /*This test validate the method used to find the mean quaternion
   * in noise_qualifier.cpp
   */

  //base quaternion
  Eigen::Quaterniond quat_1 (0,1,0,0);
  Eigen::Quaterniond quat_2 (0,0,1,0);

  //std::cout << quat_2.angularDistance(quat_1) << "\n";
  std::vector<Eigen::Quaterniond> poses = {quat_1, quat_2};

  //average new quats
  RotationStat r = FindQuaternionMean(poses);

  Eigen::Quaterniond mean_quat1 (r.qw.mean, r.qx.mean, r.qy.mean, r.qz.mean);

  //The two quaternions are 2 pi rad apart, so the mean should be ~PI away from both
  EXPECT_LT(mean_quat1.angularDistance(quat_1) - M_PI, ANGULAR_THRESHOLD);
  EXPECT_LT(mean_quat1.angularDistance(quat_2) - M_PI, ANGULAR_THRESHOLD);

}

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
  q_ver = camera_loc.rotation();

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

  EXPECT_LT(output.x.std_dev, 1.0e-14);
  EXPECT_LT(output.y.std_dev, 1.0e-14);
  EXPECT_LT(output.z.std_dev, 1.0e-14);
  EXPECT_LT(output.q.qx.std_dev, 1.0e-14);
  EXPECT_LT(output.q.qy.std_dev, 1.0e-14);
  EXPECT_LT(output.q.qz.std_dev, 1.0e-14);
  EXPECT_LT(output.q.qw.std_dev, 1.0e-14);

  //Absolute value of quaternion is taken, since quaternions equal their oppoisite
  EXPECT_LT(abs(output.x.mean - camera_loc.translation()(0)), 1.0e-14);
  EXPECT_LT(abs(output.y.mean - camera_loc.translation()(1)), 1.0e-14);
  EXPECT_LT(abs(output.z.mean - camera_loc.translation()(2)), 1.0e-14);
  EXPECT_LT(abs(output.q.qx.mean) - abs(q_ver.x()), 1.0e-14);
  EXPECT_LT(abs(output.q.qy.mean) - abs(q_ver.y()), 1.0e-14);
  EXPECT_LT(abs(output.q.qz.mean) - abs(q_ver.z()), 1.0e-14);
  EXPECT_LT(abs(output.q.qw.mean) - abs(q_ver.w()), 1.0e-14);
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
  q_ver = camera_loc.rotation();

  //add noise boilerplate
  const double mean = 0.0;
  const double stddev = 1.0;
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

  //Project true target position into the camera
  double  uv[2];
  Eigen::Vector3d second_target_loc = camera_loc.translation();
  rct_optimizations::projectPoint(camera.intr, second_target_loc.data(), uv);

  EXPECT_LT(output.x.std_dev, 1.5 * stddev);
  EXPECT_LT(output.y.std_dev, 1.5 * stddev);
  EXPECT_LT(output.z.std_dev, 1.5 * stddev);

  //Absolute value of quaternion is taken, since quaternions equal their oppoisite
  EXPECT_LT(Eigen::Quaterniond(abs(output.q.qw.mean), abs(output.q.qx.mean), abs(output.q.qy.mean), abs(output.q.qz.mean)).angularDistance(q_ver), ANGULAR_THRESHOLD);

  EXPECT_LT(abs(output.x.mean - camera_loc.translation()(0)), (camera_loc.translation()(2)/camera.intr.fx()) * stddev);
  EXPECT_LT(abs(output.y.mean - camera_loc.translation()(1)), (camera_loc.translation()(2)/camera.intr.fy()) * stddev);
  //Max is taken in case the target location is is exactly correct & threshold goes to 0
  EXPECT_LT(abs(output.z.mean - camera_loc.translation()(2)), std::max(1.5*stddev, camera.intr.fx() * camera_loc.translation()(0) * std::sqrt(uv[0] - camera.intr.cx()) * stddev));
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

  camera_loc.translate(Eigen::Vector3d(0.0,0.0,1.0));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));

  Eigen::Quaterniond q_ver;
  q_ver = camera_loc.rotation();

  //add noise boilerplate
  const double mean = 0.0;
  const double stddev = 1.0;
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


 //find true target position in camera
 //target position relative to camera
 double  uv[2];
 Eigen::Vector3d second_target_loc = camera_loc.translation();
 rct_optimizations::projectPoint(camera.intr, second_target_loc.data(), uv);


 EXPECT_TRUE(output.x.std_dev < 1.5 * stddev);
 EXPECT_TRUE(output.y.std_dev < 1.5 * stddev);
 EXPECT_TRUE(output.z.std_dev < 1.5 * stddev);
 EXPECT_LT(Eigen::Quaterniond(output.q.qw.mean, output.q.qx.mean, output.q.qy.mean, output.q.qz.mean).angularDistance(q_ver), ANGULAR_THRESHOLD);

 //Absolute value of quaternion is taken, since quaternions equal their oppoisite
 EXPECT_LT(abs(output.x.mean - camera_loc.translation()(0)), (camera_loc.translation()(2)/camera.intr.fx()) * stddev);
 EXPECT_LT(abs(output.y.mean - camera_loc.translation()(1)), (camera_loc.translation()(2)/camera.intr.fy()) * stddev);
 EXPECT_LT(abs(output.z.mean - camera_loc.translation()(2)), std::max(1.5*stddev, camera.intr.fx() * camera_loc.translation()(0) * std::sqrt(uv[0] - camera.intr.cx()) * stddev));
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
  q_ver = camera_loc.rotation();

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

   EXPECT_LT(output.x.std_dev, 1.0e-14);
   EXPECT_LT(output.y.std_dev, 1.0e-14);
   EXPECT_LT(output.z.std_dev, 1.0e-14);
   //Rotational accuracy is less than positional, possibly because orientation is not optimized directly
   EXPECT_LT(output.q.qx.std_dev, ANGULAR_THRESHOLD);
   EXPECT_LT(output.q.qy.std_dev, ANGULAR_THRESHOLD);
   EXPECT_LT(output.q.qz.std_dev, ANGULAR_THRESHOLD);
   EXPECT_LT(output.q.qw.std_dev, ANGULAR_THRESHOLD);

   //Absolute value of quaternion is taken, since quaternions equal their oppoisite
   EXPECT_LT(abs(output.x.mean - camera_loc.translation()(0)), 1.0e-14);
   EXPECT_LT(abs(output.y.mean - camera_loc.translation()(1)), 1.0e-14);
   EXPECT_LT(abs(output.z.mean - camera_loc.translation()(2)), 1.0e-14);
   EXPECT_LT(abs(output.q.qx.mean) - abs(q_ver.x()), ANGULAR_THRESHOLD);
   EXPECT_LT(abs(output.q.qy.mean) - abs(q_ver.y()), ANGULAR_THRESHOLD);
   EXPECT_LT(abs(output.q.qz.mean) - abs(q_ver.z()), ANGULAR_THRESHOLD);
   EXPECT_LT(abs(output.q.qw.mean) - abs(q_ver.w()), ANGULAR_THRESHOLD );
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

  camera_loc.translate(Eigen::Vector3d(0.05,0.01,1.0));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));

  Eigen::Quaterniond q_ver;
  q_ver = camera_loc.rotation();

  //Noise boilerplate
  const double mean = 0.0;
  //Smaller std dev magnitude bc in meters, not pixels
  const double stddev = 0.005;
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

  EXPECT_LT(output.x.std_dev, 1.5 * stddev);
  EXPECT_LT(output.y.std_dev, 1.5 * stddev);
  EXPECT_LT(output.z.std_dev, 1.5 * stddev);
  double debug_dist = (Eigen::Quaterniond(output.q.qw.mean, output.q.qx.mean, output.q.qy.mean, output.q.qz.mean).angularDistance(q_ver));
  std::cout << debug_dist << "\n";
  EXPECT_LT(Eigen::Quaterniond(output.q.qw.mean, output.q.qx.mean, output.q.qy.mean, output.q.qz.mean).angularDistance(q_ver), ANGULAR_THRESHOLD);

  //Absolute value of quaternion is taken, since quaternions equal their oppoisite
  //Comparing to Standard Deviation, because 3d camera does not have explicit instrinsics
  EXPECT_LT(-1.0 * output.x.mean - camera_loc.translation()(0), 1.5 * stddev);
  EXPECT_LT(-1.0 * output.y.mean - camera_loc.translation()(1), 1.5 * stddev);
  EXPECT_LT(abs(output.z.mean - camera_loc.translation()(2)), 1.5 * stddev);
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

  camera_loc.translate(Eigen::Vector3d(0.01,0.01,1.0));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));
  camera_loc.rotate(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));

  Eigen::Quaterniond q_ver;
  q_ver = camera_loc.rotation();

  //Noise boilerplate
  const double mean = 0.0;
  //Smaller std dev magnitude bc in meters, not pixels
  const double stddev = 0.005;
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
  EXPECT_LT(Eigen::Quaterniond(output.q.qw.mean, output.q.qx.mean, output.q.qy.mean, output.q.qz.mean).angularDistance(q_ver), ANGULAR_THRESHOLD);

  std::cout << Eigen::Quaterniond(output.q.qw.mean, output.q.qx.mean, output.q.qy.mean, output.q.qz.mean).angularDistance(q_ver) << " \n";
  //Absolute value of quaternion is taken, since quaternions equal their oppoisite

  EXPECT_LT(abs(output.x.mean - camera_loc.translation()(0)), 1.5 * stddev);;
  EXPECT_LT(abs(output.y.mean - camera_loc.translation()(1)), 1.5 * stddev);
  EXPECT_LT(abs(output.z.mean - camera_loc.translation()(2)), 1.5 * stddev);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
