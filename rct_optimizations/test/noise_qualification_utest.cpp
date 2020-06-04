#include <rct_optimizations/noise_qualifier.h>
#include <gtest/gtest.h>
#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>

using namespace rct_optimizations;

TEST(NoiseTest, 2DFullTest)
{
  //make target
  test::Target target(5, 7, 0.025);

  //Load camera intrinsics
  //camera is a kinect
  auto camera = test::makeKinectCamera();

  //reserve observations
  std::size_t obs_cnt = 35;
  rct_optimizations::Observation2D3D::Set observations;
  observations.reserve(obs_cnt);

  //create poses outside the loop; would be better to decalre pose, then look at from there
  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  target_loc.translate(Eigen::Vector3d(1.0,0.0,0.0));
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();

  p.translation() = Eigen::Vector3d(0.05, 0, 0.1);
  p.linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  //create observations
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {

    //use camera pose for pose
    rct_optimizations::Observation2D3D ob;
    ob.to_target_mount = target_loc;
    ob.to_camera_mount = p;

    ob.correspondence_set = getCorrespondences(ob.to_camera_mount,
                                             ob.to_target_mount,
                                             camera,
                                             target,
                                             true);

    observations.push_back(ob);
  }

  //start with a perfect guess
  Eigen::Isometry3d guess = observations[0].to_camera_mount;

  //TODO: streamline initialization
  rct_optimizations::NoiseQualParams2D3D param;
  param.observations = observations;
  param.intr = camera.intr;
  param.camera_guess = guess;

  rct_optimizations::NoiseStatistics output = rct_optimizations::qualifyNoise2D(param);

  EXPECT_TRUE(output.std_dev(0) < 0.00001);
  EXPECT_TRUE(output.std_dev(1) < 0.00001);
  EXPECT_TRUE(output.std_dev(2) < 0.00001);
  EXPECT_TRUE(output.std_dev(3) < 0.00001);
  EXPECT_TRUE(output.std_dev(4) < 0.00001);
  EXPECT_TRUE(output.std_dev(5) < 0.00001);
}


//This test is a WIP
TEST(NoiseTest, 3DFullTest)
{
  //make target
  test::Target target(5, 7, 0.025);


  //reserve observations
  std::size_t obs_cnt = 35;
  rct_optimizations::Observation3D3D::Set observations;
  observations.reserve(obs_cnt);

  //create poses outside the loop; would be better to decalre pose, then look at from there
  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  target_loc.translate(Eigen::Vector3d(1.0,0.0,0.0));
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();

  p.translation() = Eigen::Vector3d(0.05, 0, 0.1);
  p.linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  //create observations
  for (std::size_t i = 0; i < obs_cnt; ++i)
  {

    //use camera pose for pose
    rct_optimizations::Observation3D3D ob;
    ob.to_target_mount = target_loc;
    ob.to_camera_mount = p;

    ob.correspondence_set = getCorrespondences(ob.to_camera_mount,
                                             ob.to_target_mount,
                                             target);

    observations.push_back(ob);
  }

  //start with a perfect guess
  Eigen::Isometry3d guess = observations[0].to_camera_mount;

  //TODO: streamline initialization
  rct_optimizations::NoiseQualParams3D3D param;
  param.observations = observations;
  param.camera_guess = guess;

  rct_optimizations::NoiseStatistics output = rct_optimizations::qualifyNoise3D(param);

  EXPECT_TRUE(output.std_dev(0) < 0.00001);
  EXPECT_TRUE(output.std_dev(1) < 0.00001);
  EXPECT_TRUE(output.std_dev(2) < 0.00001);
  EXPECT_TRUE(output.std_dev(3) < 0.00001);
  EXPECT_TRUE(output.std_dev(4) < 0.00001);
  EXPECT_TRUE(output.std_dev(5) < 0.00001);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
