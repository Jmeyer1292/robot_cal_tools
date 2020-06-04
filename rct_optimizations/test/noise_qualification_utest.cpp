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

//  p.translate(Eigen::Vector3d(1.0,1.0,1.0));
//  p = test::lookAt( p.translation(), target_loc.translation(), Eigen::Vector3d(1,0,0));
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

   // ob.correspondence_set = correspondences;
    observations.push_back(ob);
  }

  //start with a good guess, but may be too good
  Eigen::Isometry3d guess = observations[0].to_camera_mount;

  //TODO: streamline initialization
  rct_optimizations::NoiseQualParams2D3D param;
  param.observations = observations;
  param.intr = camera.intr;
  param.camera_guess = guess;

  rct_optimizations::NoiseStatistics output = rct_optimizations::qualifyNoise2D(param);

  EXPECT_TRUE(output.std_dev(0) < 10);
}

/*
 * This test is a WIP
TEST(NoiseTest, 3DFullTest)
{
  // create observations, correspondences using obs_creator

  std::size_t ob_cnt= 10;
  //reserve observations
  rct_optimizations::Observation3D3D::Set observations;
  observations.reserve(ob_cnt);

  //create observations
  for (std::size_t i = 0; i < ob_cnt; ++i)
  {

    const std::string image_name = path + img_path;
    Eigen::Isometry3d p = rct_ros_tools::loadPose(path + pose_path);

     //replace this with obs_creator
     auto maybe_obs = obs_finder.findObservations(image);
     if (!maybe_obs)
     {
       cv::imshow("points", image);
       cv::waitKey();
       continue;
     }


    //target loc = Identity
    //use camera pose for pose
    rct_optimizations::Observation2D3D ob;
    ob.to_target_mount = Eigen::Isometry3d::Identity();
    ob.to_camera_mount = p;

    rct_optimizations::CorrespondenceSet correspondences;

    // And loop over each detected dot:
    for (std::size_t j = 0; j < maybe_obs->size(); ++j)
    {
      // The 'target.points' data structure and the observation vector returned by
      // 'obs_finder.findObservations()' are in the same order! So the j-th index in each
      // respresents the same dot!
      rct_optimizations::Correspondence2D3D pair;
      pair.in_image = maybe_obs->at(j); // The obs finder and target define their points in the same order!
      pair.in_target = target.points[j];
      correspondences.push_back(pair);
    }
    ob.correspondence_set = correspondences;
    observations.push_back(ob);
  }

  //then feed observations into qualify
  //start with a good guess, but may be too good
  Eigen::Isometry3d guess = observations[0].to_camera_mount;
  const rct_optimizations::NoiseQualParams2D3D param{observations, guess};

  rct_optimizations::NoiseStatistics output = rct_optimizations::qualifyNoise3D(param);
}
*/


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
