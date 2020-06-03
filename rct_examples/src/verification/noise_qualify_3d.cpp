#include <Eigen/Dense>
#include <rct_optimizations/noise_qualifier.h>
#include <rct_optimizations/types.h>
#include <rct_image_tools/image_observation_finder.h>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <opencv/cv.hpp>

#include "rct_ros_tools/parameter_loaders.h"
#include "rct_ros_tools/data_set.h"

//3d version: no camera intrinsics

int main(int argc, char** argv)
{
  // Parse parameters
  std::string path(argv[1]);
  int rows = std::stoi(argv[2]);
  int cols = std::stoi(argv[3]);

  //make target
  rct_image_tools::ModifiedCircleGridTarget target (rows, cols, 0.01);
  rct_image_tools::ModifiedCircleGridObservationFinder obs_finder(target);


  YAML::Node root = YAML::LoadFile(path + std::string("/data.yaml"));
  //const std::string root_path = rootPath(path);

  //reserve observations
  rct_optimizations::Observation3D3D::Set observations;
  observations.reserve(root.size());

  //create observations
  for (std::size_t i = 0; i < root.size(); ++i)
  {
    // Each entry should have a pose and image path. This path is relative to the root_path directory!
    const auto img_path = root[i]["image"].as<std::string>();
    const auto pose_path = root[i]["pose"].as<std::string>();

    const std::string image_name = path + img_path;
    //need to load a point cloud here
    //static cv::Mat image = rct_ros_tools::readImageOpenCV(image_name);

    Eigen::Isometry3d p = rct_ros_tools::loadPose(path + pose_path);

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
  const rct_optimizations::NoiseQualParams2D3D param{observations, camera, guess};

  rct_optimizations::NoiseStatistics output = rct_optimizations::qualifyNoise2D(param);
}
