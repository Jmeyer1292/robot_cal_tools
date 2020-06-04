#include <Eigen/Dense>
#include <rct_optimizations/noise_qualifier.h>
#include <rct_optimizations/types.h>
#include <rct_image_tools/image_observation_finder.h>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <opencv/cv.hpp>

#include "rct_ros_tools/parameter_loaders.h"
#include "rct_ros_tools/data_set.h"

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

  //Load camera intrinsics
  rct_optimizations::CameraIntrinsics camera = rct_ros_tools::loadIntrinsics(path +  std::string("/camera_data.yaml"));

  //reserve observations
  rct_optimizations::Observation2D3D::Set observations;
  observations.reserve(root.size());

  //create observations
  for (std::size_t i = 0; i < root.size(); ++i)
  {
    // Each entry should have a pose and image path. This path is relative to the root_path directory!
    const auto img_path = root[i]["image"].as<std::string>();
    const auto pose_path = root[i]["pose"].as<std::string>();

    const std::string image_name = path + img_path;
    static cv::Mat image = rct_ros_tools::readImageOpenCV(image_name);

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

    rct_optimizations::Correspondence2D3D::Set correspondences;

    // And loop over each detected dot:
    for (std::size_t j = 0; j < maybe_obs->size(); ++j)
    {
      rct_optimizations::Correspondence2D3D pair;
      pair.in_image = maybe_obs->at(j); // The obs finder and target define their points in the same order!
      pair.in_target = target.points[j];
      correspondences.push_back(pair);
    }
    ob.correspondence_set = correspondences;
    observations.push_back(ob);
  }

  //then feed observations into qualify
  //start with a good guess
  Eigen::Isometry3d guess = observations[0].to_camera_mount;
  const rct_optimizations::NoiseQualParams2D3D param{observations, camera, guess};

  rct_optimizations::NoiseStatistics output = rct_optimizations::qualifyNoise2D(param);

  //TODO: Find proper method for reporting output
  std::cout << "The standard deviation is : x: " << output.std_dev(0) <<
                                          "\n y: " << output.std_dev(1) <<
                                          "\n z: " << output.std_dev(2) <<
                                       "\n roll: " << output.std_dev(3) <<
                                      "\n pitch: " << output.std_dev(4) <<
                                        "\n yaw: " << output.std_dev(5) <<
                                                              std::endl;
}
