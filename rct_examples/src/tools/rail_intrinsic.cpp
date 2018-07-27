#include <rct_image_tools/image_observation_finder.h>
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/experimental/rail_intrinsic.h>
#include "rct_ros_tools/data_set.h"
#include "rct_ros_tools/parameter_loaders.h"
#include "rct_ros_tools/print_utils.h"

#include <opencv2/highgui.hpp>
#include <ros/ros.h>

// This cost has a custom file loader & format
struct RailCalibrationScene
{
  std::vector<double> rail_poses;
  std::vector<rct_optimizations::CorrespondenceSet> correspondences;
};

struct RailCalibrationData
{
  std::vector<RailCalibrationScene> scenes;
};

RailCalibrationData parseCalData(const std::string& base_path, const std::vector<std::string>& images)
{
  return {};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rct_rail_intrinsic_ex", ros::init_options::AnonymousName);
  ros::NodeHandle pnh ("~");

  rct_image_tools::ModifiedCircleGridTarget target;
  if (!rct_ros_tools::loadTarget(pnh, "target", target))
  {
    return 1;
  }

  rct_optimizations::CameraIntrinsics intr;
  if (!rct_ros_tools::loadIntrinsics(pnh, "intrinsics", intr))
  {
    return 2;
  }

  // Load data sets
  std::vector<std::string> image_files;
  if (!pnh.getParam("images", image_files))
  {
    return 3;
  }

}
