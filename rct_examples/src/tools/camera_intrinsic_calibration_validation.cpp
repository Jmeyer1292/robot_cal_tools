#include <rct_optimizations/validation/camera_intrinsic_calibration_validation.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_loaders.h>
#include <rct_ros_tools/print_utils.h>
#include <rct_ros_tools/data_set.h>

// To find 2D  observations from images
#include <rct_image_tools/modified_circle_grid_finder.h>
#include <rct_image_tools/image_utils.h>

// For display of found targets
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;

void analyzeResults(const IntrinsicCalibrationAccuracyResult &result,
                    const double pos_tol, const double ang_tol)
{
  ROS_INFO_STREAM("Positional Error:\nMean (m): " << result.pos_error.first << "\nStd. Dev. (m): " << result.pos_error.second);
  ROS_INFO_STREAM("Angular Error:\nMean (rad): " << result.ang_error.first << "\nStd. Dev. (rad): " << result.ang_error.second);

  // Calculate the error such that it represents ~95% of the results (mean + 2 * sigma)
  double pos_error = result.pos_error.first + 2.0 * result.pos_error.second;
  double ang_error = result.ang_error.first + 2.0 * result.ang_error.second;
  if (pos_error > pos_tol || ang_error > ang_tol)
  {
    ROS_WARN_STREAM("Camera intrinsic calibration is not within tolerance\nPosition Error (m): "
                    << pos_error << " (" << pos_tol << " allowed)"
                    << "\nAngular Error (rad): " << ang_error << " (" << ang_tol << " allowed)");
  }
  else
  {
    ROS_INFO_STREAM("Camera intrinsic calibration is valid!");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_on_wrist_extrinsic");
  ros::NodeHandle pnh("~");

  // Load the data set path from ROS param
  std::string data_path;
  if (!pnh.getParam("data_path", data_path))
  {
    ROS_ERROR("Must set 'data_path' parameter");
    return 1;
  }

  try
  {
    auto target = TargetLoader<ModifiedCircleGridTarget>::load(pnh, "target_definition");
    CameraIntrinsics intr = loadIntrinsics(pnh, "intrinsics");

    // Attempt to load the data set via the data record yaml file:
    boost::optional<ExtrinsicDataSet> maybe_data_set = parseFromFile(
      data_path);
    if (!maybe_data_set)
    {
      ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path);
      return 2;
    }

    // We know it exists, so define a helpful alias
    const ExtrinsicDataSet& data_set = *maybe_data_set;

    // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
    Eigen::Isometry3d target_mount_to_target = loadPose(pnh, "base_to_target_guess");
    Eigen::Isometry3d camera_mount_to_camera = loadPose(pnh, "wrist_to_camera_guess");

    // Lets create a class that will search for the target in our raw images.
    ModifiedCircleGridTargetFinder target_finder(target);

    // Finally, we need to process our images into correspondence sets: for each dot in the
    // target this will be where that dot is in the target and where it was seen in the image.
    // Repeat for each image. We also tell where the wrist was when the image was taken.
    Observation2D3D::Set observations;
    observations.reserve(data_set.images.size());
    for (std::size_t i = 0; i < data_set.images.size(); ++i)
    {
      // Try to find the circle grid in this image:
      rct_image_tools::TargetFeatures target_features;
      try
      {
        target_features = target_finder.findTargetFeatures(data_set.images[i]);

        // Show the points we detected
        cv::imshow("points", target_finder.drawTargetFeatures(data_set.images[i], target_features));
        cv::waitKey();
      }
      catch(const std::runtime_error& ex)
      {
        ROS_WARN_STREAM("Unable to find the circle grid in image " << i << ": '" << ex.what() << "'");
        cv::imshow("points", data_set.images[i]);
        cv::waitKey();
        continue;
      }

      Observation2D3D obs;
      obs.correspondence_set.reserve(target_features.size());

      // So for each image we need to:
      //// 1. Record the wrist position
      obs.to_camera_mount = data_set.tool_poses[i];
      obs.to_target_mount = Eigen::Isometry3d::Identity();

      //// And finally add that to the problem
      obs.correspondence_set = target.createCorrespondences(target_features);

      observations.push_back(obs);
    }

    // Measure the intrinsic calibration accuracy
    // The assumption here is that all PnP optimizations should have a residual error less than 1.0 pixels
    IntrinsicCalibrationAccuracyResult result = measureIntrinsicCalibrationAccuracy(
        observations, intr, camera_mount_to_camera, target_mount_to_target, Eigen::Isometry3d::Identity(), 1.0);

    // Analyze the results
    // These error tolerances allow the virtual correspondence sets for each observation to deviate from the expectation by up to 1 mm and 0.05 degrees
    // The chosen tolerances are relatively arbitrary, but should be very small
    double pos_tol = 1.0e-3;
    double ang_tol = 0.05 * M_PI / 180.0;
    analyzeResults(result, pos_tol, ang_tol);
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}
