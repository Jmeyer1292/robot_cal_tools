#include <rct_optimizations/validation/camera_intrinsic_calibration_validation.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/print_utils.h>
#include <rct_ros_tools/data_set.h>

// To find 2D  observations from images
#include <rct_image_tools/image_observation_finder.h>
#include <rct_image_tools/image_utils.h>

// For display of found targets
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

void analyzeResults(const rct_optimizations::IntrinsicCalibrationAccuracyResult &result,
                    const double pos_tol, const double ang_tol)
{
  ROS_INFO_STREAM("Positional Error:\nMean (m): " << result.pos_error.first << "\nStd. Dev. (m): " << result.pos_error.second);
  ROS_INFO_STREAM("Angular Error:\nMean (m): " << result.ang_error.first << "\nStd. Dev. (m): " << result.ang_error.second);

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
    rct_image_tools::ModifiedCircleGridTarget target = rct_ros_tools::loadTarget(pnh, "target_definition");
    rct_optimizations::CameraIntrinsics intr = rct_ros_tools::loadIntrinsics(pnh, "intrinsics");

    // Attempt to load the data set via the data record yaml file:
    boost::optional<rct_ros_tools::ExtrinsicDataSet> maybe_data_set = rct_ros_tools::parseFromFile(
      data_path);
    if (!maybe_data_set)
    {
      ROS_ERROR_STREAM("Failed to parse data set from path = " << data_path);
      return 2;
    }

    // We know it exists, so define a helpful alias
    const rct_ros_tools::ExtrinsicDataSet& data_set = *maybe_data_set;

    // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
    Eigen::Isometry3d target_mount_to_target = rct_ros_tools::loadPose(pnh, "base_to_target_guess");
    Eigen::Isometry3d camera_mount_to_camera = rct_ros_tools::loadPose(pnh, "wrist_to_camera_guess");

    // Lets create a class that will search for the target in our raw images.
    rct_image_tools::ModifiedCircleGridObservationFinder obs_finder(target);

    // Finally, we need to process our images into correspondence sets: for each dot in the
    // target this will be where that dot is in the target and where it was seen in the image.
    // Repeat for each image. We also tell where the wrist was when the image was taken.
    rct_optimizations::Observation2D3D::Set observations;
    observations.reserve(data_set.images.size());
    for (std::size_t i = 0; i < data_set.images.size(); ++i)
    {
      // Try to find the circle grid in this image:
      auto maybe_obs = obs_finder.findObservations(data_set.images[i]);
      if (!maybe_obs)
      {
        ROS_WARN_STREAM("Unable to find the circle grid in image: " << i);
        cv::imshow("points", data_set.images[i]);
        cv::waitKey();
        continue;
      }
      else
      {
        // Show the points we detected
        cv::imshow("points", obs_finder.drawObservations(data_set.images[i], *maybe_obs));
        cv::waitKey();
      }

      rct_optimizations::Observation2D3D obs;
      obs.correspondence_set.reserve(maybe_obs->size());

      // So for each image we need to:
      //// 1. Record the wrist position
      obs.to_camera_mount = data_set.tool_poses[i];
      obs.to_target_mount = Eigen::Isometry3d::Identity();

      //// And finally add that to the problem
      obs.correspondence_set = rct_image_tools::getCorrespondenceSet(*maybe_obs, target.points);

      observations.push_back(obs);
    }

    // Measure the intrinsic calibration accuracy
    // The assumption here is that all PnP optimizations should have a residual error less than 1.0 pixels
    rct_optimizations::IntrinsicCalibrationAccuracyResult result
      = rct_optimizations::measureIntrinsicCalibrationAccuracy(observations,
                                                               intr,
                                                               camera_mount_to_camera,
                                                               target_mount_to_target,
                                                               Eigen::Isometry3d::Identity(),
                                                               1.0);

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
