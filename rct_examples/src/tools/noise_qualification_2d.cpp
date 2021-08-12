#include <rct_optimizations/validation/noise_qualification.h>
#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_finder_plugin.h>
#include <rct_ros_tools/loader_utils.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;

std::string WINDOW = "window";

template<typename T>
T get(const ros::NodeHandle &nh, const std::string &key)
{
  T val;
  if (!nh.getParam(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "noise_qualification_2d");
  ros::NodeHandle pnh("~");

  try
  {
    // Parse parameters
    std::string data_path = get<std::string>(pnh, "data_path");
    std::string data_file = get<std::string>(pnh, "data_file");

    // Load the target finder
    auto target_finder_config = get<XmlRpc::XmlRpcValue>(pnh, "target_finder");
    const std::string target_finder_type = static_cast<std::string>(target_finder_config["type"]);
    pluginlib::ClassLoader<TargetFinderPlugin> loader("rct_ros_tools", "rct_ros_tools::TargetFinderPlugin");
    boost::shared_ptr<TargetFinderPlugin> target_finder = loader.createInstance(target_finder_type);
    target_finder->init(toYAML(target_finder_config));

    // Load camera intrinsics
    CameraIntrinsics camera = loadIntrinsics(pnh, "intrinsics");

    // Load an initial guess for the camera to target transformation
    Eigen::Isometry3d camera_to_target_guess = loadPose(pnh, "camera_to_target_guess");

    // Load the data file which specifies the location of the images on which to perform the noise qualification
    YAML::Node root = YAML::LoadFile(data_file);

    cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);

    // Set up the noise qualification inputs
    std::vector<PnPProblem> problem_set;
    problem_set.reserve(root.size());
    for (std::size_t i = 0; i < root.size(); ++i)
    {
      // Each entry should have an image path. This path is relative to the root_path directory!
      const auto img_path = root[i]["image"].as<std::string>();
      const std::string image_name = data_path + "/" + img_path;
      static cv::Mat image = readImageOpenCV(image_name);

      // Find the observations in the image
      rct_image_tools::TargetFeatures target_features;
      try
      {
        target_features = target_finder->findTargetFeatures(image);
        if (target_features.empty())
          throw std::runtime_error("Failed to find any target features");
        ROS_INFO_STREAM("Found " << target_features.size() << " target features");

        // Show the points we detected
        cv::imshow(WINDOW, target_finder->drawTargetFeatures(image, target_features));
        cv::waitKey();
      }
      catch (const std::runtime_error& ex)
      {
        ROS_WARN_STREAM("Image " << i << ": '" << ex.what() << "'");
        cv::imshow(WINDOW, image);
        cv::waitKey();
        continue;
      }

      // Set up the PnP problem for this image
      PnPProblem problem;
      problem.intr = camera;
      problem.camera_to_target_guess = camera_to_target_guess;

      // Add the detected correspondences
      problem.correspondences = target_finder->target().createCorrespondences(target_features);

      problem_set.push_back(problem);
    }

    // Perform the noise qualification
    PnPNoiseStat result = qualifyNoise2D(problem_set);

    // Print the results
    Eigen::IOFormat fmt(4, 0, ",", "\n", "[", "]");
    ROS_INFO_STREAM("Camera to Target Noise Results");
    ROS_INFO_STREAM("Position mean (m)\n" << result.p_stat.mean.transpose().format(fmt));
    ROS_INFO_STREAM("Position standard deviation (m)\n" << result.p_stat.stdev.transpose().format(fmt));
    ROS_INFO_STREAM("Quaternion mean (qx, qy, qz, qw)\n" << result.q_stat.mean.coeffs().transpose().format(fmt));
    ROS_INFO_STREAM("Quaternion standard deviation\n" << result.q_stat.stdev);
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}
