#include <rct_ros_tools/exceptions.h>
#include <rct_ros_tools/target_loaders.h>
#include <rct_image_tools/modified_circle_grid_finder.h>
#include <yaml-cpp/yaml.h>

namespace rct_ros_tools
{
void ModifiedCircleGridTargetFinderPlugin::init(const XmlRpc::XmlRpcValue& config)
{
  int rows = static_cast<int>(config["rows"]);
  int cols = static_cast<int>(config["cols"]);
  double spacing = static_cast<double>(config["spacing"]);
  rct_image_tools::ModifiedCircleGridTarget target(rows, cols, spacing);

  try
  {
    // Load the circle detector parameters from file
    std::string circle_params_file = static_cast<std::string>(config["circle_params_file"]);

    rct_image_tools::CircleDetectorParams circle_detector_params;
    if (!rct_image_tools::CircleDetector::loadParams(circle_params_file, circle_detector_params))
      throw std::runtime_error("Failed to load circle detector parameters from '" + circle_params_file + "'");

    finder_ = std::make_shared<const rct_image_tools::ModifiedCircleGridTargetFinder>(target, circle_detector_params);
  }
  catch (const std::exception& ex)
  {
    ROS_WARN_STREAM("Failed to get circle finder parameters file: '" << ex.what() << "'. Using default values");
    finder_ = std::make_shared<const rct_image_tools::ModifiedCircleGridTargetFinder>(target);
  }
}

void ModifiedCircleGridTargetFinderPlugin::init(const std::string& file)
{
  YAML::Node n = YAML::LoadFile(file);
  int rows = n["target_definition"]["rows"].as<int>();
  int cols = n["target_definition"]["cols"].as<int>();
  double spacing = n["target_definition"]["spacing"].as<double>();  // (meters between dot centers)
  rct_image_tools::ModifiedCircleGridTarget target(rows, cols, spacing);

  try
  {
    rct_image_tools::CircleDetectorParams circle_detector_params;
    if (!rct_image_tools::CircleDetector::loadParams(file, circle_detector_params))
      throw std::runtime_error("Failed to load circle detector parameters from '" + file + "'");

    finder_ = std::make_shared<const rct_image_tools::ModifiedCircleGridTargetFinder>(target, circle_detector_params);
  }
  catch(const std::exception& ex)
  {
    ROS_WARN_STREAM("Failed to get circle finder parameters file: '" << ex.what() << "'. Using default values");
    finder_ = std::make_shared<const rct_image_tools::ModifiedCircleGridTargetFinder>(target);
  }
}

} // namespace rct_ros_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rct_ros_tools::ModifiedCircleGridTargetFinderPlugin, rct_ros_tools::TargetFinderPlugin);
