#include <rct_ros_tools/target_finder_plugin.h>
#include <rct_ros_tools/exceptions.h>
#include <rct_image_tools/charuco_finder.h>
#include <rct_image_tools/modified_circle_grid_finder.h>

#include <xmlrpcpp/XmlRpcException.h>
#include <yaml-cpp/yaml.h>

namespace rct_ros_tools
{
class ModifiedCircleGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  using TargetFinderPlugin::TargetFinderPlugin;

  void init(const XmlRpc::XmlRpcValue& config) override
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

  void init(const std::string& file) override
  {
    YAML::Node n = YAML::LoadFile(file);
    int rows = n["target_finder"]["rows"].as<int>();
    int cols = n["target_finder"]["cols"].as<int>();
    double spacing = n["target_finder"]["spacing"].as<double>();  // (meters between dot centers)
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
};

class CharucoGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  using TargetFinderPlugin::TargetFinderPlugin;

  void init(const XmlRpc::XmlRpcValue& config) override
  {
    int rows = static_cast<int>(config["rows"]);
    int cols = static_cast<int>(config["cols"]);
    double chessboard_dim = static_cast<double>(config["chessboard_dim"]);
    double aruco_marker_dim = static_cast<double>(config["aruco_marker_dim"]);
    int dictionary = static_cast<int>(config["dictionary"]);

    rct_image_tools::CharucoGridTarget target(rows, cols, static_cast<float>(chessboard_dim),
                                              static_cast<float>(aruco_marker_dim), dictionary);

    finder_ = std::make_shared<const rct_image_tools::CharucoGridBoardTargetFinder>(target);
  }

  void init(const std::string& file) override
  {
    YAML::Node n = YAML::LoadFile(file);
    int cols = n["target_finder"]["cols"].as<int>();
    int rows = n["target_finder"]["rows"].as<int>();
    float chessboard_dim = n["target_finder"]["chessboard_dim"].as<float>();
    float aruco_marker_dim = n["target_finder"]["aruco_marker_dim"].as<float>();
    int dictionary = n["target_finder"]["dictionary"].as<int>();

    rct_image_tools::CharucoGridTarget target(rows, cols, chessboard_dim, aruco_marker_dim, dictionary);
    finder_ = std::make_shared<const rct_image_tools::CharucoGridBoardTargetFinder>(target);
  }
};

}  // namespace rct_ros_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rct_ros_tools::CharucoGridTargetFinderPlugin, rct_ros_tools::TargetFinderPlugin)
PLUGINLIB_EXPORT_CLASS(rct_ros_tools::ModifiedCircleGridTargetFinderPlugin, rct_ros_tools::TargetFinderPlugin)
