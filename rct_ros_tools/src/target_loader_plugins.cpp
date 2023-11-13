#include <rct_ros_tools/target_finder_plugin.h>
#include <rct_ros_tools/exceptions.h>
#include <rct_image_tools/charuco_finder.h>
#include <rct_image_tools/modified_circle_grid_finder.h>

#include <boost/core/demangle.hpp>
#include <ros/console.h>
#include <yaml-cpp/yaml.h>

namespace
{
template <typename T>
T getMember(const YAML::Node& n, const std::string& key)
{
  try
  {
    T value = n[key].as<T>();
    return value;
  }
  catch (const YAML::BadConversion&)
  {
    throw std::runtime_error("Failed to convert parameter '" + key + "' to type '" +
                             boost::core::demangle(typeid(T).name()) + "'");
  }
  catch (const YAML::Exception&)
  {
    throw std::runtime_error("Failed to find '" + key + "' parameter of type '" +
                             boost::core::demangle(typeid(T).name()) + "'");
  }
}

/**
 * @brief Load CircleDetector parameters from a yaml node.
 * @param node
 * @throw std::runtime_error if a parameter in the file exists and fails to be parsed.  Putting a string instead of a
 * float, etc.
 */
rct_image_tools::CircleDetectorParams loadCircleDetectorParams(const YAML::Node& node)
{
  rct_image_tools::CircleDetectorParams p;

  p.nThresholds = getMember<decltype(p.nThresholds)>(node, "nThresholds");
  p.minThreshold = getMember<decltype(p.minThreshold)>(node, "minThreshold");
  p.maxThreshold = getMember<decltype(p.maxThreshold)>(node, "maxThreshold");
  p.minRepeatability = getMember<decltype(p.minRepeatability)>(node, "minRepeatability");
  p.circleInclusionRadius = getMember<decltype(p.circleInclusionRadius)>(node, "circleInclusionRadius");
  p.maxRadiusDiff = getMember<decltype(p.maxRadiusDiff)>(node, "maxRadiusDiff");
  p.maxAverageEllipseError = getMember<decltype(p.maxAverageEllipseError)>(node, "maxAverageEllipseError");

  p.filterByColor = getMember<decltype(p.filterByColor)>(node, "filterByColor");
  p.circleColor = static_cast<unsigned short>(getMember<int>(node, "circleColor"));

  p.filterByArea = getMember<decltype(p.filterByArea)>(node, "filterByArea");
  p.minArea = getMember<decltype(p.minArea)>(node, "minArea");
  p.maxArea = getMember<decltype(p.maxArea)>(node, "maxArea");

  p.filterByCircularity = getMember<decltype(p.filterByCircularity)>(node, "filterByCircularity");
  p.minCircularity = getMember<decltype(p.minCircularity)>(node, "minCircularity");
  p.maxCircularity = getMember<decltype(p.maxCircularity)>(node, "maxCircularity");

  p.filterByInertia = getMember<decltype(p.filterByInertia)>(node, "filterByInertia");
  p.minInertiaRatio = getMember<decltype(p.minInertiaRatio)>(node, "minInertiaRatio");
  p.maxInertiaRatio = getMember<decltype(p.maxInertiaRatio)>(node, "maxInertiaRatio");

  p.filterByConvexity = getMember<decltype(p.filterByConvexity)>(node, "filterByConvexity");
  p.minConvexity = getMember<decltype(p.minConvexity)>(node, "minConvexity");
  p.maxConvexity = getMember<decltype(p.maxConvexity)>(node, "maxConvexity");

  return p;
}

}  // namespace

namespace rct_ros_tools
{
class ModifiedCircleGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  using TargetFinderPlugin::TargetFinderPlugin;

  void init(const YAML::Node& config) override
  {
    int rows = config["rows"].as<int>();
    int cols = config["cols"].as<int>();
    double spacing = config["spacing"].as<double>();
    rct_image_tools::ModifiedCircleGridTarget target(rows, cols, spacing);

    try
    {
      rct_image_tools::CircleDetectorParams circle_detector_params = loadCircleDetectorParams(config["circle_detector_"
                                                                                                     "params"]);
      ROS_INFO_STREAM("Successfully loaded circle detector parameters");

      finder_ = std::make_shared<const rct_image_tools::ModifiedCircleGridTargetFinder>(target, circle_detector_params);
    }
    catch (const std::exception& ex)
    {
      ROS_WARN_STREAM("Failed to load circle detector parameters: '" << ex.what() << "'. Using default values");
      finder_ = std::make_shared<const rct_image_tools::ModifiedCircleGridTargetFinder>(target);
    }
  }
};

class CharucoGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  using TargetFinderPlugin::TargetFinderPlugin;

  void init(const YAML::Node& config) override
  {
    int cols = config["cols"].as<int>();
    int rows = config["rows"].as<int>();
    float chessboard_dim = config["chessboard_dim"].as<float>();
    float aruco_marker_dim = config["aruco_marker_dim"].as<float>();
    int dictionary = config["dictionary"].as<int>();

    rct_image_tools::CharucoGridTarget target(rows, cols, chessboard_dim, aruco_marker_dim, dictionary);
    finder_ = std::make_shared<const rct_image_tools::CharucoGridBoardTargetFinder>(target);
  }
};

}  // namespace rct_ros_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rct_ros_tools::CharucoGridTargetFinderPlugin, rct_ros_tools::TargetFinderPlugin)
PLUGINLIB_EXPORT_CLASS(rct_ros_tools::ModifiedCircleGridTargetFinderPlugin, rct_ros_tools::TargetFinderPlugin)
