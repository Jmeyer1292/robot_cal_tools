#include <rct_ros_tools/exceptions.h>
#include <rct_ros_tools/target_loaders.h>
#include <rct_image_tools/charuco_finder.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <yaml-cpp/yaml.h>

const int DEFAULT_ARUCO_DICTIONARY = cv::aruco::DICT_6X6_250;

namespace rct_ros_tools
{
void CharucoGridTargetFinderPlugin::init(const XmlRpc::XmlRpcValue& config)
{
  int rows = static_cast<int>(config["rows"]);
  int cols = static_cast<int>(config["cols"]);
  double chessboard_dim = static_cast<double>(config["chessboard_dim"]);
  double aruco_marker_dim = static_cast<double>(config["aruco_marker_dim"]);
  int dictionary;
  try
  {
    dictionary = static_cast<int>(config["dictionary"]);
  }
  catch (const XmlRpc::XmlRpcException&)
  {
    dictionary = DEFAULT_ARUCO_DICTIONARY;
    ROS_WARN_STREAM("Using default ArUco dictionary ID: " << DEFAULT_ARUCO_DICTIONARY);
  }

  rct_image_tools::CharucoGridTarget target(rows, cols, static_cast<float>(chessboard_dim),
                                            static_cast<float>(aruco_marker_dim), dictionary);

  finder_ = std::make_shared<const rct_image_tools::CharucoGridBoardTargetFinder>(target);
}

void CharucoGridTargetFinderPlugin::init(const std::string& file)
{
  YAML::Node n = YAML::LoadFile(file);
  int cols = n["target_definition"]["cols"].as<int>();
  int rows = n["target_definition"]["rows"].as<int>();
  float chessboard_dim = n["target_definition"]["chessboard_dim"].as<float>();
  float aruco_marker_dim = n["target_definition"]["aruco_marker_dim"].as<float>();

  int dictionary;
  try
  {
    dictionary = n["target_definition"]["dictionary"].as<int>();
  }
  catch (const YAML::Exception&)
  {
    dictionary = DEFAULT_ARUCO_DICTIONARY;
    ROS_WARN_STREAM("Using default ArUco dictionary ID: " << DEFAULT_ARUCO_DICTIONARY);
  }

  rct_image_tools::CharucoGridTarget target(rows, cols, chessboard_dim, aruco_marker_dim, dictionary);
  finder_ = std::make_shared<const rct_image_tools::CharucoGridBoardTargetFinder>(target);
}

}  // namespace rct_ros_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rct_ros_tools::CharucoGridTargetFinderPlugin, rct_ros_tools::TargetFinderPlugin);
