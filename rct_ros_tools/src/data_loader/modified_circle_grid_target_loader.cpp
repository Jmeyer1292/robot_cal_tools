#include <rct_ros_tools/exceptions.h>
#include <rct_ros_tools/loader_utils.h>
#include <rct_ros_tools/target_loaders.h>
#include <rct_image_tools/modified_circle_grid_target.h>

namespace rct_ros_tools
{
template<>
rct_image_tools::ModifiedCircleGridTarget TargetLoader<rct_image_tools::ModifiedCircleGridTarget>::load(const ros::NodeHandle& nh, const std::string& key)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(key, xml)) throw ros::InvalidParameterException(key);

  int rows = 0;
  int cols = 0;
  double spacing = 0.0;

  if (!read(xml, "rows", rows)) throw ros::InvalidParameterException(key + "/rows");
  if (!read(xml, "cols", cols)) throw ros::InvalidParameterException(key + "/cols");
  if (!read(xml, "spacing", spacing)) throw ros::InvalidParameterException(key + "/spacing");

  return rct_image_tools::ModifiedCircleGridTarget(rows, cols, spacing);
}

template<>
rct_image_tools::ModifiedCircleGridTarget TargetLoader<rct_image_tools::ModifiedCircleGridTarget>::load(const std::string &path)
{
  try
  {
    YAML::Node n = YAML::LoadFile(path);
    int rows = n["target_definition"]["rows"].as<int>();
    int cols = n["target_definition"]["cols"].as<int>();
    double spacing = n["target_definition"]["spacing"].as<double>(); // (meters between dot centers)
    return rct_image_tools::ModifiedCircleGridTarget(rows, cols, spacing);
  }
  catch (YAML::Exception &ex)
  {
    throw BadFileException(std::string("YAML failure: ") + ex.what());
  }
}

} // namespace rct_ros_tools
