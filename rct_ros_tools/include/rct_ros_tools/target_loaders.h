#pragma once

#include <ros/node_handle.h>
#include <string>

namespace rct_ros_tools
{
template <typename TargetT>
struct TargetLoader
{
  /**
   * \brief Load a target from a ROS parameter
   * \throws ros::InvalidParameterException
   */
  static TargetT load(const ros::NodeHandle& nh, const std::string& key);

  /**
   * \brief Load a target from a YAML file
   * \throws rct_ros_tools::BadFileException
   */
  static TargetT load(const std::string& path);
};

} // namespace rct_ros_tools
