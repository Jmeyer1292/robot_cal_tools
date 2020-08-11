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
   * \brief Load a target from a ROS parameter. Returns false if an error occurs.
   */
  static bool load(const ros::NodeHandle& nh, const std::string& key, TargetT& target);

  /**
   * \brief Load a target from a YAML file
   * \throws rct_ros_tools::BadFileException
   */
  static TargetT load(const std::string& path);

  /**
   * \brief Load a target from a YAML file. Returns false if an error occurs.
   */
  static bool load(const std::string& path, TargetT& target);
};

} // namespace rct_ros_tools
