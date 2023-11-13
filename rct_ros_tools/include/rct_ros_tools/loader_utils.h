#pragma once

#include <yaml-cpp/yaml.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace rct_ros_tools
{
/**
 * @brief Converts an XmlRpcValue object into a YAML node
 * @param value
 * @return
 */
YAML::Node toYAML(XmlRpc::XmlRpcValue value);

}  // namespace rct_ros_tools
