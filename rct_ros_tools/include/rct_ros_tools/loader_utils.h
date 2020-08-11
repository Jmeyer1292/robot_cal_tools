#pragma once

#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <yaml-cpp/yaml.h>
#include <ros/console.h>

namespace rct_ros_tools
{

template<typename T>
static bool read(XmlRpc::XmlRpcValue& xml, const std::string& key, T& value)
{
  if (!xml.hasMember(key)) return false;
  try {
    value = static_cast<T>(xml[key]);
  } catch (const XmlRpc::XmlRpcException& ex) {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }
  return true;
}

}
