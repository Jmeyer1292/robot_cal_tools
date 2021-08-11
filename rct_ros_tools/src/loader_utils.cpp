#include <rct_ros_tools/loader_utils.h>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace
{
XmlRpc::XmlRpcValue::Type getArrayType(const XmlRpc::XmlRpcValue& xml)
{
  if (xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::runtime_error("Input XmlRpcValue is not an array");

  // Get the type of the first element
  XmlRpc::XmlRpcValue::Type type = xml[0].getType();

  // Check that all subsequent elements are of the same type
  for (long i = 1; i < xml.size(); ++i)
  {
    if (xml[i].getType() != type)
    {
      std::stringstream ss;
      ss << "Type mismatch in array (element 0 = '" << type << "', element " << i << " = '" << xml[i].getType() << "')";
      throw std::runtime_error(ss.str());
    }
  }

  // Return the type
  return type;
}

template <typename T>
std::vector<T> fromArray(const XmlRpc::XmlRpcValue& xml)
{
  std::vector<T> vec;
  vec.reserve(xml.size());
  for (int i = 0; i < xml.size(); i++)
  {
    vec.push_back(static_cast<T>(xml[i]));
  }
  return vec;
}

} // namespace anonymous

namespace rct_ros_tools
{
YAML::Node toYAML(const XmlRpc::XmlRpcValue& value)
{
  YAML::Node node;

  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    throw std::runtime_error("Input XmlRpc object is not a struct");

  // Loop over all elements in the XmlRpcValue
  for (auto it = value.begin(); it != value.end(); ++it)
  {
    switch (it->second.getType())
    {
      case XmlRpc::XmlRpcValue::TypeStruct:
        node[it->first] = toYAML(it->second);
        break;
      case XmlRpc::XmlRpcValue::TypeArray:
        switch (getArrayType(it->second))
        {
          case XmlRpc::XmlRpcValue::TypeBoolean:
            node[it->first] = fromArray<bool>(it->second);
            break;
          case XmlRpc::XmlRpcValue::TypeInt:
            node[it->first] = fromArray<int>(it->second);
            break;
          case XmlRpc::XmlRpcValue::TypeDouble:
            node[it->first] = fromArray<double>(it->second);
            break;
          case XmlRpc::XmlRpcValue::TypeString:
            node[it->first] = fromArray<std::string>(it->second);
            break;
          default:
            throw std::runtime_error("Unsupported array type (" + std::to_string(it->second.getType()) + ")");
        }
        break;
      case XmlRpc::XmlRpcValue::TypeBoolean:
        node[it->first] = static_cast<bool>(it->second);
        break;
      case XmlRpc::XmlRpcValue::TypeInt:
        node[it->first] = static_cast<int>(it->second);
        break;
      case XmlRpc::XmlRpcValue::TypeDouble:
        node[it->first] = static_cast<double>(it->second);
        break;
      case XmlRpc::XmlRpcValue::TypeString:
        node[it->first] = static_cast<std::string>(it->second);
        break;
      default:
        throw std::runtime_error("Unsupported type '" + std::to_string(it->second.getType()) + "'");
    }
  }
  return node;
}

} // namespace rct_ros_tools
