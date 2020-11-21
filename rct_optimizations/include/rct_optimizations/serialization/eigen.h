#pragma once

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace YAML
{
template<typename V>
struct convert<Eigen::Matrix<V, 2, 1>>
{
  using T = Eigen::Matrix<V, 2, 1>;

  static Node encode(const T &val)
  {
    YAML::Node node;
    node["x"] = val.x();
    node["y"] = val.y();
    return node;
  }

  static bool decode(const YAML::Node &node, T &val)
  {
    if (node.size() != 2)
      return false;

    val.x() = node["x"].as<V>();
    val.y() = node["y"].as<V>();

    return true;
  }
};

template<typename V>
struct convert<Eigen::Matrix<V, 3, 1>>
{
  using T = Eigen::Matrix<V, 3, 1>;

  static Node encode(const T &val)
  {
    YAML::Node node;
    node["x"] = val.x();
    node["y"] = val.y();
    node["z"] = val.z();
    return node;
  }

  static bool decode(const YAML::Node &node, T &val)
  {
    if (node.size() != 3)
      return false;

    val.x() = node["x"].as<V>();
    val.y() = node["y"].as<V>();
    val.z() = node["z"].as<V>();

    return true;
  }
};

template<typename V>
struct convert<Eigen::Transform<V, 3, Eigen::Isometry>>
{
  using T = Eigen::Transform<V, 3, Eigen::Isometry>;

  static Node encode(const T &val)
  {
    YAML::Node node;
    node["x"] = val.translation().x();
    node["y"] = val.translation().y();
    node["z"] = val.translation().z();

    Eigen::Quaternion<V> quat(val.linear());
    node["qw"] = quat.w();
    node["qx"] = quat.x();
    node["qy"] = quat.y();
    node["qz"] = quat.z();

    return node;
  }

  static bool decode(const YAML::Node &node, T &val)
  {
    if (node.size() != 7)
      return false;

    Eigen::Matrix<V, 3, 1> trans;
    trans.x() = node["x"].as<V>();
    trans.y() = node["y"].as<V>();
    trans.z() = node["z"].as<V>();

    Eigen::Quaternion<V> quat;
    quat.w() = node["qw"].as<V>();
    quat.x() = node["qx"].as<V>();
    quat.y() = node["qy"].as<V>();
    quat.z() = node["qz"].as<V>();

    val = Eigen::Translation<V, 3>(trans) * quat;

    return true;
  }
};

} // namespace YAML
