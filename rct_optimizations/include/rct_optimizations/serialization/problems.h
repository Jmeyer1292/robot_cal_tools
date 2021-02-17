#pragma once

#include <rct_optimizations/extrinsic_hand_eye.h>
#include <rct_optimizations/serialization/types.h>

namespace YAML
{
template<>
struct convert<rct_optimizations::ExtrinsicHandEyeProblem2D3D>
{
  using T = rct_optimizations::ExtrinsicHandEyeProblem2D3D;

  static Node encode(const T &rhs)
  {
    Node node;

    node["intr"] = rhs.intr;
    node["camera_mount_to_camera_guess"] = rhs.camera_mount_to_camera_guess;
    node["target_mount_to_target_guess"] = rhs.target_mount_to_target_guess;
    node["observations"] = rhs.observations;

    return node;
  }

  static bool decode(const Node &node, T &rhs)
  {
    if (node.size() != 4)
      return false;

    rhs.intr = node["intr"].as<decltype(rhs.intr)>();
    rhs.camera_mount_to_camera_guess = node["camera_mount_to_camera_guess"].as<decltype(rhs.camera_mount_to_camera_guess)>();
    rhs.target_mount_to_target_guess = node["target_mount_to_target_guess"].as<decltype(rhs.target_mount_to_target_guess)>();
    rhs.observations = node["observations"].as<decltype(rhs.observations)>();

    return true;
  }
};

template<>
struct convert<rct_optimizations::ExtrinsicHandEyeProblem3D3D>
{
  using T = rct_optimizations::ExtrinsicHandEyeProblem3D3D;

  static Node encode(const T &rhs)
  {
    Node node;

    node["camera_mount_to_camera_guess"] = rhs.camera_mount_to_camera_guess;
    node["target_mount_to_target_guess"] = rhs.target_mount_to_target_guess;
    node["observations"] = rhs.observations;

    return node;
  }

  static bool decode(const Node &node, T &rhs)
  {
    if (node.size() != 3)
      return false;

    rhs.camera_mount_to_camera_guess = node["camera_mount_to_camera_guess"].as<decltype(rhs.camera_mount_to_camera_guess)>();
    rhs.target_mount_to_target_guess = node["target_mount_to_target_guess"].as<decltype(rhs.target_mount_to_target_guess)>();
    rhs.observations = node["observations"].as<decltype(rhs.observations)>();

    return true;
  }
};

template<>
struct convert<rct_optimizations::ExtrinsicHandEyeResult>
{
  using T = rct_optimizations::ExtrinsicHandEyeResult;
  static Node encode(const T& rhs)
  {
    Node node;
    node["converged"] = rhs.converged;
    node["initial_cost_per_obs"] = rhs.initial_cost_per_obs;
    node["final_cost_per_obs"] = rhs.final_cost_per_obs;
    node["target_mount_to_target_pos"] = rhs.target_mount_to_target;
    node["camera_mount_to_camera_pos"] = rhs.camera_mount_to_camera;

    // Serialize transform orientation in RPY for convenience
    node["target_mount_to_target_rpy"] = Eigen::Vector3d (rhs.target_mount_to_target.rotation().eulerAngles(2, 1, 0).reverse());
    node["camera_mount_to_camera_rpy"] = Eigen::Vector3d(rhs.camera_mount_to_camera.rotation().eulerAngles(2, 1, 0).reverse());
    return node;
  }
};

} // namespace YAML
