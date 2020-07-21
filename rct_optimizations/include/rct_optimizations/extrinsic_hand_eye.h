/*
 * This file defines a solver for calibrating the EXTRINSIC parameters of a 3D
 * camera attached to the wrist of a moving robot. It works by imaging a static target
 * from many robot wrist positions.
 *
 * The only difference between this the and 2D, pinhole variety is that the correspondences
 * in the observation set are 3D to 3D instead of 2D to 3D. This is meant for 3D sensors where
 * you don't know (or don't want to know) the intrinsics of the sensor or they aren't well
 * described by the pinhole model.
 *
 * For example, this calibration has been used to detect 3D features in a "3D image" produced
 * by the IFM O3D3xx cameras. Sometimes you may want to use this for Openni cameras where
 * because of terrible drivers your calibration does not affect the depth data.
 *
 * See extrinsic_camera_on_wrist.h for a description of the other parameters.
 */
#pragma once

#include <rct_optimizations/covariance_types.h>
#include <rct_optimizations/types.h>
#include <Eigen/Dense>
#include <vector>

namespace rct_optimizations
{
struct ExtrinsicHandEyeProblem2D3D
{
  typename Observation2D3D::Set observations;
  CameraIntrinsics intr;
  Eigen::Isometry3d target_mount_to_target_guess;
  Eigen::Isometry3d camera_mount_to_camera_guess;

  const std::array<std::string, 6> labels_isometry3d = {"x", "y", "z", "rx", "ry", "rz"};

  std::string label_target_mount_to_target = "wrist_to_target";

  std::string label_camera_mount_to_camera = "base_to_camera";
};

struct ExtrinsicHandEyeProblem3D3D
{
  typename Observation3D3D::Set observations;
  Eigen::Isometry3d target_mount_to_target_guess;
  Eigen::Isometry3d camera_mount_to_camera_guess;

  const std::array<std::string, 6> labels_isometry3d = {"x", "y", "z", "rx", "ry", "rz"};

  std::string label_target_mount_to_target = "wrist_to_target";

  std::string label_camera_mount_to_camera = "base_to_camera";
};

struct ExtrinsicHandEyeResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Isometry3d target_mount_to_target;
  Eigen::Isometry3d camera_mount_to_camera;

  CovarianceResult covariance;
};

ExtrinsicHandEyeResult optimize(const ExtrinsicHandEyeProblem2D3D &params);
ExtrinsicHandEyeResult optimize(const ExtrinsicHandEyeProblem3D3D &params);

} // namespace rct_optimizations
