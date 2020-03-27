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
#ifndef EXTRINSIC_3D_CAMERA_ON_WRIST_H
#define EXTRINSIC_3D_CAMERA_ON_WRIST_H

#include "rct_optimizations/types.h"
#include <Eigen/Dense>
#include <vector>

namespace rct_optimizations
{

struct Extrinsic3DCameraOnWristProblem
{
  std::vector<Eigen::Isometry3d> wrist_poses;
  std::vector<Correspondence3DSet> observations;

  Eigen::Isometry3d base_to_target_guess;
  Eigen::Isometry3d wrist_to_camera_guess;
};

struct Extrinsic3DCameraOnWristResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Isometry3d base_to_target;
  Eigen::Isometry3d wrist_to_camera;
};

Extrinsic3DCameraOnWristResult optimize(const Extrinsic3DCameraOnWristProblem& params);

}

#endif // EXTRINSIC_3D_CAMERA_ON_WRIST_H
