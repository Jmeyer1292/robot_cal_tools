/*
 * This file defines a solver for calibrating the EXTRINSIC parameters of a pinhole
 * camera attached to the wrist of a moving robot. It works by imaging a static target
 * from many robot wrist positions.
 *
 * This calibration works on undistorted or rectified images!
 *
 * This calibration uses the following "frames":
 * 1. A "base frame" in which the robot's wrist positions and the target frame are reported
 *    in. It should be static with respect to the target. This will commonly be the robot
 *    "base_link" or your workcell's "world" frame.
 * 2. A "wrist frame" to which your camera is rigidly attached. You must give this frame's
 *    position with respect to the above "base frame" at every image location. This will
 *    commonly be "tool0" or some fixed offset from the robot's flange.
 * 3. A "target frame" which is represented as the origin of your rigid 3D pattern. It is
 *    in this frame that the 3D entries of ObservationSet are given. For the circle grid
 *    target, this is the big dot, with +X going left to right across the bottom row and +Y
 *    going up the first column.
 *
 * Inputs:
 *  - Camera intrisic parameters (fx, fy, cx, cy)
 *  - A sequence of transformations: "base frame" to "wrist frame" for each image position
 *  - A sequence of observation sets for each image position
 *  - Each observation set is a set of correspondences: a position in the "target frame" (3D)
 *    and where that point was seen in the image (2D).
 *  - You must also provide a guess for the two transforms being calibrated.
 *
 * Output:
 *  - The best-fit transform from robot "wrist frame" to the camera optical frame
 *  - The best-fit transform from "base frame" to the "target frame"
 *  - A final cost per observation representing the average reprojection error, in pixels,
 *    along the u or v image axis.
 *
 *  author: Jonathan Meyer
 */

#ifndef EXTRINSIC_CAMERA_ON_WRIST_H
#define EXTRINSIC_CAMERA_ON_WRIST_H

#include "rct_optimizations/types.h"
#include <Eigen/Dense>
#include <vector>

namespace rct_optimizations
{

struct ExtrinsicCameraOnWristProblem
{
  /**
   * @brief intr
   */
  CameraIntrinsics intr;

  /**
   * @brief wrist_poses
   */
  std::vector<Eigen::Affine3d> wrist_poses;

  /**
   * @brief image_observations
   */
  std::vector<ObservationSet> image_observations;

  /**
   * @brief base_to_target_guess
   */
  Eigen::Affine3d base_to_target_guess;

  /**
   * @brief wrist_to_camera_guess
   */
  Eigen::Affine3d wrist_to_camera_guess;
};

struct ExtrinsicCameraOnWristResult
{
  /**
   * @brief converged
   */
  bool converged;

  /**
   * @brief initial_cost_per_obs
   */
  double initial_cost_per_obs;

  /**
   * @brief final_cost_per_obs
   */
  double final_cost_per_obs;

  /**
   * @brief base_to_target
   */
  Eigen::Affine3d base_to_target;

  /**
   * @brief wrist_to_camera
   */
  Eigen::Affine3d wrist_to_camera;
};

ExtrinsicCameraOnWristResult optimize(const ExtrinsicCameraOnWristProblem& params);
}

#endif
