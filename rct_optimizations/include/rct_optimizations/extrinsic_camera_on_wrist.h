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
 * 4. The "camera frame" or "camera optical frame". This frame is solved for and should be
 *    fixed with respect to the "wrist frame". Uses the OpenCV model for camera coordinates.
 *
 * Inputs:
 *  - Camera intrisic parameters (fx, fy, cx, cy)
 *  - A sequence of transformations: "base frame" to "wrist frame" for each image position
 *  - A sequence of correspondence sets for each image position
 *  - Each correspondence set is a set of correspondences: a position in the "target frame" (3D)
 *    and where that point was seen in the image (2D).
 *  - You must also provide a guess for the two transforms being calibrated.
 *
 * Output:
 *  - The best-fit transform from robot "wrist frame" to the camera optical frame
 *  - The best-fit transform from "base frame" to the "target frame"
 *  - A final cost per observation representing the average reprojection error, in pixels,
 *    along the u or v image axis.
 *
 * To use: Fill out ExtrinsicCameraOnWristProblem then call optimize()
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
   * @brief The pinhole camera parameters used to project into the rectified image
   */
  CameraIntrinsics intr;

  /**
   * @brief The transforms, "base to wrist", at which each observation set was taken. Should be
   * same size as @e image_observations.
   */
  std::vector<Eigen::Isometry3d> wrist_poses;

  /**
   * @brief A sequence of observation sets corresponding to the image locations in @e wrist_poses.
   * Each observation set consists of a set of correspodences: a 3D position (e.g. a dot) in "target
   * frame" to the image location it was detected at (2D).
   */
  std::vector<CorrespondenceSet> image_observations;

  /**
   * @brief Your best guess at the transform from "base frame" to "target frame". Should be static
   * as the robot moves.
   */
  Eigen::Isometry3d base_to_target_guess;

  /**
   * @brief Your best guess at teh transform from "wrist frame" to the camera optical frame.
   */
  Eigen::Isometry3d wrist_to_camera_guess;
};

struct ExtrinsicCameraOnWristResult
{
  /**
   * @brief Whether the underlying solver converged. If this is false, your calibration did not go well.
   * If this is true, your calibration MAY have gone well.
   */
  bool converged;

  /**
   * @brief The initial reprojection error (in pixels) per residual based on your input guesses.
   */
  double initial_cost_per_obs;

  /**
   * @brief The final reprojection error (in pixels) per residual after optimization. Note that each circle
   * has two residuals: a U and V error in the image. So a value of 1.2 means that each circle was described
   * to within 1.2 pixels in X and 1.2 pixels in Y.
   *
   * A low value here is encouraging if you had a diversity of images. If you took few images, you can get
   * a low score without getting a calibration that describes your workcell.
   */
  double final_cost_per_obs;

  /**
   * @brief The final calibrated result of "base frame" to "target frame".
   */
  Eigen::Isometry3d base_to_target;

  /**
   * @brief The final calibrated result of "wrist frame" to camera optical frame.
   */
  Eigen::Isometry3d wrist_to_camera;
};

ExtrinsicCameraOnWristResult optimize(const ExtrinsicCameraOnWristProblem& params);

}

#endif
