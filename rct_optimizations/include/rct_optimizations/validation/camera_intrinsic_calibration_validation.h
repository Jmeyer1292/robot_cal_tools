#pragma once

#include <rct_optimizations/experimental/pnp.h>
#include <rct_optimizations/types.h>

namespace rct_optimizations
{
/**
 * @brief Divides a set of correspondences into two halves and calculates the transform from one half to the other
 *   Method:
 *     1. Split the correspondence set into two virtual correspondence sets
 *     2. Solve PnP optimization for each virtual correspondence set to get the transform from the camera to the virtual targets
 *     3. Return the transform between the two virtual targets
 *
 *  Note: the two virtual targets are composed of different points, but those points are relative to the same origin
 *  Therefore, the transformation from one virtual target to the other should be identity, given perfect camera intrinsic parameters
 *
 * @param correspondences - set of corresponding observed features and target features
 * @param intr - camera intrinsic parameters
 * @param camera_to_target_guess - an initial guess about the location of the target relative to the camera (default: identity)
 * @param residual_sq_error_threshold - Max squared error allowed for a PnP optimization.
 * This value should be driven by the accuracy of the sensor providing the observations (default: 1.0 pixel^2)
 * @return the transformation from the first virtual target to the second (identity, given perfect camera intrinsic parameters)
 */
Eigen::Isometry3d getInternalTargetTransformation(
  const Correspondence2D3D::Set &correspondences,
  const CameraIntrinsics &intr,
  const Eigen::Isometry3d &camera_to_target_guess = Eigen::Isometry3d::Identity(),
  const double residual_sq_error_threshold = 1.0);

/**
 * @brief Calculates the mean and variance of the transform between two virtual targets (extracted from a single correspondence set)
 * for a set of calibration observations
 *   Method:
 *     1. Check that the correspondences in all observations are the same size
 *       - Assumptions:
 *         - Correspondences are ordered in the same way for each observation
 *         - The correspondences in each observation can be divided in half to get the same two sets of features
 *     2. Calculate transform between two virtual targets in correspondences in adjacent observations
 *     3. Calculate the mean and standard deviation of the positional difference in this transform for all observations
 *       - Theoretically, this difference should be zero for a perfectly intrinsically calibrated camera
 *         that perfectly observed the target features
 *     4. Compare the mean and standard deviation of all the measurements with the threshold
 *
 * @param observations - a set of calibration observations
 * @param intr - camera intrinsic parameters
 * @param threshold - The max allowable deviation in world units (i.e. meters)
 * This value should be driven by the accuracy of the sensor providing the observations
 * @param camera_mount_to_camera - The transformation from the camera mount frame to the camera
 * @param target_mount_to_target - The transformation from the target mount frame to the target
 * @param camera_base_to_target_base - the transform from the camera base frame to the target base frame (typically identity)
 * @return True if the camera intrinsic calibration is valid, false otherwise
 */
bool validateCameraIntrinsicCalibration(const Observation2D3D::Set &observations,
                                        const CameraIntrinsics &intr,
                                        const double threshold,
                                        const Eigen::Isometry3d &camera_mount_to_camera,
                                        const Eigen::Isometry3d &target_mount_to_target,
                                        const Eigen::Isometry3d &camera_base_to_target_base);

} // namespace rct_optimizations
