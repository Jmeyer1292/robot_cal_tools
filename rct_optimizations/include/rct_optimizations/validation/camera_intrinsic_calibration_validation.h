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
 * @param residual_sq_error_threshold - Max squared error allowed for a PnP optimization (default: 1.0 pixel^2)
 * @return the transformation from the first virtual target to the second (identity, given perfect camera intrinsic parameters)
 */
Eigen::Isometry3d getInternalTargetTransformation(
  const Correspondence2D3D::Set &correspondences,
  const CameraIntrinsics &intr,
  const Eigen::Isometry3d &camera_to_target_guess = Eigen::Isometry3d::Identity(),
  const double residual_sq_error_threshold = 1.0);

/**
 * @brief
 *   Method:
 *     1. Check that the correspondences in all observations are the same size
 *       - Assumptions:
 *         - Correspondences are ordered in the same way for each observation
 *         - The correspondences in each observation can be divided in half to get the same two sets of features
 *     2. Calculate transform between two virtual targets in correspondences in adjacent observations
 *     3. Calculate the positional difference in this transform for all observations
 *       - Theoretically, this difference should be zero for a perfectly intrinsically calibrated camera
 *         that perfectly observed the target features
 *     4. Record the mean and standard deviation of
 *     5. Return whether or not the mean of all the measurements was less than the input threshold
 *
 * @param observations
 * @param intr - camera intrinsic parameters
 * @param camera_to_target_guess -
 * @param diff_threshold - The max error
 * @return
 */
bool validateCameraIntrinsicCalibration(const Observation2D3D::Set &observations,
                                        const CameraIntrinsics &intr,
                                        const Eigen::Isometry3d &camera_to_target_guess,
                                        const double diff_threshold);

} // namespace rct_optimizations
