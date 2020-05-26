#ifndef RCT_OPTIMIZATIONS_TESTS_OBS_CREATOR_H
#define RCT_OPTIMIZATIONS_TESTS_OBS_CREATOR_H

#include <rct_optimizations_tests/utilities.h>

namespace rct_optimizations
{
namespace test
{

/**
 * @brief Observes a set of simulated 2D-3D correspondences given a 2D camera and target definition
 * This method performs validity checks to ensure target observations appear in the camera field of view
 * @param camera_pose - the transform from a common frame to the camera
 * @param target_pose - the transform from a common frame to the target
 * @param camera - the camera intrinsic parameters
 * @param target - the observation target definition
 * @param require_all - flag to require all target observations be seen in the camera
 * @return A set of all "seen" correspondences
 * @throws Exception if @ref require_all is true and not all observations are seen
 */
Correspondence2D3D::Set getCorrespondences(const Eigen::Isometry3d &camera_pose,
                                           const Eigen::Isometry3d &target_pose,
                                           const Camera &camera,
                                           const Target &target,
                                           const bool require_all = false);

/**
 * @brief Observes a set of simulated 3D-3D correspondences given a target definition.
 * Note this method does not consider whether or not target observations can be seen by the camera
 * @param camera_pose - the transform from a common frame to the camera
 * @param target_pose - the transform from a common frame to the target
 * @param target - the observation target definition
 * @return
 */
Correspondence3D3D::Set getCorrespondences(const Eigen::Isometry3d &camera_pose,
                                           const Eigen::Isometry3d &target_pose,
                                           const Target &target) noexcept;

/**
 * @brief Defines a camera matrix using a camera origin, a position its looking at, and an up vector hint
 * @param origin - The position of the camera focal point
 * @param eye - A point that the camera is looking at
 * @param up - The upvector in world-space
 */
Eigen::Isometry3d lookAt(const Eigen::Vector3d &origin,
                         const Eigen::Vector3d &eye,
                         const Eigen::Vector3d &up) noexcept;

} // namespace test
} // namespace rct_optimizations

#endif // RCT_OPTIMIZATIONS_TESTS_OBS_CREATOR_H
