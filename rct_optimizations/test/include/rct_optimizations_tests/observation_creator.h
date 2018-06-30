#ifndef RCT_OPTIMIZATIONS_TESTS_OBS_CREATOR_H
#define RCT_OPTIMIZATIONS_TESTS_OBS_CREATOR_H

#include <rct_optimizations_tests/utilities.h>

namespace rct_optimizations
{
namespace test
{

// Given a virtual camera position, produce the expected 2D observations
bool project(const Eigen::Affine3d& camera_pose, const Eigen::Affine3d& target_pose,
             const Camera& camera, const Target& target, std::vector<Eigen::Vector2d>& out_observations);

/**
 * @brief Defines a camera matrix using a camera origin, a position its looking at, and an up vector hint
 * @param origin The position of the camera focal point
 * @param eye A point that the camera is looking at
 * @param up The upvector in world-space
 */
Eigen::Affine3d lookat(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up);

}
}

#endif // RCT_OPTIMIZATIONS_TESTS_OBS_CREATOR_H
