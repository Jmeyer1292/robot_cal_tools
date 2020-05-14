#ifndef ROBOT_OBSERVATION_CREATOR_H
#define ROBOT_OBSERVATION_CREATOR_H

#include <rct_optimizations/experimental/dh_robot.h>
#include <rct_optimizations/types.h>
#include <rct_optimizations_tests/utilities.h>

namespace rct_optimizations
{
namespace test
{

/**
 * @brief create
 * This function uses a "magical" camera that can obtain the target regardless of view point
 * @param to_camera_chain - DH chain to get to the camera
 * @param to_target_chain - DH chain to get to the target
 * @param camera_base_to_target_base - Transformation between the root link of the camera chain to the root link of the target chain
 * @param target - Observation target definition
 * @param n - Number of samples
 * @return A vector of 3D-3D observations
 */
Observation3D3D::Set create(DHRobot to_camera_chain,
                            DHRobot to_target_chain,
                            const Eigen::Isometry3d &camera_base_to_target_base,
                            const Target &target,
                            const std::size_t n);
/**
 * @brief create2d
 * Attempt a bunch of random poses and in which ones the target is observed
 * @param to_camera_chain
 * @param to_target_chain
 * @param camera_base_to_target_base
 * @param target
 * @param n
 * @return
 */
Observation2D3D::Set create(DHRobot to_camera_chain,
                            DHRobot to_target_chain,
                            const Eigen::Isometry3d &camera_base_to_target_base,
                            const Target &target,
                            const Camera &camera,
                            const std::size_t n);

} // namespace test
} // namespace rct_optimizations

#endif // ROBOT_OBSERVATION_CREATOR_H
