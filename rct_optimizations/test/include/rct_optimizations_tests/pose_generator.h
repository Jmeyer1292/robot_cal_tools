#pragma once

#include "rct_optimizations_tests/observation_creator.h"
#include <rct_optimizations_tests/utilities.h>
#include <Eigen/Geometry>
#include <vector>

namespace rct_optimizations
{
namespace test
{
/**
 * @brief genHemispherePose Generates camera poses in a hemisphere pattern
 * @param target_pose The position of the target
 * @param r Radius of the hemisphere
 * @param theta_cnt The number of points in the theta-wise direction
 * @param phi_cnt The number of points in the phi-wise direction
 * @return A vector of camera positions & orientations
 */
std::vector<Eigen::Isometry3d> genHemispherePose(const Eigen::Vector3d& target_pose, const double r, const unsigned int theta_cnt, const unsigned int phi_cnt);
/**
 * @brief genConicalPose Generates camera positions using a conical template, with the target at the 'point'
 * @param target_pose The position of the target
 * @param r Radius of the cone
 * @param h Height of the cone (distance to target)
 * @param theta_cnt Number of observations in a theta direction
 * @return A vector of camera positions & orientations
 */
std::vector<Eigen::Isometry3d> genConicalPose(const Eigen::Vector3d& target_pose, const unsigned int observations, const double r, const double h);

/**
 * @brief genGridPose Generates camera positions in a grid pattern
 * @param target_pose The position of the target
 * @param grid_side number of columns & rows to go into grid
 * @param spacing Distance betweeon points
 * @param h Grid distance to target
 * @return A vector of camera positions & orientations
 */
std::vector<Eigen::Isometry3d> genGridPose(const Eigen::Vector3d& target_pose, const unsigned int grid_side, const double spacing, const double h);
}  // namespace test
}  // namespace rct_optimizations
