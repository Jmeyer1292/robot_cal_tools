#ifndef POSE_GEN_H
#define POSE_GEN_H

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
   * @param target_pose The position/orientation of the target
   * @param r Radius of the hemisphere
   * @param theta_cnt The number of points in the theta-wise direction
   * @param phi_cnt The number of points in the phi-wise direction
   * @return A vector of camera positions & orientations
   */
  std::vector<Eigen::Isometry3d> genHemispherePose(Eigen::Isometry3d target_pose,
                                                   double r,
                                                   int theta_cnt,
                                                   int phi_cnt
                                                   );
  /**
   * @brief genConicalPose Generates camera positions using a conical template, with the target at the 'point'
   * @param target_pose Position and orientation of the target
   * @param r Radius of the cone
   * @param h Height of the cone (distance to target)
   * @param theta_cnt Number of observations in a theta direction
   * @return A vector of camera positions & orientations
   */
  std::vector<Eigen::Isometry3d> genConicalPose(Eigen::Isometry3d target_pose,
                                                int observations,
                                                double r,
                                                double h
                                                );

  /**
   * @brief genGridPose Generates camera positions in a grid pattern
   * @param target_pose Position & orientation of the target
   * @param grid_side number of columns & rows to go into grid
   * @param spacing Distance betweeon points
   * @param h Grid distance to target
   * @return A vector of camera positions & orientations
   */
  std::vector<Eigen::Isometry3d> genGridPose(Eigen::Isometry3d target_pose,
                                                int grid_side,
                                                double spacing,
                                                double h
                                                );
} //namespace test
} //namespace rct_optimizations
#endif //POSE_GEN
