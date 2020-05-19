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
   * @brief genHemispherePose
   * @param target_pose
   * @param r
   * @param theta_cnt
   * @param phi_cnt
   * @return
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
   */
  std::vector<Eigen::Isometry3d> genConicalPose(Eigen::Isometry3d target_pose,
                                                int observations,
                                                double r,
                                                double h
                                                );

} //namespace test
} //namespace rct_optimizations
#endif //POSE_GEN
