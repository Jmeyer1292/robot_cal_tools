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
///*
//class CorrGen
//{
//public:
//  /**
//   * @brief CoreGen constructor
//   */
//  CoreGen(Eigen::Isometry3d focus);

//  /**
//   * @brief setTargetType modify target characteristics
//   */

//  void setTargetType();

//  /**
//   * @brief setFocus Set target location
//   */

//  void setFocus();

//  /**
//   * @brief GenHemispherePose
//   * @param r Hemisphere Radius
//   * @param theta_cnt Spatial discretization in the theta-wise direction
//   * @param phi_cnt Spatial discretization in the phi-wise direction
//   */

//  void genHemispherePose(double r,
//                         int theta_cnt,
//                         int phi_cnt
//                         );

//  /**
//   * @brief genConicalPose Generates camera positions using a conical template, with the target at the 'point'
//   * @param r Radius of the cone
//   * @param h Height of the cone (distance to target)
//   * @param theta_cnt Number of observations in a theta direction
//   */
//  void genConicalPose(int observations,
//                      double r,
//                      double h
//                      );

//  /**
//   * @brief clearPoses Resets the camera pose vector
//   */
//  void clearPoses();

//  /**
//   * @brief genObservations Generate observations using internal target, camera positions
//   */
//  void genObservations();

//  /**
//   * @brief gen2dCorrespondence generate 2d correspondences from internal observation vector
//   * @return A vector of 2d Correspondences
//   */
//  std::vector<Correspondence2D3D> gen2dCorrespondence();

//  /**
//   * @brief gen3dCorrespondence generate 23 correspondences from internal observation vector
//   * @return A vector of 3d Correspondences
//   */
//  std::vector<Correspondence3D3D> gen3dCorrespondence();

//private:

//  /**
//   * @brief focus_ Spatial location of the target
//   */
//  Eigen::Isometry3d focus_{ Eigen::Isometry3d::Identity() };

//  /**
//   * @brief target_ Representation of target chracteristics
//   */
//  rct_optimizations::test::Target target_;

//  /**
//   * @brief observations_ observations generated from camera positions
//   */
//  std::vector<Eigen::Vector2d> observations_;


//  /**
//   * @brief camera_positions
//   */
//  std::vector<Eigen::Isometry3d> camera_positions_;

//};

