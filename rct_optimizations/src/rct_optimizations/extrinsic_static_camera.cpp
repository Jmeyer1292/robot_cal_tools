#include "rct_optimizations/extrinsic_static_camera.h"
#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/eigen_conversions.h"
#include "rct_optimizations/types.h"

#include <ceres/ceres.h>
#include <iostream>

namespace
{

//using rct_optimizations;

//struct ReprojectionCost
//{
//  ReprojectionCost(const Eigen::Vector2d& obs, const CameraIntrinsics& intr, const Eigen::Affine3d& wrist_to_base,
//                   const Eigen::Vector3d& point_in_target)
//      : obs_(obs), intr_(intr), wrist_pose_(poseEigenToCal(wrist_to_base)), target_pt_(point_in_target)
//  {
//  }

//  Eigen::Vector2d obs_;
//  CameraIntrinsics intr_;
//  Pose6d wrist_pose_;
//  Eigen::Vector3d target_pt_;

////  (const double observed_x,
////    const double observed_y, const double focal_length_x,
////    const double focal_length_y, const double optical_center_x,
////    const double optical_center_y, Pose6D link_pose,
////    Point3D point) : observed_x_(observed_x), observed_y_(observed_y),
////    focal_length_x_(focal_length_x), focal_length_y_(focal_length_y),
////    optical_center_x_(optical_center_x), optical_center_y_(optical_center_y),
////    link_pose_(link_pose), point_(point) { }

//  template<typename T> bool operator() (const T* const base_to_camera,
//    const T* target_to_link, T* residual) const
//  {
//    const T* camera_angle_axis(&base_to_camera[0]);
//    const T* camera_position(&base_to_camera[3]);

//    const T* target_angle_axis(&target_to_link[0]);
//    const T* target_position(&target_to_link[3]);

//    T link_point[3]; // Point in link coordinates
//    T world_point[3]; // Point in world coordinates (base of robot)
//    T camera_point[3]; // Point in camera coordinates

//    // Transform point into camera coordinates
//    // If target is attached to the robot tool, each point is first put into the frame of
//    // the tool using the estimated value from (target_to_link)
//    // Then the known transform between base-tool is used to put each point in the frame of
//    // the robot base (world)
//    // Then the point is put into the frame of the camera using the estimated transform from (base_to_camera).
//    transformPoint3D(target_angle_axis, target_position, point_.asVector(), link_point);
//    poseTransformPoint(link_pose_, link_point, world_point);
//    transformPoint(camera_angle_axis, camera_position, world_point, camera_point);

//    // Compute projected point into image plane and compute residual
//    T focal_length_x = T(focal_length_x_);
//    T focal_length_y = T(focal_length_y_);
//    T optical_center_x = T(optical_center_x_);
//    T optical_center_y = T(optical_center_y_);
//    T observed_x = T(observed_x_);
//    T observed_y = T(observed_y_);

//    cameraPointResidual(camera_point, focal_length_x, focal_length_y, optical_center_x,
//      optical_center_y, observed_x, observed_y, residual);

//    return true;
//  }

//};

}

rct_optimizations::ExtrinsicStaticCameraMovingTargetProblem
rct_optimizations::optimize(const rct_optimizations::ExtrinsicStaticCameraMovingTargetResult& params)
{

}
