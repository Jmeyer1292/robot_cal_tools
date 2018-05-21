#include "rct_optimizations/extrinsic_camera_on_wrist.h"
#include "rct_optimizations/types.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <iostream>

using namespace rct_optimizations;

namespace
{

template<typename T>
inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3])
{
  ceres::AngleAxisRotatePoint(angle_axis, point, t_point);

  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

template<typename T>
inline void poseTransformPoint(const Pose6d &pose, const T point[3], T t_point[3])
{
  T angle_axis[3];

  angle_axis[0]  = T(pose.rx());
  angle_axis[1]  = T(pose.ry());
  angle_axis[2]  = T(pose.rz());

  T translation[3];
  translation[0] = T(pose.x());
  translation[1] = T(pose.y());
  translation[2] = T(pose.z());

  transformPoint(angle_axis, translation, point, t_point);
}

template <typename T>
inline void projectPoint(const CameraIntrinsics& intr, const T point[3], T xy_image[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  // Scale into the image plane by distance away from camera
  T xp, yp;

  if (zp1 == T(0)) // Avoid divide by zero
  {
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  // Perform projection using focal length and camera optical center into image plane
  xy_image[0] = intr.fx() * xp + intr.cx();
  xy_image[1] = intr.fy() * yp + intr.cy();
}

class ReprojectionCost
{
public:
  /**
   * @brief A CERES cost function class that represents a single observation. Each observation is:
   *  - One point on the calibration target as seen by the camera with the given intrinsics
   *  - Associated with a given wrist position (which may have produced many such calibrations)
   * @param obs
   * @param intr
   * @param wrist_pose
   * @param point_in_target
   */
  ReprojectionCost(const Eigen::Vector2d& obs, const CameraIntrinsics& intr, const Eigen::Affine3d& wrist_to_base,
                   const Eigen::Vector3d& point_in_target)
    : obs_(obs), intr_(intr), wrist_pose_(poseEigenToCal(wrist_to_base)), target_pt_(point_in_target)
  {}

  template <typename T>
  bool operator() (const T* pose_camera_to_wrist, const T* pose_base_to_target, T* residual) const
  {
    const T* camera_angle_axis = pose_camera_to_wrist + 0;
    const T* camera_position = pose_camera_to_wrist + 3;

    const T* target_angle_axis = pose_base_to_target + 0;
    const T* target_position = pose_base_to_target + 3;

    T world_point[3]; // Point in world coordinates
    T link_point[3]; // Point in link coordinates
    T camera_point[3]; // Point in camera coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));
    transformPoint(target_angle_axis, target_position, target_pt, world_point);
    poseTransformPoint(wrist_pose_, world_point, link_point);
    transformPoint(camera_angle_axis, camera_position, link_point, camera_point);

    T xy_image [2];
    projectPoint(intr_, camera_point, xy_image);

    residual[0] = xy_image[0] - obs_.x();
    residual[1] = xy_image[1] - obs_.y();

    return true;
  }

private:
  Eigen::Vector2d obs_;
  CameraIntrinsics intr_;
  Pose6d wrist_pose_;
  Eigen::Vector3d target_pt_;
};

}

rct_optimizations::ExtrinsicCameraOnWristResult rct_optimizations::optimize(const ExtrinsicCameraOnWristProblem &params)
{
  assert(params.image_observations.size() == params.wrist_poses.size());

  Pose6d internal_base_to_target = poseEigenToCal(params.base_to_target_guess);
  Pose6d internal_camera_to_wrist = poseEigenToCal(params.wrist_to_camera_guess.inverse());

  ceres::Problem problem;

  for (std::size_t i = 0; i < params.wrist_poses.size(); ++i) // For each wrist pose / image set
  {
    for (std::size_t j = 0; j < params.image_observations[i].size(); ++j) // For each 3D point seen in the 2D image
    {
      // Define
      const auto& img_obs = params.image_observations[i][j].in_image;
      const auto& point_in_target = params.image_observations[i][j].in_target;
      const auto wrist_to_base = params.wrist_poses[i].inverse();

      // Allocate Ceres data structures - ownership is taken by the ceres Problem data structure
      auto* cost_fn = new ReprojectionCost(img_obs, params.intr, wrist_to_base, point_in_target);

      auto* cost_block = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 6, 6>(cost_fn);

      problem.AddResidualBlock(cost_block, NULL, internal_camera_to_wrist.values.data(),
                               internal_base_to_target.values.data());
    }
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicCameraOnWristResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.base_to_target = poseCalToEigen(internal_base_to_target);
  result.wrist_to_camera = poseCalToEigen(internal_camera_to_wrist).inverse();
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  return result;
}
