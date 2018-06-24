#include "rct_optimizations/extrinsic_multi_static_camera.h"
#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/eigen_conversions.h"
#include "rct_optimizations/types.h"

#include <ceres/ceres.h>
#include <iostream>

using namespace rct_optimizations;

namespace
{

class ReprojectionCost
{
public:
  ReprojectionCost(const Eigen::Vector2d& obs, const CameraIntrinsics& intr, const Eigen::Affine3d& base_to_wrist,
                   const Eigen::Vector3d& point_in_target)
    : obs_(obs), intr_(intr), wrist_pose_(poseEigenToCal(base_to_wrist)), target_pt_(point_in_target)
  {}

  template <typename T>
  bool operator() (const T* const pose_camera_to_base, const T* pose_wrist_to_target, T* residual) const
  {
    const T* camera_angle_axis = pose_camera_to_base + 0;
    const T* camera_position = pose_camera_to_base + 3;

    const T* target_angle_axis = pose_wrist_to_target + 0;
    const T* target_position = pose_wrist_to_target + 3;

    T link_point[3]; // Point in wrist coordinates
    T world_point[3]; // Point in world coordinates (base of robot)
    T camera_point[3]; // Point in camera coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    transformPoint(target_angle_axis, target_position, target_pt, link_point);
    poseTransformPoint(wrist_pose_, link_point, world_point);
    transformPoint(camera_angle_axis, camera_position, world_point, camera_point);

    // Compute projected point into image plane and compute residual
    T xy_image[2];
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

rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetResult
rct_optimizations::optimize(const rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetProblem& params)
{
  Pose6d internal_wrist_to_target = poseEigenToCal(params.wrist_to_target_guess);

  std::vector<Pose6d> internal_camera_to_base;
  internal_camera_to_base.resize(params.base_to_camera_guess.size());

  ceres::Problem problem;

  for (std::size_t c = 0; c < params.base_to_camera_guess.size(); ++c)
  {
    assert(params.image_observations[c].size() == params.wrist_poses[c].size());
    internal_camera_to_base[c] = poseEigenToCal(params.base_to_camera_guess[c].inverse());
    for (std::size_t i = 0; i < params.wrist_poses[c].size(); ++i) // For each wrist pose / image set
    {
      for (std::size_t j = 0; j < params.image_observations[c][i].size(); ++j) // For each 3D point seen in the 2D image
      {
        // Define
        const auto& img_obs = params.image_observations[c][i][j].in_image;
        const auto& point_in_target = params.image_observations[c][i][j].in_target;
        const auto wrist_to_base = params.wrist_poses[c][i];//.inverse();
        const auto& intr = params.intr[c];

        // Allocate Ceres data structures - ownership is taken by the ceres
        // Problem data structure
        auto* cost_fn = new ReprojectionCost(img_obs, intr, wrist_to_base, point_in_target);

        auto* cost_block = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 6, 6>(cost_fn);

        problem.AddResidualBlock(cost_block, NULL, internal_camera_to_base[c].values.data(),
                                 internal_wrist_to_target.values.data());
      }
    }
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicMultiStaticCameraMovingTargetResult result;
  result.base_to_camera.resize(params.base_to_camera_guess.size());
  result.converged = summary.termination_type == ceres::CONVERGENCE;

  for (std::size_t i = 0; i < params.base_to_camera_guess.size(); ++i)
    result.base_to_camera[i] = poseCalToEigen(internal_camera_to_base[i]).inverse();

  result.wrist_to_target = poseCalToEigen(internal_wrist_to_target);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  return result;
}
