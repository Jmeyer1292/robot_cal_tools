#include "rct_optimizations/experimental/multi_camera_pnp.h"
#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/eigen_conversions.h"
#include "rct_optimizations/types.h"

#include <ceres/ceres.h>

using namespace rct_optimizations;

namespace
{

class ReprojectionCost
{
public:
  ReprojectionCost(const Eigen::Vector2d& obs,
                   const CameraIntrinsics& intr,
                   const Eigen::Affine3d& camera_to_base,
                   const Eigen::Vector3d& point_in_target)
    : obs_(obs),
      intr_(intr),
      camera_to_base_pose_(poseEigenToCal(camera_to_base)),
      target_pt_(point_in_target)
  {}

  template <typename T>
  bool operator() (const T* const pose_base_to_target, T* residual) const
  {
    const T* target_angle_axis = pose_base_to_target + 0;
    const T* target_position = pose_base_to_target + 3;

    T world_point[3]; // Point in world coordinates (base of robot)
    T camera_point[3]; // Point in camera coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    transformPoint(target_angle_axis, target_position, target_pt, world_point);
    poseTransformPoint(camera_to_base_pose_, world_point, camera_point);

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
  Pose6d camera_to_base_pose_;
  Eigen::Vector3d target_pt_;
};

} // end anon ns

rct_optimizations::MultiCameraPnPResult
rct_optimizations::optimize(const rct_optimizations::MultiCameraPnPProblem& params)
{
  Pose6d internal_base_to_target = poseEigenToCal(params.base_to_target_guess);

  ceres::Problem problem;

  for (std::size_t c = 0; c < params.base_to_camera.size(); ++c) // For each camera
  {
    for (std::size_t i = 0; i < params.image_observations[c].size(); ++i) // For each 3D point seen in the 2D image
    {
      // Define
      const auto& img_obs = params.image_observations[c][i].in_image;
      const auto& point_in_target = params.image_observations[c][i].in_target;
      const auto& base_to_camera = params.base_to_camera[c];
      const auto& intr = params.intr[c];

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new ReprojectionCost(img_obs, intr, base_to_camera.inverse(), point_in_target);

      auto* cost_block = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 6>(cost_fn);

      problem.AddResidualBlock(cost_block, NULL, internal_base_to_target.values.data());
    }
  } // end for each camera

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  MultiCameraPnPResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.base_to_target = poseCalToEigen(internal_base_to_target);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  return result;
}
