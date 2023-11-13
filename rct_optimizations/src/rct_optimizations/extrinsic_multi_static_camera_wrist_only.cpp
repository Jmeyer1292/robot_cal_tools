#include "rct_optimizations/extrinsic_multi_static_camera_wrist_only.h"
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
  ReprojectionCost(const Eigen::Vector2d& obs,
                   const CameraIntrinsics& intr,
                   const Eigen::Isometry3d& base_to_wrist,
                   const Eigen::Isometry3d& base_to_camera,
                   const Eigen::Vector3d& point_in_target)
    : obs_(obs)
    , intr_(intr)
    , wrist_pose_(poseEigenToCal(base_to_wrist))
    , camera_to_base_orig_(poseEigenToCal(base_to_camera.inverse()))
    , target_pt_(point_in_target)
  {
  }

  template <typename T>
  bool operator()(const T* const pose_camera_to_base_correction, const T* pose_wrist_to_target, T* residual) const
  {
    const T* camera_angle_axis = pose_camera_to_base_correction + 0;
    const T* camera_position = pose_camera_to_base_correction + 3;

    const T* target_angle_axis = pose_wrist_to_target + 0;
    const T* target_position = pose_wrist_to_target + 3;

    T link_point[3];         // Point in wrist coordinates
    T world_point[3];        // Point in world coordinates (base of robot)
    T camera_point_orig[3];  // Point in camera coordinates before correction
    T camera_point[3];       // Point in camera coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    transformPoint(target_angle_axis, target_position, target_pt, link_point);
    poseTransformPoint(wrist_pose_, link_point, world_point);
    transformPoint(camera_angle_axis, camera_position, world_point, camera_point_orig);
    poseTransformPoint(camera_to_base_orig_, camera_point_orig, camera_point);

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
  Pose6d camera_to_base_orig_;
  Eigen::Vector3d target_pt_;
};

}  // namespace

rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetWristOnlyResult
rct_optimizations::optimize(const rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetWristOnlyProblem& params)
{
  Pose6d internal_wrist_to_target = poseEigenToCal(params.wrist_to_target_guess);

  Pose6d internal_camera_to_base_correction = poseEigenToCal(Eigen::Isometry3d::Identity());

  ceres::Problem problem;

  for (std::size_t c = 0; c < params.base_to_camera_guess.size(); ++c)  // For each camera
  {
    assert(params.image_observations[c].size() == params.wrist_poses.size());
    for (std::size_t i = 0; i < params.wrist_poses.size(); ++i)  // For each wrist pose / image set
    {
      for (std::size_t j = 0; j < params.image_observations[c][i].size();
           ++j)  // For each 3D point seen in the 2D image
      {
        // Define
        const auto& img_obs = params.image_observations[c][i][j].in_image;
        const auto& point_in_target = params.image_observations[c][i][j].in_target;
        const auto& base_to_wrist = params.wrist_poses[i];
        const auto& base_to_camera_orig = params.base_to_camera_guess[c];
        const auto& intr = params.intr[c];

        // Allocate Ceres data structures - ownership is taken by the ceres
        // Problem data structure
        auto* cost_fn = new ReprojectionCost(img_obs, intr, base_to_wrist, base_to_camera_orig, point_in_target);

        auto* cost_block = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 6, 6>(cost_fn);

        problem.AddResidualBlock(
            cost_block, NULL, internal_camera_to_base_correction.values.data(), internal_wrist_to_target.values.data());
      }
    }  // for each wrist pose
  }    // end for each camera

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicMultiStaticCameraMovingTargetWristOnlyResult result;
  result.base_to_camera.resize(params.base_to_camera_guess.size());
  result.converged = summary.termination_type == ceres::CONVERGENCE;

  Eigen::Isometry3d base_to_camera_correction = poseCalToEigen(internal_camera_to_base_correction).inverse();
  for (std::size_t i = 0; i < params.base_to_camera_guess.size(); ++i)
    result.base_to_camera[i] = base_to_camera_correction * params.base_to_camera_guess[i];

  result.wrist_to_target = poseCalToEigen(internal_wrist_to_target);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  return result;
}
