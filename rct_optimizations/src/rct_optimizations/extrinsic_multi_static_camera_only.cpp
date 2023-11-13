#include "rct_optimizations/extrinsic_multi_static_camera_only.h"
#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/eigen_conversions.h"
#include "rct_optimizations/types.h"

#include <ceres/ceres.h>
#include <iostream>

using namespace rct_optimizations;

namespace
{
class ReprojectionFreeCameraCost
{
public:
  ReprojectionFreeCameraCost(const Eigen::Vector2d& obs,
                             const CameraIntrinsics& intr,
                             const Eigen::Vector3d& point_in_target)
    : obs_(obs), intr_(intr), target_pt_(point_in_target)
  {
  }

  template <typename T>
  bool operator()(const T* const pose_camera_to_base, const T* pose_base_to_target, T* residual) const
  {
    const T* camera_angle_axis = pose_camera_to_base + 0;
    const T* camera_position = pose_camera_to_base + 3;

    const T* target_angle_axis = pose_base_to_target + 0;
    const T* target_position = pose_base_to_target + 3;

    T world_point[3];   // Point in world coordinates (base of robot)
    T camera_point[3];  // Point in camera coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    transformPoint(target_angle_axis, target_position, target_pt, world_point);
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
  Eigen::Vector3d target_pt_;
};

class ReprojectionFixedCameraCost
{
public:
  ReprojectionFixedCameraCost(const Eigen::Vector2d& obs,
                              const CameraIntrinsics& intr,
                              const Eigen::Isometry3d& base_to_camera,
                              const Eigen::Vector3d& point_in_target)
    : obs_(obs), intr_(intr), camera_to_base_(poseEigenToCal(base_to_camera.inverse())), target_pt_(point_in_target)
  {
  }

  template <typename T>
  bool operator()(const T* const pose_base_to_target, T* residual) const
  {
    const T* target_angle_axis = pose_base_to_target + 0;
    const T* target_position = pose_base_to_target + 3;

    T world_point[3];   // Point in world coordinates (base of robot)
    T camera_point[3];  // Point in camera coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    transformPoint(target_angle_axis, target_position, target_pt, world_point);
    poseTransformPoint(camera_to_base_, world_point, camera_point);

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
  Pose6d camera_to_base_;
  Eigen::Vector3d target_pt_;
};

}  // namespace

rct_optimizations::ExtrinsicMultiStaticCameraOnlyResult
rct_optimizations::optimize(const rct_optimizations::ExtrinsicMultiStaticCameraOnlyProblem& params)
{
  std::vector<Pose6d> internal_base_to_target;
  std::vector<Pose6d> internal_camera_to_base;
  internal_camera_to_base.resize(params.base_to_camera_guess.size());
  internal_base_to_target.resize(params.base_to_target_guess.size());

  ceres::Problem problem;

  for (std::size_t i = 0; i < params.base_to_target_guess.size(); ++i)  // For each wrist pose / image set
  {
    internal_base_to_target[i] = poseEigenToCal(params.base_to_target_guess[i]);
    for (std::size_t c = 0; c < params.base_to_camera_guess.size(); ++c)  // For each camera
    {
      assert(params.image_observations[c].size() == params.base_to_target_guess.size());
      internal_camera_to_base[c] = poseEigenToCal(params.base_to_camera_guess[c].inverse());

      for (std::size_t j = 0; j < params.image_observations[c][i].size();
           ++j)  // For each 3D point seen in the 2D image
      {
        // Define
        const auto& img_obs = params.image_observations[c][i][j].in_image;
        const auto& point_in_target = params.image_observations[c][i][j].in_target;
        const auto& intr = params.intr[c];

        // Allocate Ceres data structures - ownership is taken by the ceres
        // Problem data structure
        if (params.fix_first_camera && (c == 0))
        {
          auto* cost_fn =
              new ReprojectionFixedCameraCost(img_obs, intr, params.base_to_camera_guess[c], point_in_target);

          auto* cost_block = new ceres::AutoDiffCostFunction<ReprojectionFixedCameraCost, 2, 6>(cost_fn);

          problem.AddResidualBlock(cost_block, NULL, internal_base_to_target[i].values.data());
        }
        else
        {
          auto* cost_fn = new ReprojectionFreeCameraCost(img_obs, intr, point_in_target);

          auto* cost_block = new ceres::AutoDiffCostFunction<ReprojectionFreeCameraCost, 2, 6, 6>(cost_fn);

          problem.AddResidualBlock(
              cost_block, NULL, internal_camera_to_base[c].values.data(), internal_base_to_target[i].values.data());
        }
      }
    }  // for each wrist pose
  }    // end for each camera

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicMultiStaticCameraOnlyResult result;
  result.base_to_camera.resize(params.base_to_camera_guess.size());
  result.base_to_target.resize(params.base_to_target_guess.size());
  result.converged = summary.termination_type == ceres::CONVERGENCE;

  for (std::size_t i = 0; i < params.base_to_camera_guess.size(); ++i)
    result.base_to_camera[i] = poseCalToEigen(internal_camera_to_base[i]).inverse();

  for (std::size_t i = 0; i < params.base_to_target_guess.size(); ++i)
    result.base_to_target[i] = poseCalToEigen(internal_base_to_target[i]);

  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  return result;
}
