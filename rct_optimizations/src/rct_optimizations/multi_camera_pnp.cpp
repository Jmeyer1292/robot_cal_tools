#include "rct_optimizations/experimental/multi_camera_pnp.h"
#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/eigen_conversions.h"

#include <ceres/ceres.h>

namespace
{

struct SolvePnPCostFunc
{
public:
  SolvePnPCostFunc(const rct_optimizations::CameraIntrinsics& intr, const Eigen::Vector3d& pt_in_target,
                   const Eigen::Vector2d& pt_in_image,
                   const rct_optimizations::Pose6d& trans_to_first_camera)
    : intr_(intr), in_target_(pt_in_target), in_image_(pt_in_image)
    , trans_to_first_camera_(trans_to_first_camera)
  {}

  template<typename T>
  bool operator()(const T* const target_pose, T* const residual) const
  {
    const T* target_angle_axis = target_pose + 0;
    const T* target_position = target_pose + 3;

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(in_target_(0));
    target_pt[1] = T(in_target_(1));
    target_pt[2] = T(in_target_(2));

    T camera_point[3];  // Point in the FIRST camera coordinates
    rct_optimizations::transformPoint(target_angle_axis, target_position, target_pt, camera_point);

    T this_camera_point[3]; // Point in THIS camera's coordinates
    rct_optimizations::poseTransformPoint(trans_to_first_camera_, camera_point, this_camera_point);

    T xy_image[2];
    rct_optimizations::projectPoint(intr_, camera_point, xy_image);

    residual[0] = xy_image[0] - in_image_.x();
    residual[1] = xy_image[1] - in_image_.y();

    return true;
  }

  rct_optimizations::CameraIntrinsics intr_;
  Eigen::Vector3d in_target_;
  Eigen::Vector2d in_image_;
  rct_optimizations::Pose6d trans_to_first_camera_;
};

}

rct_optimizations::MultiCameraPnPResult rct_optimizations::optimize(const MultiCameraPnPProblem& params)
{
  using namespace rct_optimizations;

  // We optimize over a single pose guess; this represents the target in the FIRST camera
  // Each cost will understand its camera transfrom to the first camera
  Pose6d internal_camera_to_target = poseEigenToCal(params.camera_to_target_guess);

  ceres::Problem problem;

  for (std::size_t i = 0; i < params.correspondences.size(); ++i) // For each camera
  {
    // Look up the camera parameters for this camera
    const auto& camera_intr = params.intr[i];

    // Determine this camera's transfrom to get back to FIRST camera
    Pose6d trans_to_first_camera;
    if (i == 0) trans_to_first_camera = poseEigenToCal(Eigen::Affine3d::Identity()); // This IS first camera
    else trans_to_first_camera = poseEigenToCal(params.camera_transforms[i-1].inverse());

    for (std::size_t j = 0; j < params.correspondences[i].size(); ++j) // For each 3D point seen in the 2D image
    {
      const auto& point_in_image = params.correspondences[i][j].in_image;
      const auto& point_in_target = params.correspondences[i][j].in_target;

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new SolvePnPCostFunc(camera_intr, point_in_target, point_in_image, trans_to_first_camera);
      auto* cost_block = new ceres::AutoDiffCostFunction<SolvePnPCostFunc, 2, 6>(cost_fn);
      problem.AddResidualBlock(cost_block, NULL, internal_camera_to_target.values.data());
    }
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  MultiCameraPnPResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  result.camera_to_target = poseCalToEigen(internal_camera_to_target);

  return result;
}
