#include "rct_optimizations/extrinsic_hand_eye.h"

#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/eigen_conversions.h"
#include "rct_optimizations/types.h"

#include <ceres/ceres.h>
#include <iostream>

using namespace rct_optimizations;

namespace
{
template<Eigen::Index OBS_DIMENSION>
class ObservationCost
{
public:
  /**
    * @brief A Ceres cost function class that represents a single observation of a 3D camera and target
    * @param obs - The observation of a feature in the camera frame
    * @param camera_mount_to_base - The transform from the camera "mount" frame to the common base frame
    * @param base_to_target_mount - The transform from the common base frame to the target "mount" frame
    * @param point_in_target - The corresponding feature in the target frame
    */
  ObservationCost(const Eigen::Matrix<double, OBS_DIMENSION, 1> &obs,
                  const Eigen::Isometry3d &camera_mount_to_base,
                  const Eigen::Isometry3d &base_to_target_mount,
                  const Eigen::Vector3d &point_in_target)
    : obs_(obs)
    , camera_mount_to_base_(poseEigenToCal(camera_mount_to_base))
    , base_to_target_mount_(poseEigenToCal(base_to_target_mount))
    , target_pt_(point_in_target)
  {
  }

  template<typename T>
  bool operator()(const T * pose_camera_to_camera_mount,
                  const T * pose_target_mount_to_target,
                  T * residual) const;

  template<typename T>
  void getTargetPointInCamera(const T *pose_camera_to_camera_mount,
                              const T *pose_target_mount_to_target,
                              T* camera_point) const
  {
    const T *camera_angle_axis = pose_camera_to_camera_mount + 0;
    const T *camera_position = pose_camera_to_camera_mount + 3;

    const T *target_angle_axis = pose_target_mount_to_target + 0;
    const T *target_position = pose_target_mount_to_target + 3;

    T target_mount_point[3]; // Point in target mount coordinates
    T world_point[3]; // Point in world coordinates
    T camera_mount_point[3]; // Point in camera mount coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));
    transformPoint(target_angle_axis, target_position, target_pt, target_mount_point);
    poseTransformPoint(base_to_target_mount_, target_mount_point, world_point);
    poseTransformPoint(camera_mount_to_base_, world_point, camera_mount_point);
    transformPoint(camera_angle_axis, camera_position, camera_mount_point, camera_point);
  }

protected:
  Eigen::Matrix<double, OBS_DIMENSION, 1> obs_;
  Pose6d camera_mount_to_base_;
  Pose6d base_to_target_mount_;
  Eigen::Vector3d target_pt_;
};

/**
 * @brief The ObservationCost2D3D class
 */
class ObservationCost2D3D : public ObservationCost<2>
{
public:
  ObservationCost2D3D(const Eigen::Vector2d &obs,
                      const Eigen::Isometry3d &camera_mount_to_base,
                      const Eigen::Isometry3d &base_to_target_mount,
                      const Eigen::Vector3d &point_in_target,
                      const CameraIntrinsics &intr)
    : ObservationCost(obs, camera_mount_to_base, base_to_target_mount, point_in_target)
    , intr_(intr)
  {
  }

  template<typename T>
  bool operator()(const T *pose_camera_to_camera_mount,
                  const T *pose_target_mount_to_target,
                  T *residual) const
  {
    // Get the point in the frame of the camera
    T camera_point[3];
    getTargetPointInCamera(pose_camera_to_camera_mount, pose_target_mount_to_target, camera_point);

    // Project the point into the image
    T xy_image[2];
    projectPoint(intr_, camera_point, xy_image);

    // Calculate the residual error
    residual[0] = xy_image[0] - obs_.x();
    residual[1] = xy_image[1] - obs_.y();

    return true;
  }

private:
  CameraIntrinsics intr_;
};

/**
 * @brief The ObservationCost3D3D class
 */
class ObservationCost3D3D : public ObservationCost<3>
{
public:
  using ObservationCost<3>::ObservationCost;

  template<typename T>
  bool operator()(const T *pose_camera_to_camera_mount,
                  const T *pose_target_mount_to_target,
                  T *residual) const
  {
    // Get the target point in the frame of the camera
    T camera_point[3];
    getTargetPointInCamera(pose_camera_to_camera_mount, pose_target_mount_to_target, camera_point);

    // Calculate the residual error
    residual[0] = camera_point[0] - obs_.x();
    residual[1] = camera_point[1] - obs_.y();
    residual[2] = camera_point[2] - obs_.z();

    return true;
  }
};

bool isPointVisible(const Pose6d &camera_to_camera_mount,
                    const Pose6d &target_mount_to_target,
                    const ObservationCost2D3D *cost_fn)
{
  const double *pose_camera_to_camera_mount = camera_to_camera_mount.values.data();
  const double* pose_target_mount_to_target = target_mount_to_target.values.data();

  double camera_point[3];
  cost_fn->getTargetPointInCamera(pose_camera_to_camera_mount,
                                  pose_target_mount_to_target,
                                  camera_point);

  // Return whether or not the projected point's Z value is greater than zero
  return camera_point[2] > 0.0;
}

} // namespace anonymous

namespace rct_optimizations
{
ExtrinsicHandEyeResult optimize(const ExtrinsicHandEyeProblem2D3D& params)
{
  Pose6d internal_base_to_target = poseEigenToCal(params.target_mount_to_target_guess);
  Pose6d internal_camera_to_wrist = poseEigenToCal(params.camera_mount_to_camera_guess.inverse());

  ceres::Problem problem;

  for (const auto &observation : params.observations)
  {
    for (const auto& correspondence : observation.correspondence_set)
    {
      // Define
      const auto& img_obs = correspondence.in_image;
      const auto& point_in_target = correspondence.in_target;
      const auto wrist_to_base = observation.to_camera_mount.inverse();

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto *cost_fn = new ObservationCost2D3D(img_obs,
                                              wrist_to_base,
                                              observation.to_target_mount,
                                              point_in_target,
                                              params.intr);

      auto *cost_block = new ceres::AutoDiffCostFunction<ObservationCost2D3D, 2, 6, 6>(cost_fn);

      // Check that the target feature in camera coordinates is visible by the camera
      // Target features that project behind the camera tend to prevent the optimization from converging
      if (!isPointVisible(internal_camera_to_wrist, internal_base_to_target, cost_fn))
      {
        throw std::runtime_error(
          "Projected target feature lies behind the image plane using the "
          "current target mount and camera mount transform guesses. Try updating the initial "
          "transform guesses to more accurately represent the problem");
      }

      problem.AddResidualBlock(cost_block, NULL, internal_camera_to_wrist.values.data(),
                               internal_base_to_target.values.data());
    }
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 150;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicHandEyeResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.target_mount_to_target = poseCalToEigen(internal_base_to_target);
  result.camera_mount_to_camera = poseCalToEigen(internal_camera_to_wrist).inverse();
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  return result;

}

ExtrinsicHandEyeResult optimize(const ExtrinsicHandEyeProblem3D3D& params)
{
  Pose6d internal_base_to_target = poseEigenToCal(params.target_mount_to_target_guess);
  Pose6d internal_camera_to_wrist = poseEigenToCal(params.camera_mount_to_camera_guess.inverse());

  ceres::Problem problem;

  for (const auto &observation : params.observations)
  {
    for (const auto& correspondence : observation.correspondence_set)
    {
      // Define
      const auto& img_obs = correspondence.in_image;
      const auto& point_in_target = correspondence.in_target;
      const auto wrist_to_base = observation.to_camera_mount.inverse();

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new ObservationCost3D3D(img_obs, wrist_to_base, observation.to_target_mount, point_in_target);

      auto* cost_block = new ceres::AutoDiffCostFunction<ObservationCost3D3D, 3, 6, 6>(cost_fn);

      problem.AddResidualBlock(cost_block, NULL, internal_camera_to_wrist.values.data(),
                               internal_base_to_target.values.data());
    }
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  ExtrinsicHandEyeResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.target_mount_to_target = poseCalToEigen(internal_base_to_target);
  result.camera_mount_to_camera = poseCalToEigen(internal_camera_to_wrist).inverse();
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  return result;
}
} // namespace rct_optimizations
