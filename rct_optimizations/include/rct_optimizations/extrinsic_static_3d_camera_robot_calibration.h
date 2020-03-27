#ifndef RCT_EXTRINSIC_STATIC_CAMERA_ROBOT_CALIBRATION_H
#define RCT_EXTRINSIC_STATIC_CAMERA_ROBOT_CALIBRATION_H

#include "rct_optimizations/types.h"
#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/eigen_conversions.h"

#include <ceres/ceres.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>

namespace rct_optimizations
{

using RobotChainCalculatorFn = std::function<std::vector<Eigen::Isometry3d>(const Eigen::Ref<const Eigen::VectorXd>&)>;

struct ExtrinsicStatic3DCameraRobotCalibrationProblem
{
  /**
   * @brief Ceres Solver Options
   */
  ceres::Solver::Options options;

  /** @brief The robot joint positions at which each observation set was taken. The rows of the matrix should match
   * the vector of @e image_observations in size.
   */
  Eigen::MatrixXd joint_poses;

  /**
   * @brief This function must return the relative transformation to compute the forward kinematics. Example it returns
   * a vector of transforms {A, B, C, D, E, F} where the forwad kin is calculated as FwdKin = A * B * C * D * E * F.
   */
  RobotChainCalculatorFn robot_calculator;

  /** @brief A sequence of observation sets corresponding to the image locations in @e wrist_poses.
   * Each observation set consists of a set of correspodences: a 3D position (e.g. a dot) in "target
   * frame" to the image location it was detected at (2D). The outer-most vector is for each camera,
   * the inner vector is the images valid for that camera.
   */
  std::vector<Correspondence3DSet> image_observations;

  /** @brief Your best guess at the "wrist frame" to "target frame" transform */
  Eigen::Isometry3d wrist_to_target_guess;

  /** @brief Your best guess at the "base frame" to "camera frame" transform; one for each camera */
  Eigen::Isometry3d base_to_camera_guess;
};

struct ExtrinsicStatic3DCameraRobotCalibrationResult
{
  /**
   * @brief Whether the underlying solver converged. If this is false, your calibration did not go well.
   * If this is true, your calibration MAY have gone well.
   */
  bool converged;

  /**
   * @brief Store ceres solver summary. Print using summary.FullReport()
   */
  ceres::Solver::Summary summary;

  /**
   * @brief The initial reprojection error (in pixels) per residual based on your input guesses.
   */
  double initial_cost_per_obs;

  /**
   * @brief The final reprojection error (in pixels) per residual after optimization. Note that each circle
   * has two residuals: a U and V error in the image. So a value of 1.2 means that each circle was described
   * to within 1.2 pixels in X and 1.2 pixels in Y.
   *
   * A low value here is encouraging if you had a diversity of images. If you took few images, you can get
   * a low score without getting a calibration that describes your workcell.
   */
  double final_cost_per_obs;

  /**
   * @brief The final calibrated result of "wrist frame" to "target frame".
   */
  Eigen::Isometry3d wrist_to_target;

  /**
   * @brief The final calibrated result of "base frame" to "camera optical frame".
   */
  Eigen::Isometry3d base_to_camera;

  /**
   * @brief The delta correction transform to be applied to each joint transform
   */
  std::vector<Eigen::Isometry3d> robot_joint_delta;
};

namespace detail
{

class ObservationCost
{
public:
  ObservationCost(const Eigen::Vector3d& obs,
                  const Eigen::Isometry3d& base_to_camera,
                  const std::vector<Eigen::Isometry3d>& robot_chain_transforms,
                  const Eigen::Vector3d& point_in_target)
    : obs_(obs)
    , camera_to_base_(poseEigenToCal(base_to_camera.inverse()))
    , target_pt_(point_in_target)
  {
    robot_chain_poses_.reserve(robot_chain_transforms.size());
    for (const auto& t : robot_chain_transforms)
      robot_chain_poses_.push_back(poseEigenToCal(t));
  }

  template <typename T>
  bool operator() (const T* pose_wrist_to_target, const T* robot_pose_deltas, T* residual) const
  {
    const T* target_angle_axis = pose_wrist_to_target + 0;
    const T* target_position = pose_wrist_to_target + 3;

    T link_point[3]; // Point in wrist coordinates
    T world_point[3]; // Point in world coordinates (base of robot)
    T camera_point[3]; // Point in camera coordinates

    T camera_angle_axis[3];
    camera_angle_axis[0] = T(camera_to_base_.rx());
    camera_angle_axis[1] = T(camera_to_base_.ry());
    camera_angle_axis[2] = T(camera_to_base_.rz());

    T camera_position[3];
    camera_position[0] = T(camera_to_base_.x());
    camera_position[1] = T(camera_to_base_.y());
    camera_position[2] = T(camera_to_base_.z());

    T wrist_angle_axis[3];
    wrist_angle_axis[0] = robot_pose_deltas[0];
    wrist_angle_axis[1] = robot_pose_deltas[1];
    wrist_angle_axis[2] = robot_pose_deltas[2];

    T wrist_position[3];
    wrist_position[0] = robot_pose_deltas[3];
    wrist_position[1] = robot_pose_deltas[4];
    wrist_position[2] = robot_pose_deltas[5];

    T inner_angle_axis[3];
    T inner_position[3];
    std::size_t num_joints = robot_chain_poses_.size();

    for (std::size_t i = 0; i < (num_joints - 1); ++i)
    {
      transformPose6d(wrist_angle_axis, wrist_position, robot_chain_poses_[i], inner_angle_axis, inner_position);

      wrist_angle_axis[0] = inner_angle_axis[0];
      wrist_angle_axis[1] = inner_angle_axis[1];
      wrist_angle_axis[2] = inner_angle_axis[2];
      wrist_position[0] = inner_position[0];
      wrist_position[1] = inner_position[1];
      wrist_position[2] = inner_position[2];

      const T* joint_angle_axis = robot_pose_deltas + ((i+1)*num_joints);
      const T* joint_position = robot_pose_deltas + ((i+1)*num_joints) + 3;

      transformPose6d(wrist_angle_axis, wrist_position, joint_angle_axis, joint_position, inner_angle_axis, inner_position);

      wrist_angle_axis[0] = inner_angle_axis[0];
      wrist_angle_axis[1] = inner_angle_axis[1];
      wrist_angle_axis[2] = inner_angle_axis[2];
      wrist_position[0] = inner_position[0];
      wrist_position[1] = inner_position[1];
      wrist_position[2] = inner_position[2];
    }

    transformPose6d(wrist_angle_axis, wrist_position, robot_chain_poses_[num_joints - 1], inner_angle_axis, inner_position);

    wrist_angle_axis[0] = inner_angle_axis[0];
    wrist_angle_axis[1] = inner_angle_axis[1];
    wrist_angle_axis[2] = inner_angle_axis[2];
    wrist_position[0] = inner_position[0];
    wrist_position[1] = inner_position[1];
    wrist_position[2] = inner_position[2];

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    transformPoint(target_angle_axis, target_position, target_pt, link_point);
    transformPoint(wrist_angle_axis, wrist_position, link_point, world_point);
    transformPoint(camera_angle_axis, camera_position, world_point, camera_point);

    residual[0] = camera_point[0] - obs_.x();
    residual[1] = camera_point[1] - obs_.y();
    residual[2] = camera_point[2] - obs_.z();

    return true;
  }

private:
  Eigen::Vector3d obs_;
  Pose6d camera_to_base_;
  std::vector<Pose6d> robot_chain_poses_;
  Eigen::Vector3d target_pt_;
};

class ObservationCost2
{
public:
  ObservationCost2(const Eigen::Vector3d& obs,
                   const std::vector<Eigen::Isometry3d>& robot_chain_transforms,
                   const Eigen::Vector3d& point_in_target)
    : obs_(obs)
    , target_pt_(point_in_target)
  {
    robot_chain_poses_.reserve(robot_chain_transforms.size());
    for (const auto& t : robot_chain_transforms)
      robot_chain_poses_.push_back(poseEigenToCal(t));
  }

  template <typename T>
  bool operator() (const T* pose_camera_to_base, const T* pose_wrist_to_target, const T* robot_pose_deltas, T* residual) const
  {
    const T* camera_angle_axis = pose_camera_to_base + 0;
    const T* camera_position = pose_camera_to_base + 3;

    const T* target_angle_axis = pose_wrist_to_target + 0;
    const T* target_position = pose_wrist_to_target + 3;

    T link_point[3]; // Point in wrist coordinates
    T world_point[3]; // Point in world coordinates (base of robot)
    T camera_point[3]; // Point in camera coordinates

    T wrist_angle_axis[3];
    wrist_angle_axis[0] = T(0); //robot_pose_deltas[0];
    wrist_angle_axis[1] = T(0); //robot_pose_deltas[1];
    wrist_angle_axis[2] = T(0); //robot_pose_deltas[2];

    T wrist_position[3];
    wrist_position[0] = T(0); //robot_pose_deltas[3];
    wrist_position[1] = T(0); //robot_pose_deltas[4];
    wrist_position[2] = T(0); //robot_pose_deltas[5];

    T inner_angle_axis[3];
    T inner_position[3];
    std::size_t num_joints = robot_chain_poses_.size();
    // An error matrix is not added to the last joint because it would compete with the target error matrix
    for (std::size_t i = 0; i < (num_joints - 1); ++i)
    {
      transformPose6d(wrist_angle_axis, wrist_position, robot_chain_poses_[i], inner_angle_axis, inner_position);

      wrist_angle_axis[0] = inner_angle_axis[0];
      wrist_angle_axis[1] = inner_angle_axis[1];
      wrist_angle_axis[2] = inner_angle_axis[2];
      wrist_position[0] = inner_position[0];
      wrist_position[1] = inner_position[1];
      wrist_position[2] = inner_position[2];

//      const T* joint_angle_axis = robot_pose_deltas + ((i+1)*num_joints);
//      const T* joint_position = robot_pose_deltas + ((i+1)*num_joints) + 3;
      const T* joint_angle_axis = robot_pose_deltas + (i * num_joints);
      const T* joint_position = robot_pose_deltas + (i * num_joints) + 3;

      transformPose6d(wrist_angle_axis, wrist_position, joint_angle_axis, joint_position, inner_angle_axis, inner_position);

      wrist_angle_axis[0] = inner_angle_axis[0];
      wrist_angle_axis[1] = inner_angle_axis[1];
      wrist_angle_axis[2] = inner_angle_axis[2];
      wrist_position[0] = inner_position[0];
      wrist_position[1] = inner_position[1];
      wrist_position[2] = inner_position[2];
    }

    transformPose6d(wrist_angle_axis, wrist_position, robot_chain_poses_[num_joints - 1], inner_angle_axis, inner_position);

    wrist_angle_axis[0] = inner_angle_axis[0];
    wrist_angle_axis[1] = inner_angle_axis[1];
    wrist_angle_axis[2] = inner_angle_axis[2];
    wrist_position[0] = inner_position[0];
    wrist_position[1] = inner_position[1];
    wrist_position[2] = inner_position[2];

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    transformPoint(target_angle_axis, target_position, target_pt, link_point);
    transformPoint(wrist_angle_axis, wrist_position, link_point, world_point);
    transformPoint(camera_angle_axis, camera_position, world_point, camera_point);

    residual[0] = camera_point[0] - obs_.x();
    residual[1] = camera_point[1] - obs_.y();
    residual[2] = camera_point[2] - obs_.z();

    return true;
  }

private:
  Eigen::Vector3d obs_;
  std::vector<Pose6d> robot_chain_poses_;
  Eigen::Vector3d target_pt_;
};

}

template <int num_joints>
ExtrinsicStatic3DCameraRobotCalibrationResult optimize(const ExtrinsicStatic3DCameraRobotCalibrationProblem& params)
{
  assert(num_joints == params.joint_poses.cols());
  Pose6d internal_wrist_to_target = poseEigenToCal(params.wrist_to_target_guess);

  // An error matrix is not added to the last joint because it would compete with the target error matrix
  std::vector<double> robot_pose_deltas(6 * num_joints);

  ceres::Problem problem;

  assert(params.image_observations.size() == params.joint_poses.rows());
  for (std::size_t i = 0; i < params.joint_poses.rows(); ++i) // For each joint pose / image set
  {
    std::vector<Eigen::Isometry3d> robot_chain_transforms = params.robot_calculator(params.joint_poses.row(i));

    for (std::size_t j = 0; j < params.image_observations[i].size(); ++j) // For each 3D point seen in the 2D image
    {
      // Define
      const auto& img_obs = params.image_observations[i][j].in_image;
      const auto& point_in_target = params.image_observations[i][j].in_target;

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new detail::ObservationCost(img_obs,
                                                  params.base_to_camera_guess,
                                                  robot_chain_transforms,
                                                  point_in_target);

      // An error matrix is not added to the last joint because it would compete with the target error matrix
      auto* cost_block = new ceres::AutoDiffCostFunction<detail::ObservationCost, 3, 6, 6 * num_joints>(cost_fn);

      problem.AddResidualBlock(cost_block,
                               NULL,
                               internal_wrist_to_target.values.data(),
                               robot_pose_deltas.data());
    }
  } // for each wrist pose

  ExtrinsicStatic3DCameraRobotCalibrationResult result;
  ceres::Solve(params.options, &problem, &result.summary);

  result.robot_joint_delta.resize(num_joints);
  result.converged = result.summary.termination_type == ceres::CONVERGENCE;

  for (std::size_t i = 0; i < num_joints; ++i)
  {
    Pose6d p;
    std::copy(robot_pose_deltas.begin() + (i * 6), robot_pose_deltas.begin() + (i * 6) + 6, std::begin(p.values));
    result.robot_joint_delta[i] = poseCalToEigen(p);
  }

  result.base_to_camera = result.robot_joint_delta[0].inverse() * params.base_to_camera_guess;
  result.wrist_to_target = poseCalToEigen(internal_wrist_to_target);
  result.initial_cost_per_obs = result.summary.initial_cost / result.summary.num_residuals;
  result.final_cost_per_obs = result.summary.final_cost / result.summary.num_residuals;
  return result;
}

template <int num_joints>
ExtrinsicStatic3DCameraRobotCalibrationResult optimize2(const ExtrinsicStatic3DCameraRobotCalibrationProblem& params)
{
  assert(num_joints == params.joint_poses.cols());
  Pose6d internal_wrist_to_target = poseEigenToCal(params.wrist_to_target_guess);
  Pose6d internal_camera_to_base = poseEigenToCal(params.base_to_camera_guess.inverse());

  // An error matrix is not added to the last joint because it would compete with the target error matrix
  std::vector<double> robot_pose_deltas(6 * (num_joints-1));

  ceres::Problem problem;

  assert(params.image_observations.size() == params.joint_poses.rows());
  for (std::size_t i = 0; i < params.joint_poses.rows(); ++i) // For each joint pose / image set
  {
    std::vector<Eigen::Isometry3d> robot_chain_transforms = params.robot_calculator(params.joint_poses.row(i));

    for (std::size_t j = 0; j < params.image_observations[i].size(); ++j) // For each 3D point seen in the 2D image
    {
      // Define
      const auto& img_obs = params.image_observations[i][j].in_image;
      const auto& point_in_target = params.image_observations[i][j].in_target;

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new detail::ObservationCost2(img_obs,
                                                   robot_chain_transforms,
                                                   point_in_target);

      // An error matrix is not added to the last joint because it would compete with the target error matrix
      auto* cost_block = new ceres::AutoDiffCostFunction<detail::ObservationCost2, 3, 6, 6, 6 * (num_joints-1)>(cost_fn);

      problem.AddResidualBlock(cost_block,
                               NULL,
                               internal_camera_to_base.values.data(),
                               internal_wrist_to_target.values.data(),
                               robot_pose_deltas.data());
    }
  } // for each wrist pose

  ExtrinsicStatic3DCameraRobotCalibrationResult result;
  ceres::Solve(params.options, &problem, &result.summary);

  result.robot_joint_delta.resize((num_joints-1));
  result.converged = result.summary.termination_type == ceres::CONVERGENCE;

  for (std::size_t i = 0; i < (num_joints-1); ++i)
  {
    Pose6d p;
    std::copy(robot_pose_deltas.begin() + (i * 6), robot_pose_deltas.begin() + (i * 6) + 6, std::begin(p.values));
    result.robot_joint_delta[i] = poseCalToEigen(p);
  }

  result.base_to_camera = poseCalToEigen(internal_camera_to_base).inverse();
  result.wrist_to_target = poseCalToEigen(internal_wrist_to_target);
  result.initial_cost_per_obs = result.summary.initial_cost / result.summary.num_residuals;
  result.final_cost_per_obs = result.summary.final_cost / result.summary.num_residuals;
  return result;
}

}
#endif // RCT_EXTRINSIC_STATIC_CAMERA_ROBOT_CALIBRATION_H
