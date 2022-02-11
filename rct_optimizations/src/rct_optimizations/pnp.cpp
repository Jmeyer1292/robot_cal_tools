#include "rct_optimizations/pnp.h"
#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/covariance_analysis.h"
#include <ceres/ceres.h>

namespace
{
struct SolvePnPCostFunc
{
  public:
  SolvePnPCostFunc(const rct_optimizations::CameraIntrinsics& intr,
                   const Eigen::Vector3d& pt_in_target,
                   const Eigen::Vector2d& pt_in_image)
    : intr_(intr), in_target_(pt_in_target), in_image_(pt_in_image)
  {
  }

  template<typename T>
  bool operator()(const T *const cam_to_tgt_angle_axis_ptr, const T *const cam_to_tgt_translation_ptr, T *const residual) const
  {
    using Isometry3 = Eigen::Transform<T, 3, Eigen::Isometry>;
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector2 = Eigen::Matrix<T, 2, 1>;

    Eigen::Map<const Vector3> cam_to_tgt_angle_axis(cam_to_tgt_angle_axis_ptr);
    Eigen::Map<const Vector3> cam_to_tgt_translation(cam_to_tgt_translation_ptr);
    Isometry3 camera_to_target = Isometry3::Identity();
    camera_to_target = Eigen::Translation<T, 3>(cam_to_tgt_translation)
                       * Eigen::AngleAxis<T>(cam_to_tgt_angle_axis.norm(),
                                             cam_to_tgt_angle_axis.normalized());

    // Transform points into camera coordinates
    Vector3 camera_pt = camera_to_target * in_target_.cast<T>();

    Vector2 xy_image = projectPoint(intr_, camera_pt);

    residual[0] = xy_image[0] - in_image_.x();
    residual[1] = xy_image[1] - in_image_.y();

    return true;
  }

  rct_optimizations::CameraIntrinsics intr_;
  Eigen::Vector3d in_target_;
  Eigen::Vector2d in_image_;
};

struct SolvePnPCostFunc3D
{
public:
  SolvePnPCostFunc3D(const Eigen::Vector3d& pt_in_target,
                     const Eigen::Vector3d& pt_in_image)
    :in_target_(pt_in_target), in_image_(pt_in_image)
  {}

  template<typename T>
  bool operator()(const T *const cam_to_tgt_angle_axis_ptr, const T *const cam_to_tgt_translation_ptr, T *const residual) const
  {
    using Isometry3 = Eigen::Transform<T, 3, Eigen::Isometry>;
    using Vector3 = Eigen::Matrix<T, 3, 1>;

    Eigen::Map<const Vector3> cam_to_tgt_angle_axis(cam_to_tgt_angle_axis_ptr);
    Eigen::Map<const Vector3> cam_to_tgt_translation(cam_to_tgt_translation_ptr);
    Isometry3 camera_to_target = Isometry3::Identity();
    camera_to_target = Eigen::Translation<T, 3>(cam_to_tgt_translation)
                       * Eigen::AngleAxis<T>(cam_to_tgt_angle_axis.norm(),
                                             cam_to_tgt_angle_axis.normalized());

    // Transform points into camera coordinates
    Vector3 camera_pt = camera_to_target * in_target_.cast<T>();

    residual[0] = camera_pt[0] - in_image_.x();
    residual[1] = camera_pt[1] - in_image_.y();
    residual[2] = camera_pt[2] - in_image_.z();
    return true;
  }

  Eigen::Vector3d in_target_;
  Eigen::Vector3d in_image_;
};

} // namespace anonymous

namespace rct_optimizations
{

PnPResult optimize(const PnPProblem &params)
{
  // Create the optimization variables from the input guess
  Eigen::AngleAxisd cam_to_tgt_rotation(params.camera_to_target_guess.rotation());
  Eigen::Vector3d cam_to_tgt_angle_axis = cam_to_tgt_rotation.angle() * cam_to_tgt_rotation.axis();
  Eigen::Vector3d cam_to_tgt_translation(params.camera_to_target_guess.translation());

  ceres::Problem problem;

  // For each 3D point seen in the 2D image
  for (const auto& corr : params.correspondences)
  {
    // Allocate Ceres data structures - ownership is taken by the ceres
    // Problem data structure
    auto *cost_fn = new SolvePnPCostFunc(params.intr, corr.in_target, corr.in_image);

    auto *cost_block = new ceres::AutoDiffCostFunction<SolvePnPCostFunc, 2, 3, 3>(cost_fn);

    problem.AddResidualBlock(cost_block, nullptr, cam_to_tgt_angle_axis.data(), cam_to_tgt_translation.data());
  }

  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  ceres::Solve(options, &problem, &summary);

  PnPResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  result.camera_to_target = Eigen::Translation3d(cam_to_tgt_translation)
                            * Eigen::AngleAxisd(cam_to_tgt_angle_axis.norm(),
                                                cam_to_tgt_angle_axis.normalized());

  // compose labels "camera_to_target_x", etc.
  std::vector<std::string> labels_camera_to_target_guess_translation;
  for (auto label_t : params.labels_translation)
  {
    labels_camera_to_target_guess_translation.emplace_back(params.label_camera_to_target_guess + "_" + label_t);
  }

  // compose labels "camera_to_target_qx", etc.
  std::vector<std::string> labels_camera_to_target_guess_quaternion;
  for (auto label_r : params.labels_rotation)
  {
    labels_camera_to_target_guess_quaternion.emplace_back(params.label_camera_to_target_guess + "_" + label_r);
  }
  std::map<const double*, std::vector<std::string>> param_labels;
  param_labels[cam_to_tgt_translation.data()] = labels_camera_to_target_guess_translation;
  param_labels[cam_to_tgt_angle_axis.data()] = labels_camera_to_target_guess_quaternion;

  result.covariance = rct_optimizations::computeCovariance(problem,
                                                           std::vector<const double *>({cam_to_tgt_translation.data(), cam_to_tgt_angle_axis.data()}),
                                                           param_labels);

  return result;
}

PnPResult optimize(const rct_optimizations::PnPProblem3D& params)
{
  // Create the optimization variables from the input guess
  Eigen::AngleAxisd cam_to_tgt_rotation(params.camera_to_target_guess.rotation());
  Eigen::Vector3d cam_to_tgt_angle_axis(cam_to_tgt_rotation.angle() * cam_to_tgt_rotation.axis());
  Eigen::Vector3d cam_to_tgt_translation(params.camera_to_target_guess.translation());

  ceres::Problem problem;

  //only one loop, for correspondences
  for (const auto& corr : params.correspondences) // For each 3D point seen in the 3D image
  {
    // Allocate Ceres data structures - ownership is taken by the ceres
    // Problem data structure

    auto* cost_fn = new SolvePnPCostFunc3D(corr.in_target, corr.in_image);

    auto* cost_block = new ceres::AutoDiffCostFunction<SolvePnPCostFunc3D, 3, 3, 3>(cost_fn);

    problem.AddResidualBlock(cost_block, nullptr, cam_to_tgt_angle_axis.data(), cam_to_tgt_translation.data());
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  PnPResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  result.camera_to_target = Eigen::Translation3d(cam_to_tgt_translation)
                            * Eigen::AngleAxisd(cam_to_tgt_angle_axis.norm(),
                                                cam_to_tgt_angle_axis.normalized());

  // compose labels "camera_to_target_x", etc.
  std::vector<std::string> labels_camera_to_target_guess_translation;
  for (auto label_t : params.labels_translation)
  {
    labels_camera_to_target_guess_translation.emplace_back(params.label_camera_to_target_guess + "_" + label_t);
  }

  // compose labels "camera_to_target_qx", etc.
  std::vector<std::string> labels_camera_to_target_guess_quaternion;
  for (auto label_r : params.labels_rotation)
  {
    labels_camera_to_target_guess_quaternion.emplace_back(params.label_camera_to_target_guess + "_" + label_r);
  }
  std::map<const double*, std::vector<std::string>> param_labels;
  param_labels[cam_to_tgt_translation.data()] = labels_camera_to_target_guess_translation;
  param_labels[cam_to_tgt_angle_axis.data()] = labels_camera_to_target_guess_quaternion;

  result.covariance = rct_optimizations::computeCovariance(problem,
                                                           std::vector<const double *>({cam_to_tgt_translation.data(), cam_to_tgt_angle_axis.data()}),
                                                           param_labels);

  return result;
}
} // namespace rct_optimizations
