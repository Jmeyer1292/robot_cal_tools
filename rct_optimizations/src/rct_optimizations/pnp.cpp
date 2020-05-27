#include "rct_optimizations/pnp.h"
#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/local_parameterization.h"

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
  bool operator()(const T *const target_q, const T *const target_t, T *const residual) const
  {
    using Isometry3 = Eigen::Transform<T, 3, Eigen::Isometry>;
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Vector2 = Eigen::Matrix<T, 2, 1>;

    Eigen::Map<const Eigen::Quaternion<T>> q(target_q);
    Eigen::Map<const Vector3> t(target_t);
    Isometry3 camera_to_target = Isometry3::Identity();
    camera_to_target = Eigen::Translation<T, 3>(t) * q;

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
  SolvePnPCostFunc3D(const rct_optimizations::CameraIntrinsics& intr, const Eigen::Vector3d& pt_in_target,
                     const Eigen::Vector3d& pt_in_image)
    : intr_(intr), in_target_(pt_in_target), in_image_(pt_in_image)
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

    T camera_point[3];  // Point in camera coordinates
    rct_optimizations::transformPoint(target_angle_axis, target_position, target_pt, camera_point);

    T xyz_image[3];
    //rct_optimizations::projectPoint(intr_, camera_point, xyz_image); //Does this really need to go?

    residual[0] = xyz_image[0] - in_image_.x();
    residual[1] = xyz_image[1] - in_image_.y();
    residual[2] = xyz_image[2] - in_image_.z();
    return true;
  }

  rct_optimizations::CameraIntrinsics intr_;
  Eigen::Vector3d in_target_;
  Eigen::Vector3d in_image_;
};

} // namespace anonymous

namespace rct_optimizations
{

PnPResult optimize(const PnPProblem &params)
{
  // Create the optimization variables from the input guess
  Eigen::Quaterniond q(params.camera_to_target_guess.rotation());
  Eigen::Vector3d t(params.camera_to_target_guess.translation());

  ceres::Problem problem;

  // For each 3D point seen in the 2D image
  for (std::size_t i = 0; i < params.correspondences.size(); ++i)
  {
    // Define
    const auto &img_obs = params.correspondences[i].in_image;
    const auto &point_in_target = params.correspondences[i].in_target;

    // Allocate Ceres data structures - ownership is taken by the ceres
    // Problem data structure
    auto *cost_fn = new SolvePnPCostFunc(params.intr, point_in_target, img_obs);

    auto *cost_block = new ceres::AutoDiffCostFunction<SolvePnPCostFunc, 2, 4, 3>(cost_fn);

    problem.AddResidualBlock(cost_block, nullptr, q.coeffs().data(), t.data());
  }

  ceres::LocalParameterization *q_param
    = new ceres::AutoDiffLocalParameterization<EigenQuaternionPlus, 4, 3>();
  problem.SetParameterization(q.coeffs().data(), q_param);

  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  ceres::Solve(options, &problem, &summary);

  PnPResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  result.camera_to_target = Eigen::Translation3d(t) * q;

  return result;
}

PnPResult optimize(const rct_optimizations::PnPProblem3D& params)
{
  // Create the optimization variables from the input guess
  Eigen::Quaterniond q(params.camera_to_target_guess.rotation());
  Eigen::Vector3d t(params.camera_to_target_guess.translation());

  ceres::Problem problem;

  //only one loop, for correspondences
  for (std::size_t i = 0; i < params.correspondences.size(); ++i) // For each 3D point seen in the 3D image
  {
    // Define
    const auto& img_obs = params.correspondences[i].in_image; //Why decalare when you can deduce!
    const auto& point_in_target = params.correspondences[i].in_target;

    // Allocate Ceres data structures - ownership is taken by the ceres
    // Problem data structure


    auto* cost_fn = new SolvePnPCostFunc3D(params.intr, point_in_target, img_obs);

    auto* cost_block = new ceres::AutoDiffCostFunction<SolvePnPCostFunc3D, 2, 6>(cost_fn);

    problem.AddResidualBlock(cost_block, nullptr, q.coeffs().data(), t.data());
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  PnPResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;
  result.camera_to_target = Eigen::Translation3d(t) * q;

  return result;
}
} // namespace rct_optimizations
