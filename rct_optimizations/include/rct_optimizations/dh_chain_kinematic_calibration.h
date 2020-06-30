#pragma once

#include <rct_optimizations/types.h>
#include <rct_optimizations/dh_chain.h>
#include <rct_optimizations/ceres_math_utilities.h>

namespace rct_optimizations
{
struct KinematicCalibrationProblem2D3D
{
  KinematicCalibrationProblem2D3D(DHChain camera_chain_, DHChain target_chain_)
    : camera_chain(std::move(camera_chain_))
    , target_chain(std::move(target_chain_))
    , camera_mount_to_camera_guess(Eigen::Isometry3d::Identity())
    , target_mount_to_target_guess(Eigen::Isometry3d::Identity())
    , camera_base_to_target_base_guess(Eigen::Isometry3d::Identity())
  {
  }

  DHChain camera_chain;
  DHChain target_chain;
  KinObservation2D3D::Set observations;
  CameraIntrinsics intr;
  Eigen::Isometry3d camera_mount_to_camera_guess;
  Eigen::Isometry3d target_mount_to_target_guess;
  Eigen::Isometry3d camera_base_to_target_base_guess;
};

struct KinematicCalibrationResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;
  Eigen::Isometry3d camera_mount_to_camera;
  Eigen::Isometry3d target_mount_to_target;
  Eigen::Isometry3d camera_base_to_target_base;
  Eigen::MatrixX4d camera_chain_dh_offsets;
  Eigen::MatrixX4d target_chain_dh_offsets;

  // TODO: Add covariance matrix/matrices
//  Eigen::MatrixXd covariance;
};

class DualDHChainCost2D3D
{
  public:
  DualDHChainCost2D3D(const Eigen::Vector2d &obs,
       const Eigen::Vector3d &point_in_target,
       const CameraIntrinsics &intr,
       const DHChain &camera_chain,
       const DHChain &target_chain,
       const Eigen::VectorXd &camera_chain_joints,
       const Eigen::VectorXd &target_chain_joints)
    : obs_(obs)
    , target_pt_(point_in_target)
    , intr_(intr)
    , camera_chain_(camera_chain)
    , target_chain_(target_chain)
    , camera_chain_joints_(camera_chain_joints)
    , target_chain_joints_(target_chain_joints)
  {
  }

  template<typename T>
  Isometry3<T> createTransform(T const *const *params, const std::size_t idx) const
  {
    Eigen::Map<const Vector3<T>> t(params[idx]);
    Eigen::Map<const Vector3<T>> aa(params[idx + 1]);

    Isometry3<T> result = Isometry3<T>::Identity() * Eigen::Translation<T, 3>(t);

    T aa_norm = aa.norm();
    if (aa_norm > std::numeric_limits<T>::epsilon())
    {
      result *= Eigen::AngleAxis<T>(aa_norm, aa.normalized());
    }

    return result;
  }

  static std::vector<double *> constructParameters(Eigen::MatrixX4d &camera_chain_dh_offsets,
                                                   Eigen::MatrixX4d &target_chain_dh_offsets,
                                                   Eigen::Vector3d &t_cm_to_c,
                                                   Eigen::Vector3d &aa_cm_to_c,
                                                   Eigen::Vector3d &t_tm_to_t,
                                                   Eigen::Vector3d &aa_tm_to_t,
                                                   Eigen::Vector3d &t_ccb_to_tcb,
                                                   Eigen::Vector3d &aa_ccb_to_tcb)
  {
    std::vector<double *> parameters;
    parameters.push_back(camera_chain_dh_offsets.data());
    parameters.push_back(target_chain_dh_offsets.data());
    parameters.push_back(t_cm_to_c.data());
    parameters.push_back(aa_cm_to_c.data());
    parameters.push_back(t_tm_to_t.data());
    parameters.push_back(aa_tm_to_t.data());
    parameters.push_back(t_ccb_to_tcb.data());
    parameters.push_back(aa_ccb_to_tcb.data());
    return parameters;
  }

  template<typename T>
  bool operator()(T const *const *parameters, T *residual) const
  {
    // Step 1: Load the data
    // The first parameter is a pointer to the DH parameter offsets of the camera kinematic chain
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 4>> camera_chain_dh_offsets(parameters[0], camera_chain_.dof(), 4);

    // The next parameter is a pointer to the DH parameter offsets of the target kinematic chain
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 4>> target_chain_dh_offsets(parameters[1], target_chain_.dof(), 4);

    // The next two parameters are pointers to the quaternion and translation of the transform from the camera mount to the camera
    std::size_t cm_to_c_idx = 2;
    const Isometry3<T> camera_mount_to_camera = createTransform(parameters, cm_to_c_idx);

    // The next two parameters are pointers to the quaternion and translation of the transform from the target mount to the target
    std::size_t tm_to_t_idx = cm_to_c_idx + 2;
    const Isometry3<T> target_mount_to_target = createTransform(parameters, tm_to_t_idx);

    // The next two parameters are pointers to the quaternion and translation of the transform from chain 1 to chain 2
    std::size_t cb_to_tb_idx = tm_to_t_idx + 2;
    const Isometry3<T> camera_base_to_target_base = createTransform(parameters, cb_to_tb_idx);

    // Step 2: Transformation math
    // Build the transforms from the camera chain base out to the camera
    Isometry3<T> camera_chain_fk = camera_chain_.getFK<T>(camera_chain_joints_.cast<T>(),
                                                          camera_chain_dh_offsets);
    Isometry3<T> camera_base_to_camera = camera_chain_fk * camera_mount_to_camera;

    // Build the transforms from the camera chain base out to the target
    Isometry3<T> target_chain_fk = target_chain_.getFK<T>(target_chain_joints_.cast<T>(),
                                                          target_chain_dh_offsets);
    Isometry3<T> camera_base_to_target = camera_base_to_target_base * target_chain_fk
                                         * target_mount_to_target;

    // Now that we have two transforms in the same frame, get the target point in the camera frame
    Isometry3<T> camera_to_target = camera_base_to_camera.inverse() * camera_base_to_target;
    Vector3<T> target_in_camera = camera_to_target * target_pt_.cast<T>();

    // Project the target into the image plane
    Vector2<T> target_in_image = projectPoint(intr_, target_in_camera);

    // Step 3: Calculate the error
    residual[0] = target_in_image.x() - obs_.x();
    residual[1] = target_in_image.y() - obs_.y();

    return true;
  }

  protected:
  Eigen::Vector2d obs_;
  Eigen::Vector3d target_pt_;
  CameraIntrinsics intr_;

  const DHChain &camera_chain_;
  const DHChain &target_chain_;

  Eigen::VectorXd camera_chain_joints_;
  Eigen::VectorXd target_chain_joints_;
};

KinematicCalibrationResult optimize(const KinematicCalibrationProblem2D3D &problem);

} // namespace rct_optimizations

