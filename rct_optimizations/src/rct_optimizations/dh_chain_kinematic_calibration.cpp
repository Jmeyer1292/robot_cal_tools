#include <rct_optimizations/dh_chain_kinematic_calibration.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/covariance_analysis.h>

#include <ceres/ceres.h>

namespace rct_optimizations
{

KinematicCalibrationResult optimize(const KinematicCalibrationProblem2D3D &params)
{
  // Initialize the optimization variables
  // Camera mount to camera (cm_to_c) quaternion and translation
  Eigen::Quaterniond q_cm_to_c(params.camera_mount_to_camera_guess.rotation());
  Eigen::Vector3d t_cm_to_c(params.camera_mount_to_camera_guess.translation());

  // Target mount to target (cm_to_c) quaternion and translation
  Eigen::Vector3d t_tm_to_t(params.target_mount_to_target_guess.translation());
  Eigen::Quaterniond q_tm_to_t(params.target_mount_to_target_guess.rotation());

  // Camera chain base to target_chain_base (ccb_to_tcb) quaternion and translation
  Eigen::Vector3d t_ccb_to_tcb(params.camera_base_to_target_base_guess.translation());
  Eigen::Quaterniond q_ccb_to_tcb(params.camera_base_to_target_base_guess.rotation());

  // Create containers for the kinematic chain DH offsets
  // Ceres will not work with parameter blocks of size zero, so create a dummy set of DH offsets for chains with DoF == 0
  Eigen::MatrixX4d camera_chain_dh_offsets;
  if (params.camera_chain.dof() != 0)
    camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.camera_chain.dof(), 4);
  else
    camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(1, 4);

  Eigen::MatrixX4d target_chain_dh_offsets;
  if (params.target_chain.dof() != 0)
    target_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.target_chain.dof(), 4);
  else
    target_chain_dh_offsets = Eigen::MatrixX4d::Zero(1, 4);

  // Create a vector of the pointers to the optimization variables in the order that the cost function expects them
  std::vector<double *> parameters;
  parameters.push_back(camera_chain_dh_offsets.data());
  parameters.push_back(target_chain_dh_offsets.data());
  parameters.push_back(t_cm_to_c.data());
  parameters.push_back(q_cm_to_c.coeffs().data());
  parameters.push_back(t_tm_to_t.data());
  parameters.push_back(q_tm_to_t.coeffs().data());
  parameters.push_back(t_ccb_to_tcb.data());
  parameters.push_back(q_ccb_to_tcb.coeffs().data());

  // Set up the problem
  ceres::Problem problem;

  for (const auto &observation : params.observations)
  {
    for (const auto &correspondence : observation.correspondence_set)
    {
      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto *cost_fn = new DualDHChainCost2D3D(correspondence.in_image,
                                              correspondence.in_target,
                                              params.intr,
                                              params.camera_chain,
                                              params.target_chain,
                                              observation.camera_chain_joints,
                                              observation.target_chain_joints);

      auto *cost_block = new ceres::DynamicAutoDiffCostFunction<DualDHChainCost2D3D, 4>(cost_fn);

      // Add the optimization parameters
      // DH parameters for both kinematic chains
      cost_block->AddParameterBlock(camera_chain_dh_offsets.size());
      cost_block->AddParameterBlock(target_chain_dh_offsets.size());

      // Camera mount to camera transform
      cost_block->AddParameterBlock(3);
      cost_block->AddParameterBlock(4);

      // Target mount to target transform
      cost_block->AddParameterBlock(3);
      cost_block->AddParameterBlock(4);

      // Camera kinematic chain to target kinematic chain transform
      cost_block->AddParameterBlock(3);
      cost_block->AddParameterBlock(4);

      // Residual error
      cost_block->SetNumResiduals(2);

      // Add the residual block to the problem
      problem.AddResidualBlock(cost_block, NULL, parameters);
    }
  }

  // Set the local parameterization for the quaternions
  // We want to represent the quaternions with the minimum number of variables (3 vs. 4) in the optimization
  problem.SetParameterization(q_cm_to_c.coeffs().data(),
                              new ceres::EigenQuaternionParameterization());
  problem.SetParameterization(q_tm_to_t.coeffs().data(),
                              new ceres::EigenQuaternionParameterization());
  problem.SetParameterization(q_ccb_to_tcb.coeffs().data(),
                              new ceres::EigenQuaternionParameterization());

  // Tell the optimization to keep constant the dummy DH offsets that might have been added to the 0-DoF chains
  if (params.camera_chain.dof() == 0)
    problem.SetParameterBlockConstant(camera_chain_dh_offsets.data());
  if (params.target_chain.dof() == 0)
    problem.SetParameterBlockConstant(target_chain_dh_offsets.data());

  // TODO: Set bounds on the DH parameter offsets
//  problem.SetParameterUpperBound(camera_chain_dh_offsets.data(), 0, 0.01);
//  problem.SetParameterLowerBound(camera_chain_dh_offsets.data(), 0, -0.01);

  // TODO: Set identity local parameterization on individual variables that we want to remain constant
  // This will likely require an additional argument

  // Setup the Ceres optimization parameters
  ceres::Solver::Options options;
  options.max_num_iterations = 150;
  options.num_threads = 4;
  ceres::Solver::Summary summary;

  // Solve the optimization
  ceres::Solve(options, &problem, &summary);

  // Report and save the results
  KinematicCalibrationResult result;
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  // Save the transforms
  result.camera_mount_to_camera = Eigen::Translation3d(t_cm_to_c) * q_cm_to_c;
  result.target_mount_to_target = Eigen::Translation3d(t_tm_to_t) * q_tm_to_t;
  result.camera_base_to_target_base = Eigen::Translation3d(t_ccb_to_tcb) * q_ccb_to_tcb;

  // Save the DH parameter offsets
  if (params.camera_chain.dof() == 0)
    result.camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.camera_chain.dof(), 4);
  else
    result.camera_chain_dh_offsets = camera_chain_dh_offsets;

  if (params.target_chain.dof() == 0)
    result.target_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.target_chain.dof(), 4);
  else
    result.target_chain_dh_offsets = target_chain_dh_offsets;

  // TODO: Calculate the covariance
  // 1. Camera DH parameters
  // 2. Target DH parameters
  // 3. Camera pose to camera
  // 4. Target mount to target
  // 5. Camera base to target
  // 6. All parameters relative to each other

  return result;
}

} // namespace rct_optimizations

