#include <rct_optimizations/dh_chain_kinematic_calibration.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/covariance_analysis.h>

#include <ceres/ceres.h>

namespace rct_optimizations
{

Eigen::Isometry3d createTransform(const Eigen::Vector3d& t, const Eigen::Vector3d& aa)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(t);

  double aa_norm = aa.norm();
  if (aa_norm > std::numeric_limits<double>::epsilon())
  {
    result *= Eigen::AngleAxisd(aa_norm, aa.normalized());
  }

  return result;
}

KinematicCalibrationResult optimize(const KinematicCalibrationProblem2D3D &params)
{
  // Initialize the optimization variables
  // Camera mount to camera (cm_to_c) quaternion and translation
  Eigen::Vector3d t_cm_to_c(params.camera_mount_to_camera_guess.translation());
  std::array<std::string, 3> t_cm_to_c_labels({params.label_camera_mount_to_camera + "_x",
                                               params.label_camera_mount_to_camera + "_y",
                                               params.label_camera_mount_to_camera + "_z"});
  Eigen::AngleAxisd rot_cm_to_c(params.camera_mount_to_camera_guess.rotation());
  Eigen::Vector3d aa_cm_to_c(rot_cm_to_c.angle() * rot_cm_to_c.axis());
  std::array<std::string, 3> aa_cm_to_c_labels({params.label_camera_mount_to_camera + "_rx",
                                                params.label_camera_mount_to_camera + "_ry",
                                                params.label_camera_mount_to_camera + "_rz"});

  // Target mount to target (cm_to_c) quaternion and translation
  Eigen::Vector3d t_tm_to_t(params.target_mount_to_target_guess.translation());
  std::array<std::string, 3> t_tm_to_t_labels({params.label_target_mount_to_target + "_x",
                                               params.label_target_mount_to_target + "_y",
                                               params.label_target_mount_to_target + "_z"});
  Eigen::AngleAxisd rot_tm_to_t(params.target_mount_to_target_guess.rotation());
  Eigen::Vector3d aa_tm_to_t(rot_tm_to_t.angle() * rot_tm_to_t.axis());
  std::array<std::string, 3> aa_tm_to_t_labels({params.label_target_mount_to_target + "_rx",
                                                params.label_target_mount_to_target + "_ry",
                                                params.label_target_mount_to_target + "_rz"});

  // Camera chain base to target_chain_base (ccb_to_tcb) quaternion and translation
  Eigen::Vector3d t_ccb_to_tcb(params.camera_base_to_target_base_guess.translation());
  std::array<std::string, 3> t_ccb_to_tcb_labels({params.label_camera_base_to_target + "_x",
                                                  params.label_camera_base_to_target + "_y",
                                                  params.label_camera_base_to_target + "_z"});
  Eigen::AngleAxisd rot_ccb_to_tcb(params.camera_base_to_target_base_guess.rotation());
  Eigen::Vector3d aa_ccb_to_tcb(rot_ccb_to_tcb.angle() * rot_ccb_to_tcb.axis());
  std::array<std::string, 3> aa_ccb_to_tcb_labels({params.label_camera_base_to_target + "_rx",
                                                   params.label_camera_base_to_target + "_ry",
                                                   params.label_camera_base_to_target + "_rz"});

  // Create containers for the kinematic chain DH offsets
  // Ceres will not work with parameter blocks of size zero, so create a dummy set of DH offsets for chains with DoF == 0
  Eigen::MatrixX4d camera_chain_dh_offsets;
  std::vector<std::array<std::string, 4>> camera_chain_param_labels;
  if (params.camera_chain.dof() != 0)
  {
    camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.camera_chain.dof(), 4);
    camera_chain_param_labels = params.camera_chain.getParamLabels();
  }
  else
  {
    camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(1, 4);
    camera_chain_param_labels = {{"camera_ph_d", "camera_ph_theta", "camera_ph_r", "camera_ph_alpha"}};
  }


  Eigen::MatrixX4d target_chain_dh_offsets;
  std::vector<std::array<std::string, 4>> target_chain_param_labels;
  if (params.target_chain.dof() != 0)
  {
    target_chain_dh_offsets = Eigen::MatrixX4d::Zero(params.target_chain.dof(), 4);
    target_chain_param_labels = params.target_chain.getParamLabels();
  }
  else
  {
    target_chain_dh_offsets = Eigen::MatrixX4d::Zero(1, 4);
    target_chain_param_labels = {{"target_ph_d", "target_ph_theta", "target_ph_r", "target_ph_alpha"}};
  }

  // Create a vector of the pointers to the optimization variables in the order that the cost function expects them
  std::vector<double *> parameters
    = DualDHChainCost2D3D::constructParameters(camera_chain_dh_offsets,
                                               target_chain_dh_offsets,
                                               t_cm_to_c,
                                               aa_cm_to_c,
                                               t_tm_to_t,
                                               aa_tm_to_t,
                                               t_ccb_to_tcb,
                                               aa_ccb_to_tcb);

  std::vector<std::vector<std::string>> param_labels = DualDHChainCost2D3D::constructParameterLabels(camera_chain_param_labels,
                                                                                                     target_chain_param_labels,
                                                                                                     t_cm_to_c_labels,
                                                                                                     aa_cm_to_c_labels,
                                                                                                     t_tm_to_t_labels,
                                                                                                     aa_tm_to_t_labels,
                                                                                                     t_ccb_to_tcb_labels,
                                                                                                     aa_ccb_to_tcb_labels);

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

      auto *cost_block = new ceres::DynamicAutoDiffCostFunction<DualDHChainCost2D3D>(cost_fn);

      // Add the optimization parameters
      // DH parameters for both kinematic chains
      cost_block->AddParameterBlock(camera_chain_dh_offsets.size());
      cost_block->AddParameterBlock(target_chain_dh_offsets.size());

      // Camera mount to camera transform
      cost_block->AddParameterBlock(3);
      cost_block->AddParameterBlock(3);

      // Target mount to target transform
      cost_block->AddParameterBlock(3);
      cost_block->AddParameterBlock(3);

      // Camera kinematic chain to target kinematic chain transform
      cost_block->AddParameterBlock(3);
      cost_block->AddParameterBlock(3);

      // Residual error
      cost_block->SetNumResiduals(2);

      // Add the residual block to the problem
      problem.AddResidualBlock(cost_block, NULL, parameters);
    }
  }

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
  result.camera_mount_to_camera = createTransform(t_cm_to_c, aa_cm_to_c);
  result.target_mount_to_target = createTransform(t_tm_to_t, aa_tm_to_t);
  result.camera_base_to_target_base = createTransform(t_ccb_to_tcb, aa_ccb_to_tcb);

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

  // TODO: fix calculation of near-rank-deficient covariance matrix
  ceres::Covariance::Options cov_options = rct_optimizations::DefaultCovarianceOptions();
  cov_options.min_reciprocal_condition_number = -1;
  result.covariance = computeCovariance(problem, std::vector<const double*>(parameters.begin(), parameters.end()), param_labels, cov_options);

  return result;
}

} // namespace rct_optimizations

