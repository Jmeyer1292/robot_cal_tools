#include <rct_optimizations/validation/camera_intrinsic_calibration_validation.h>

namespace rct_optimizations
{
Eigen::Isometry3d getInternalTargetTransformation(const Correspondence2D3D::Set &correspondences,
                                                  const CameraIntrinsics &intr,
                                                  const Eigen::Isometry3d &camera_to_target_guess,
                                                  const double residual_sq_error_threshold)
{
  // Create a lambda for doing the PnP optimization
  auto solve_pnp = [&intr, &camera_to_target_guess, &residual_sq_error_threshold](
                     const Correspondence2D3D::Set &corr) -> Eigen::Isometry3d {
    // Create the first virtual target PnP problem
    PnPProblem problem;
    problem.intr = intr;
    problem.correspondences = corr;
    problem.camera_to_target_guess = camera_to_target_guess;

    PnPResult result = optimize(problem);
    if (!result.converged || result.final_cost_per_obs > residual_sq_error_threshold)
    {
      std::stringstream ss;
      ss << "PnP optimization " << (result.converged ? "converged" : "did not converge")
         << " with residual error of " << result.final_cost_per_obs << " ("
         << residual_sq_error_threshold << " max)";
      throw std::runtime_error(ss.str());
    }

    return result.camera_to_target;
  };

  // Calculate the size of half of the correspondence set
  std::size_t half_size = correspondences.size() / 2;

  // Create two half sets of correspondences
  Correspondence2D3D::Set set_1(correspondences.begin(), correspondences.begin() + half_size);
  Correspondence2D3D::Set set_2(correspondences.begin() + half_size, correspondences.end());

  /* Get the camera to target transformation for each half set
   * Note: these transforms are from the camera to the origin of each virtual target.
   *   The origin of the second virtual target is still the same as the first virtual target,
   *   so the two transforms should be the same, given perfect camera intrinsics */
  Eigen::Isometry3d camera_to_target_1 = solve_pnp(set_1);
  Eigen::Isometry3d camera_to_target_2 = solve_pnp(set_2);

  // Return the transformation from target 1 to target 2
  return camera_to_target_1.inverse() * camera_to_target_2;
}

} // namespace rct_optimizations
