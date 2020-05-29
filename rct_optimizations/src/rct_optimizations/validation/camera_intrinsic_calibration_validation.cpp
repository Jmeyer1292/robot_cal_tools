#include <rct_optimizations/validation/camera_intrinsic_calibration_validation.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

// Specialize std::numeric_limits for the Eigen::Vector3d type
// Ref: https://stackoverflow.com/questions/7908982/using-boost-accumulators-with-eigenvector-types
namespace std
{
template<>
struct numeric_limits<Eigen::Vector3d>
{
  static const bool is_specialized = true;
  static Eigen::Vector3d max()
  {
    return Eigen::Vector3d::Ones() * std::numeric_limits<double>::max();
  }
};
} // namespace std

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

bool validateCameraIntrinsicCalibration(const Observation2D3D::Set &observations,
                                        const CameraIntrinsics &intr,
                                        const Eigen::Isometry3d &camera_to_target_guess,
                                        const double diff_threshold)
{
  // Check that the observations are all the same size
  // Assuming that each observation's correspondences are ordered the same
  for (std::size_t i = 0; i < observations.size() - 1; ++i)
  {
    const Observation2D3D &obs_1 = observations[i];
    const Observation2D3D &obs_2 = observations[i + 1];

    // Check that the correspondences in all observations are the same size
    if (obs_1.correspondence_set.size() != obs_2.correspondence_set.size())
    {
      throw std::runtime_error("They don't match");
    }
  }

  // Create accumulators for mean and variance
  namespace ba = boost::accumulators;
  ba::accumulator_set<Eigen::Vector3d, ba::stats<ba::tag::mean>> acc;

  // Accumulate the position vector of the transformation
  for (const auto &obs : observations)
  {
    Eigen::Isometry3d t = getInternalTargetTransformation(obs.correspondence_set,
                                                          intr,
                                                          camera_to_target_guess,
                                                          1.0);
    acc(t.translation());
  }

  // Calculate the mean and variance of the measurements
  Eigen::Vector3d mean = ba::mean(acc);

  // Make sure that the mean is less than the acceptable threshold
  return mean.norm() < diff_threshold;
}

} // namespace rct_optimizations
