#ifndef RCT_CAMERA_INTRINSIC_H
#define RCT_CAMERA_INTRINSIC_H

#include <rct_optimizations/covariance_types.h>
#include "rct_optimizations/types.h"
#include "boost/optional.hpp"

namespace rct_optimizations
{
struct IntrinsicEstimationProblem
{
  std::vector<Correspondence2D3D::Set> image_observations;
  CameraIntrinsics intrinsics_guess;
  bool use_extrinsic_guesses;
  std::vector<Eigen::Isometry3d> extrinsic_guesses;

  std::string label_extr = "pose";
  const std::array<std::string, 9> labels_intrinsic_params = {
    { "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3" }
  };
  const std::array<std::string, 6> labels_isometry3d = { { "x", "y", "z", "rx", "ry", "rz" } };
};

struct IntrinsicEstimationResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  CameraIntrinsics intrinsics;
  std::array<double, 5> distortions;

  std::vector<Eigen::Isometry3d> target_transforms;

  CovarianceResult covariance;
};

IntrinsicEstimationResult optimize(const IntrinsicEstimationProblem& params);

}  // namespace rct_optimizations

#endif  // RCT_CAMERA_INTRINSIC_H
