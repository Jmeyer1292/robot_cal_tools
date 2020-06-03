#ifndef RCT_CAMERA_INTRINSIC_H
#define RCT_CAMERA_INTRINSIC_H

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
};

struct IntrinsicEstimationResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  CameraIntrinsics intrinsics;
  std::array<double, 5> distortions;

  std::vector<Eigen::Isometry3d> target_transforms;
};

IntrinsicEstimationResult optimize(const IntrinsicEstimationProblem& params);

}

#endif //RCT_CAMERA_INTRINSIC_H
