#ifndef RCT_RAIL_INTRINSIC_H
#define RCT_RAIL_INTRINSIC_H

#include "rct_optimizations/types.h"

namespace rct_optimizations
{

struct RailIntrinsicProblem
{
  std::vector<Eigen::Affine3d> extrinsic_guesses;                 // for each "scene"
  std::vector<std::vector<double>> rail_distances;                // for each scene, for each image
  std::vector<std::vector<CorrespondenceSet>> image_observations; // for each scene, for each image, for each dot
  double skew_x_guess, skew_y_guess;

  CameraIntrinsics intrinsics_guess;
};

struct RailIntrinsicResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  CameraIntrinsics intrinsics;
  std::array<double, 5> distortions;

  std::vector<Eigen::Affine3d> target_transforms;
  double skew_x, skew_y;
};

RailIntrinsicResult optimize(const RailIntrinsicProblem& params);

}

#endif // RCT_RAIL_INTRINSIC_H
