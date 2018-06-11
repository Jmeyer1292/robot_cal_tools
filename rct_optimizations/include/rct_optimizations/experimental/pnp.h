#ifndef RCT_PNP_H
#define RCT_PNP_H

#include <rct_optimizations/types.h>

namespace rct_optimizations
{

struct PnPProblem
{
  rct_optimizations::CameraIntrinsics intr;
  ObservationSet correspondences;

  Eigen::Affine3d camera_to_target_guess;
};

struct PnPResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Affine3d camera_to_target;
};

PnPResult optimize(const PnPProblem& params);

}

#endif // RCT_PNP_H
