#ifndef RCT_MULTI_CAMERA_PNP_H
#define RCT_MULTI_CAMERA_PNP_H

#include <rct_optimizations/types.h>

namespace rct_optimizations
{

struct MultiCameraPnPProblem
{
  // For each camera, define its intrinsics
  std::vector<CameraIntrinsics> intr;
  // For each camera, define the correspondences it saw
  std::vector<CorrespondenceSet> correspondences;

  // If you have more than 1 camera, this vector specifies the transform between the optical
  // frame of the FIRST camera in the above image set and each subsequent camera.
  // So camera_transforms[0] = camera 0 to camera 1
  //    camera_ttransforms[1] = camera 0 to camera 2
  // etc...
  std::vector<Eigen::Affine3d> camera_transforms;

  // The transform from the FIRST camera to the target
  Eigen::Affine3d camera_to_target_guess;
};

struct MultiCameraPnPResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  // The transform from the FIRST camera to the target
  Eigen::Affine3d camera_to_target;
};

MultiCameraPnPResult optimize(const MultiCameraPnPProblem& params);

}

#endif // RCT_MUTLI_CAMERA_PNP_H
