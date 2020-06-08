#pragma once
#include <rct_optimizations/types.h>
#include <rct_image_tools/modified_circle_grid_target.h>

namespace rct_image_tools
{
struct ProclivityParams
{
  rct_optimizations::CameraIntrinsics intr;
  rct_image_tools::ModifiedCircleGridTarget target;
  rct_optimizations::Observation2D3D ob; //learn to use templates, and then do this right
  double max_residual; //this can be omitted if residuals are compared internally
};

bool checkObservationProclivity(const ProclivityParams& params);

}//rct_image_tools
