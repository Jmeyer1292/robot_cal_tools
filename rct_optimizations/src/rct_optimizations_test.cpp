#include "rct_optimizations/extrinsic_camera_on_wrist.h"

int main()
{
  rct_optimizations::ExtrinsicCameraOnWristParameters p;
  rct_optimizations::optimize(p);
  return 0;
}
