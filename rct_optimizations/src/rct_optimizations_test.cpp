#include "rct_optimizations/extrinsic_camera_on_wrist.h"

int main()
{
  using namespace rct_optimizations;

  Observation2d obs;
  obs.x() = 400;
  obs.y() = 400;

  CameraIntrinsics intr;
  intr.cx() = 640 / 2;
  intr.cy() = 480 / 2;
  intr.fx() = 550;
  intr.fy() = 550;

  Pose6d wrist_pose;
  wrist_pose.z() = 0;
  wrist_pose.x() = 10.0;
  wrist_pose.y() = 0.0;

  wrist_pose.rx() = 0;
  wrist_pose.ry() = -3.14159/2.;
  wrist_pose.rz() = 0;

  Point3d in_target;
  in_target.values[0] = in_target.values[1] = in_target.values[2] = 0.0;

  Pose6d target_guess ({0, 0, 0, 1, 0, 0});
  Pose6d camera_guess ({0, -3.14159/2.0, 0, 0, 0, 0});

  rct_optimizations::ExtrinsicCameraOnWristParameters p;
  p.intr = intr;
  p.base_to_target_guess = target_guess;
  p.wrist_to_camera_guess = camera_guess;
  // TODO: Generate fake data

  const auto opt_result = rct_optimizations::optimize(p);

  return 0;
}
