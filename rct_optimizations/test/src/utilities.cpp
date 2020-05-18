#include "rct_optimizations_tests/utilities.h"
#include <random>

namespace rct_optimizations
{
namespace test
{
Camera makeKinectCamera()
{
  CameraIntrinsics intr;
  intr.fx() = 550.0;
  intr.fy() = 550.0;
  intr.cx() = 640/2;
  intr.cy() = 480/2;

  Camera camera;
  camera.intr = intr;
  camera.width = 640;
  camera.height = 480;

  return camera;
}

Eigen::Isometry3d perturbPose(const Eigen::Isometry3d& pose, double spatial_noise, double angle_noise)
{
  std::random_device dev;
  std::default_random_engine eng (dev());

  std::uniform_real_distribution<double> spatial_dist (-spatial_noise, spatial_noise);
  std::uniform_real_distribution<double> angle_dist (-angle_noise, angle_noise);
  std::uniform_real_distribution<double> one_to_one (-1, 1);

  Eigen::Vector3d translation (spatial_dist(eng), spatial_dist(eng), spatial_dist(eng));
  Eigen::Vector3d rot_axis (one_to_one(eng), one_to_one(eng), one_to_one(eng));
  rot_axis.normalize();

  double angle = angle_dist(eng);

  Eigen::Isometry3d new_pose = pose * Eigen::Translation3d(translation) * Eigen::AngleAxisd(angle, rot_axis);

  return new_pose;
}

} // namespace test
} // namespace rct_optimizations



