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
  auto spatial_dist = std::bind(std::uniform_real_distribution<double>{-spatial_noise, spatial_noise}, std::mt19937(std::random_device{}()));
  auto angle_dist = std::bind(std::uniform_real_distribution<double>{-angle_noise, angle_noise}, std::mt19937(std::random_device{}()));
  auto one_to_one = std::bind(std::uniform_real_distribution<double>{-1, 1}, std::mt19937(std::random_device{}()));

  Eigen::Vector3d translation (spatial_dist(), spatial_dist(), spatial_dist());
  Eigen::Vector3d rot_axis (one_to_one(), one_to_one(), one_to_one());
  rot_axis.normalize();

  double angle = angle_dist();

  Eigen::Isometry3d new_pose = pose * Eigen::Translation3d(translation) * Eigen::AngleAxisd(angle, rot_axis);

  return new_pose;
}

} // namespace test
} // namespace rct_optimizations



