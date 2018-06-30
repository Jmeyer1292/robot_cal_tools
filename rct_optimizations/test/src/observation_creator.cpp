#include "rct_optimizations_tests/observation_creator.h"
#include "rct_optimizations/ceres_math_utilities.h"

static bool projectAndTest(const Eigen::Vector3d& pt_in_camera, const rct_optimizations::test::Camera& camera,
                           Eigen::Vector2d& pt_in_image)
{
  rct_optimizations::projectPoint(camera.intr, pt_in_camera.data(), pt_in_image.data());
  if (pt_in_image(0) < 0 || pt_in_image(0) > camera.width ||
      pt_in_image(1) < 0 || pt_in_image(1) > camera.height)
  {
    return false;
  }
  else return true;
}

bool rct_optimizations::test::project(const Eigen::Affine3d& camera_pose, const Eigen::Affine3d& target_pose,
                                      const test::Camera& camera, const test::Target& target,
                                      std::vector<Eigen::Vector2d>& out_observations)
{
  Eigen::Affine3d to_camera = camera_pose.inverse() * target_pose;

  std::vector<Eigen::Vector2d> obs;
  // Loop over points
  for (const auto& point : target.points)
  {
    Eigen::Vector3d pt_in_camera = to_camera * point;

    if (pt_in_camera.z() < 0.0) // behind the camera
    {
      return false;
    }

    Eigen::Vector2d in_image;
    if (!projectAndTest(pt_in_camera, camera, in_image)) // If this fails, the point is outside image
    {
      return false;
    }

    // Point is good
    obs.push_back(in_image);
  }

  out_observations = obs;
  return true;
}

Eigen::Affine3d rct_optimizations::test::lookat(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up)
{
  Eigen::Vector3d z = (eye - origin).normalized();
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Affine3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}
