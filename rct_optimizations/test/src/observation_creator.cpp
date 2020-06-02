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

namespace rct_optimizations
{
namespace test
{
Correspondence2D3D::Set getCorrespondences(const Eigen::Isometry3d &camera_pose,
                                           const Eigen::Isometry3d &target_pose,
                                           const Camera &camera,
                                           const Target &target,
                                           const bool require_all)
{
  Correspondence2D3D::Set correspondences;
  correspondences.reserve(target.points.size());

  Eigen::Isometry3d camera_to_target = camera_pose.inverse() * target_pose;

  // Loop over points
  for (const auto &point : target.points)
  {
    Correspondence2D3D corr;
    corr.in_target = point;

    // Calculate the target coordinates in the camera frame
    Eigen::Vector3d pt_in_camera = camera_to_target * point;

    // Include points only in front of the camera and inside the image
    if (pt_in_camera.z() > 0.0 && projectAndTest(pt_in_camera, camera, corr.in_image))
    {
      correspondences.push_back(corr);
    }
    else if (require_all)
    {
      throw std::runtime_error("Target point was not observed");
    }
  }

  return correspondences;
}


Correspondence3D3D::Set getCorrespondences(const Eigen::Isometry3d &camera_pose,
                                           const Eigen::Isometry3d &target_pose,
                                           const Target &target) noexcept
{
  Correspondence3D3D::Set correspondences;
  correspondences.reserve(target.points.size());

  Eigen::Isometry3d camera_to_target = camera_pose.inverse() * target_pose;

  for (const auto &pt : target.points)
  {
    Correspondence3D3D corr;
    corr.in_target = pt;
    corr.in_image = camera_to_target * pt;
    correspondences.push_back(corr);
  }

  return correspondences;
}

Eigen::Isometry3d lookAt(const Eigen::Vector3d &origin,
                         const Eigen::Vector3d &eye,
                         const Eigen::Vector3d &up) noexcept
{
  Eigen::Vector3d z = (eye - origin).normalized();
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Isometry3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

} // namespace test
} // namespace rct_optimizations

