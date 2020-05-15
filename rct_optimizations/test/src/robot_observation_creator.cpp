#include <rct_optimizations_tests/robot_observation_creator.h>
#include <rct_optimizations_tests/observation_creator.h>

namespace rct_optimizations
{
namespace test
{
Observation3D3D::Set create(DHRobot to_camera_chain,
                            DHRobot to_target_chain,
                            const Eigen::Isometry3d &camera_base_to_target_base,
                            const Target &target,
                            const std::size_t n)
{
  Observation3D3D::Set observations;
  observations.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    Observation3D3D obs;
    obs.to_camera_mount = to_camera_chain.createUniformlyRandomPose();
    obs.to_target_mount = to_target_chain.createUniformlyRandomPose();

    // Get the target transform in the same base frame as the camera
    Eigen::Isometry3d camera_base_to_target = camera_base_to_target_base * obs.to_camera_mount;

    obs.correspondence_set = getCorrespondences(obs.to_camera_mount,
                                                camera_base_to_target,
                                                target);
    observations.push_back(obs);
  }

  return observations;
}

Observation2D3D::Set create(DHRobot to_camera_chain,
                            DHRobot to_target_chain,
                            const Eigen::Isometry3d &camera_base_to_target_base,
                            const Target &target,
                            const Camera &camera,
                            const std::size_t n)
{
  Observation2D3D::Set observations;
  observations.reserve(n);

  std::size_t correspondences = 0;
  std::size_t attempts = 0;
  const std::size_t max_attempts = 1e6;
  while (correspondences < n * target.points.size() && attempts < max_attempts)
  {
    Observation2D3D obs;
    obs.to_camera_mount = to_camera_chain.createUniformlyRandomPose();
    obs.to_target_mount = to_target_chain.createUniformlyRandomPose();

    // Get the transform to the target in the same root frame as the transform to the camera
    Eigen::Isometry3d camera_base_to_target = camera_base_to_target_base * obs.to_target_mount;

    obs.correspondence_set = getCorrespondences(obs.to_camera_mount,
                                                camera_base_to_target,
                                                camera,
                                                target,
                                                false);
    if (obs.correspondence_set.size() > 0)
    {
      observations.push_back(obs);
      correspondences += obs.correspondence_set.size();
    }
    ++attempts;
  }

  if (attempts > max_attempts || correspondences < n * target.points.size())
  {
    std::stringstream ss;
    ss << "Failed to generate required number of observations (" << observations.size()
       << "/" << observations.capacity() << ")";
    throw std::runtime_error(ss.str());
  }

  return observations;
}

} // namespace test
} // namespace rct_optimizations
