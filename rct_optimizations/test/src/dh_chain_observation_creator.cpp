#include <rct_optimizations_tests/dh_chain_observation_creator.h>
#include <rct_optimizations_tests/observation_creator.h>

namespace rct_optimizations
{
namespace test
{
Observation3D3D::Set create(const DHChain& to_camera_chain,
                            const DHChain& to_target_chain,
                            const Eigen::Isometry3d& true_mount_to_camera,
                            const Eigen::Isometry3d& true_mount_to_target,
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

    obs.correspondence_set = getCorrespondences(obs.to_camera_mount * true_mount_to_camera,
                                                camera_base_to_target_base * obs.to_target_mount * true_mount_to_target,
                                                target);
    observations.push_back(obs);
  }

  return observations;
}

Observation2D3D::Set create(const DHChain& to_camera_chain,
                            const DHChain& to_target_chain,
                            const Eigen::Isometry3d& true_mount_to_camera,
                            const Eigen::Isometry3d& true_mount_to_target,
                            const Eigen::Isometry3d &camera_base_to_target_base,
                            const Target &target,
                            const Camera &camera,
                            const std::size_t n)
{
  Observation2D3D::Set observations;
  observations.reserve(n);

  std::size_t correspondences = 0;
  std::size_t attempts = 0;
  const std::size_t max_attempts = 10000;
  while (correspondences < n * target.points.size() && attempts < max_attempts)
  {
    Observation2D3D obs;
    obs.to_camera_mount = to_camera_chain.createUniformlyRandomPose();
    obs.to_target_mount = to_target_chain.createUniformlyRandomPose();

    const Eigen::Isometry3d base_to_camera = obs.to_camera_mount * true_mount_to_camera;
    const Eigen::Isometry3d base_to_target = camera_base_to_target_base * obs.to_target_mount
                                             * true_mount_to_target;

    const Eigen::Vector3d &camera_z = base_to_camera.matrix().col(2).head<3>();
    const Eigen::Vector3d &target_z = base_to_target.matrix().col(2).head<3>();

    // Make sure the camera and target normals are not pointing in the same direction
    double dp = camera_z.dot(target_z);
    if (dp < -0.2)
    {
      try
      {
        obs.correspondence_set = getCorrespondences(base_to_camera,
                                                    base_to_target,
                                                    camera,
                                                    target,
                                                    true);
      }
      catch (const std::exception &)
      {
        continue;
      }

      if (obs.correspondence_set.size() > 0)
      {
        observations.push_back(obs);
        correspondences += obs.correspondence_set.size();
      }
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
