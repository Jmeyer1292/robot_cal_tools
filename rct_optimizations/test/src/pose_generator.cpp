#include "rct_optimizations_tests/observation_creator.h"
#include "rct_optimizations_tests/pose_generator.h"
#include <stdexcept>

// hemisphere

std::vector<Eigen::Isometry3d>
rct_optimizations::test::genHemispherePose(const Eigen::Vector3d& target_pose, const double r, const unsigned int theta_cnt, const unsigned int phi_cnt)
{
  // all in the target coordinate system, with z forward
  std::vector<Eigen::Isometry3d> camera_positions;

  Eigen::VectorXd theta_range = Eigen::VectorXd::LinSpaced(theta_cnt, 0.0, M_PI);
  Eigen::VectorXd phi_range = Eigen::VectorXd::LinSpaced(phi_cnt, 0.0, M_PI);

  for (std::size_t theta_it = 0; theta_it < theta_range.size(); ++theta_it)
  {
    std::vector<Eigen::Isometry3d> local_scan_path;
    local_scan_path.reserve(phi_range.size() * theta_range.size());
    for (std::size_t phi_it = 0; phi_it < phi_range.size(); ++phi_it)
    {
      double theta_cur = theta_range(theta_it);
      double phi_cur = phi_range(phi_it);

      // position in target coordinate frame
      Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();
      camera.translation()(0) = r * std::cos(theta_cur) * std::sin(phi_cur);
      camera.translation()(1) = r * std::sin(theta_cur) * std::sin(phi_cur);
      camera.translation()(2) = r * std::cos(phi_cur);

      // x is 'up' in target frame (confirm)
      Eigen::Isometry3d camera_oriented =
          rct_optimizations::test::lookAt(camera.translation(), target_pose, Eigen::Vector3d(1, 0, 0));

      // this vector is still in target spatial coordinates
      camera_positions.push_back(camera_oriented);
    }
  }

  return camera_positions;
}

std::vector<Eigen::Isometry3d>
rct_optimizations::test::genConicalPose(const Eigen::Vector3d& target_pose, const unsigned int observations, const double r, const double h)
{
  std::vector<Eigen::Isometry3d> camera_positions;
  // Generates positions in target frame; need to convert to world frame
  double dt = 2.0f * M_PI / double(observations);  // delta theta about cone base

  for (int i = 0; i < observations; ++i)
  {
    Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
    camera_pose.translation() = target_pose;

    // preserving target spatial coordinate frame:
    camera_pose.translate(Eigen::Vector3d{ r * cos(i * dt), r * sin(i * dt), h });

    // change orientation to look at target
    Eigen::Isometry3d camera_oriented =
        rct_optimizations::test::lookAt(camera_pose.translation(), target_pose, Eigen::Vector3d(1, 0, 0));

    camera_positions.push_back(camera_oriented);
  }

  return camera_positions;
}

std::vector<Eigen::Isometry3d>
rct_optimizations::test::genGridPose(const Eigen::Vector3d& target_pose, const unsigned int grid_side, const double spacing, const double h)
{
  // Generates positions in target frame; need to convert to world frame
  std::vector<Eigen::Isometry3d> camera_positions;

  for (int i = -1 * (grid_side - 1) / 2; i <= (grid_side - 1) / 2; ++i)
  {
    for (double j = -1 * (grid_side - 1) / 2; j <= (grid_side - 1) / 2; ++j)
    {
      Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
      camera_pose.translation() = Eigen::Vector3d{ i * spacing, j * spacing, h };

      // preserving target spatial coordinate frame:
      camera_pose.translate(Eigen::Vector3d{ i * spacing, j * spacing, h });

      // change orientation to look at target
      Eigen::Isometry3d camera_oriented = rct_optimizations::test::lookAt(
          camera_pose.translation(), target_pose, Eigen::Vector3d(1, 0, 0));

      camera_positions.push_back(camera_oriented);
    }
  }


  return camera_positions;
}
