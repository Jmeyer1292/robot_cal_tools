#pragma once

#include <Eigen/Geometry>
#include <vector>

namespace rct_optimizations
{
namespace test
{

/**
 * @brief Defines a camera matrix using a camera origin, a position its looking at, and an up vector hint
 * @param origin - The position of the camera focal point
 * @param eye - A point that the camera is looking at
 * @param up - The upvector in world-space
 */
Eigen::Isometry3d lookAt(const Eigen::Vector3d &origin,
                         const Eigen::Vector3d &eye,
                         const Eigen::Vector3d &up) noexcept;

struct PoseGenerator
{
  /**
   * @brief Generates a set of camera poses
   * @param target_origin The position of the target
   * @return A vector of camera positions & orientations
   */
  virtual std::vector<Eigen::Isometry3d> generate(const Eigen::Vector3d &target_origin) const = 0;
};

/**
 * @brief Generates camera poses in a hemisphere pattern
 */
struct HemispherePoseGenerator : PoseGenerator
{
  inline HemispherePoseGenerator(double r_ = 2.0,
                                 unsigned theta_cnt_ = 10,
                                 unsigned phi_cnt_ = 10,
                                 double z_rot_ = 0.0,
                                 const Eigen::Isometry3d& target_offset_ = Eigen::Isometry3d::Identity())
    : r(r_)
    , theta_cnt(theta_cnt_)
    , phi_cnt(phi_cnt_)
    , z_rot(z_rot_)
    , target_offset(target_offset_)
  {
  }

  virtual std::vector<Eigen::Isometry3d> generate(
    const Eigen::Vector3d &target_origin) const override final;

  double r; /* @brief Radius of the hemisphere */
  unsigned theta_cnt; /* @brief The number of points in the theta-wise direction*/
  unsigned phi_cnt; /* @brief The number of points in the phi-wise direction */
  double z_rot; /* @brief Rotation to apply around camera Z-axis (radians) */
  Eigen::Isometry3d target_offset; /* @brief Transform from target origin to origin of pose pattern */
};

/**
 * @brief Generates camera positions using a conical template, with the target at the 'point'
 */
struct ConicalPoseGenerator : PoseGenerator
{
  inline ConicalPoseGenerator(double r_ = 1.0,
                              double h_ = 2.0,
                              unsigned n_poses_ = 20,
                              double z_rot_ = 0.0,
                              const Eigen::Isometry3d& target_offset_ = Eigen::Isometry3d::Identity())
    : r(r_)
    , h(h_)
    , n_poses(n_poses_)
    , z_rot(z_rot_)
    , target_offset(target_offset_)
  {
  }

  virtual std::vector<Eigen::Isometry3d> generate(
    const Eigen::Vector3d &target_origin) const override final;

  double r; /** @brief Radius of the cone*/
  double h; /** @brief Height of the cone (distance to target) */
  unsigned n_poses; /** @brief Number of poses to generate */
  double z_rot; /* @brief Rotation to apply around camera Z-axis (radians) */
  Eigen::Isometry3d target_offset; /* @brief Transform from target origin to origin of pose pattern */
};

/**
 * @brief Generates poses in a grid
 */
struct GridPoseGenerator : PoseGenerator
{
  inline GridPoseGenerator(double spacing_ = 0.2,
                           double h_ = 2.0,
                           unsigned grid_side_ = 10,
                           double z_rot_ = 0.0,
                           const Eigen::Isometry3d& target_offset_ = Eigen::Isometry3d::Identity())
    : spacing(spacing_)
    , h(h_)
    , grid_side(grid_side_)
    , z_rot(z_rot_)
    , target_offset(target_offset_)
  {
  }

  virtual std::vector<Eigen::Isometry3d> generate(
    const Eigen::Vector3d &target_origin) const override final;

  double spacing; /** @brief Distance betweeon points */
  double h; /** @brief Grid distance to target */
  unsigned grid_side; /** number of columns & rows to go into grid */
  double z_rot; /* @brief Rotation to apply around camera Z-axis (radians) */
  Eigen::Isometry3d target_offset; /* @brief Transform from target origin to origin of pose pattern */
};

}  // namespace test
}  // namespace rct_optimizations
