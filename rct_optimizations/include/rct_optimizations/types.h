#ifndef RCT_TYPES_H
#define RCT_TYPES_H

#include <array>
#include <vector>
#include <Eigen/Dense>

namespace rct_optimizations
{
/**
 * @brief Structure representing camera intrinsic parameters for a pin-hole model camera
 */
struct CameraIntrinsics
{
  std::array<double, 4> values;

  double& fx() { return values[0]; }
  double& fy() { return values[1]; }
  double& cx() { return values[2]; }
  double& cy() { return values[3]; }

  const double& fx() const { return values[0]; }
  const double& fy() const { return values[1]; }
  const double& cx() const { return values[2]; }
  const double& cy() const { return values[3]; }
};

/**
 * @brief Representation of an isometry homogeneous transform for better integration with Ceres
 */
struct Pose6d
{
  Pose6d() = default;
  Pose6d(std::array<double, 6> l) : values(l) {}

  std::array<double, 6> values;

  double& rx() { return values[0]; }
  double& ry() { return values[1]; }
  double& rz() { return values[2]; }
  double& x() { return values[3]; }
  double& y() { return values[4]; }
  double& z() { return values[5]; }
  const double& rx() const { return values[0]; }
  const double& ry() const { return values[1]; }
  const double& rz() const { return values[2]; }
  const double& x() const { return values[3]; }
  const double& y() const { return values[4]; }
  const double& z() const { return values[5]; }
};

/**
 * @brief A pair of corresponding features in a N-dimensional sensor "image" and 3D target
 */
template<Eigen::Index OBS_DIMENSION>
struct Correspondence
{
  using Set = std::vector<Correspondence<OBS_DIMENSION>>;
  Correspondence()
    : in_target(Eigen::Vector3d::Zero())
    , in_image(Eigen::Matrix<double, OBS_DIMENSION, 1>::Zero())
  {
  }

  Correspondence(const Eigen::Vector3d& in_target_,
                 const Eigen::Matrix<double, OBS_DIMENSION, 1>& in_image_)
    : in_target(in_target_)
    , in_image(in_image_)
  {
  }

  /** @brief XYZ location of the feature relative to the target origin (meters) */
  Eigen::Vector3d in_target;

  /** @brief N-dimensional location of the feature relative to the sensor */
  Eigen::Matrix<double, OBS_DIMENSION, 1> in_image;
};
/** @brief Typedef for correspondence between 2D feature in image coordinates and 3D feature in target coordinates */
using Correspondence2D3D = Correspondence<2>;
/** @brief Typedef for correspondence between 3D feature in sensor coordinates and 3D feature in target coordinates */
using Correspondence3D3D = Correspondence<3>;

// Deprecated typedefs
using CorrespondenceSet = Correspondence2D3D::Set;
using Correspondence3DSet = Correspondence3D3D::Set;

/**
 * @brief A set of data representing a single observation of a calibration target.
 * This consists of the feature correspondences as well as the transforms to the "mount" frames of the camera and target.
 * For a moving camera or target, the "mount" pose would likely be the transform from the robot base to the robot tool flange.
 * For a stationary camera or target, this "mount" pose would simply be identity.
 * Keep in mind that the optimization itself determines the final calibrated transforms from these "mount" frames to the camera and target.
 */
template<Eigen::Index OBS_DIMENSION>
struct Observation
{
  using Set = std::vector<Observation<OBS_DIMENSION>>;
  Observation()
    : base_to_camera_mount(Eigen::Isometry3d::Identity())
    , base_to_target_mount(Eigen::Isometry3d::Identity())
  {
  }

  Observation(const Eigen::Isometry3d& base_to_camera_mount_,
              const Eigen::Isometry3d& base_to_target_mount_)
    : base_to_camera_mount(base_to_camera_mount_)
    , base_to_target_mount(base_to_target_mount_)
  {
  }

  /** @brief A set of feature correspondences between the sensor output and target */
  typename Correspondence<OBS_DIMENSION>::Set correspondence_set;
  /** @brief The transform from a common root frame to the frame to which the camera is mounted. */
  Eigen::Isometry3d base_to_camera_mount;
  /** @brief The transform from a common root frame to the frame to which the target is mounted. */
  Eigen::Isometry3d base_to_target_mount;
};
/** @brief Typedef for observations of 2D image to 3D target correspondences */
using Observation2D3D = Observation<2>;
/** @brief Typedef for observations of 3D sensor to 3D target correspondences */
using Observation3D3D = Observation<3>;

} // namespace rct_optimizations

#endif
