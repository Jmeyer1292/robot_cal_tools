#pragma once

#include <Eigen/Geometry>
#include <memory>
#include <utility>

namespace rct_optimizations
{
enum class DHJointType : unsigned
{
  LINEAR,
  REVOLUTE,
  FIXED
};

/**
 * @brief Struct representing the DH parameters of a single transformation between adjacent links.
 * This struct follows the classical DH parameter convention: Trans[Zi-1](d) * Rot[Zi-1](theta) * Trans[Xi](r) * Rot[Xi](alpha)
 * See @link https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters for reference
 */
struct DHTransform
{
  using Ptr = std::unique_ptr<DHTransform>;

  DHTransform(std::array<double, 4> params_, DHJointType type_);
  virtual ~DHTransform() = default;

  /**
   * @brief Creates the homogoneous transformation from the previous link to the current link
   * @param joint_value
   * @return
   */
  virtual Eigen::Isometry3d createRelativeTransform(const double joint_value) const;

  double createRandomJointValue() const;

  /** @brief DH parameters
   *  d: The linear offset in Z
   *  theta: The rotational offset about Z
   *  r: The linear offset in X
   *  alpha: The rotational offset about X
   */
  std::array<double, 4> params;
  DHJointType type; /** @brief The type of actuation of the joint */
  double max = M_PI; /** @brief Joint max */
  double min = -M_PI; /** @brief Joint min */
};

/**
 * @brief Override of the @ref DHTransform class to provide Gaussian noise for the joint value when calculating forward kinematics
 */
struct GaussianNoiseDHTransform : public DHTransform
{
  GaussianNoiseDHTransform(std::array<double, 4> params_,
                           DHJointType type_,
                           double mean_,
                           double std_dev_);

  virtual Eigen::Isometry3d createRelativeTransform(const double joint) const override;

  double mean;
  double std_dev;
};

/**
 * @brief Robot representation using DH parameters
 */
class DHChain
{
public:
    DHChain(std::vector<DHTransform::Ptr> transforms);

  /**
   * @brief Calculates forward kinematics for the robot with the joints provided.
   * Note: the transform to the n-th link is calculated, where n is the size of @ref joint_values
   * @param joint_values - The joint values with which to calculate forward kinematics.
   * @return
   * @throws Exception if the size of joint values is larger than the number of DH transforms in the robot
   */
  Eigen::Isometry3d getFK(const std::vector<double> &joint_values) const;

  /**
   * @brief Override function of @ref getFK but using a data pointer for easier integration with Ceres
   * @param joint_values
   * @param n_joints
   * @return
   */
  Eigen::Isometry3d getFK(const double *joint_values, const std::size_t n_joints) const;

  Eigen::Isometry3d createUniformlyRandomPose() const;

protected:
  std::vector<DHTransform::Ptr> transforms_;
};

} // namespace rct_optimizations
