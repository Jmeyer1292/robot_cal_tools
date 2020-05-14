#ifndef DH_PARAMETER_H
#define DH_PARAMETER_H

#include <utility>
#include <random>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>

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

  DHTransform(DHJointType type_)
    : type(type_)
  {
  }

  DHTransform(DHJointType type_, double d_, double theta_, double r_, double alpha_)
    : type(type_)
    , d(d_)
    , theta(theta_)
    , r(r_)
    , alpha(alpha_)
  {
  }

  virtual ~DHTransform() = default;

  /**
   * @brief Creates the homogoneous transformation from the previous link to the current link
   * @param joint_value
   * @return
   */
  virtual Eigen::Isometry3d createRelativeTransform(const double joint_value)
  {
    Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());

    // Create aliases for the d and theta parameters such that the joint value could be added to them appropriately
    double d_ = d;
    double theta_ = theta;
    switch (type)
    {
    case DHJointType::LINEAR:
      d_ += joint_value;
      break;
    case DHJointType::REVOLUTE:
      theta_ += joint_value;
      break;
    default:
      break;
    }

    // Perform the DH transformations
    transform.translate(Eigen::Vector3d(0.0, 0.0, d_));
    transform.rotate(Eigen::AngleAxisd(theta_, Eigen::Vector3d::UnitZ()));
    transform.translate(Eigen::Vector3d(r, 0.0, 0.0));
    transform.rotate(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()));

    return transform;
  }

  double createRandomJointValue() const
  {
    std::random_device rd;
    std::default_random_engine gen(rd());
    std::uniform_real_distribution<double> dist(min, max);
    return (dist(gen));
  }

  DHJointType type; /** @brief The type of actuation of the joint */
  double d; /** @brief The linear offset in Z */
  double theta; /** @brief The rotational offset about Z */
  double r; /** @brief The linear offset in X */
  double alpha; /** @brief The rotational offset about X */

  double max =  M_PI / 2.0; // std::numeric_limits<double>::max(); /** @brief Joint max */
  double min = -M_PI / 2.0; // std::numeric_limits<double>::lowest(); /** @brief Joint min */
};

/**
 * @brief Override of the @ref DHTransform class to provide Gaussian noise for the joint value when calculating forward kinematics
 */
struct GaussianNoiseDHTransform : public DHTransform
{
  GaussianNoiseDHTransform(DHJointType type_, double mean, double std_dev)
    : DHTransform(type_)
    , gen(rd())
    , dist(mean, std_dev)
  {
  }

  GaussianNoiseDHTransform(DHJointType type_,
                           double d_,
                           double theta_,
                           double r_,
                           double alpha_,
                           double mean,
                           double std_dev)
    : DHTransform(type_, d_, theta_, r_, alpha_)
    , gen(rd())
    , dist(mean, std_dev)
  {
  }

  virtual Eigen::Isometry3d createRelativeTransform(const double joint) override
  {
    return DHTransform::createRelativeTransform(joint + dist(gen));
  }

  std::random_device rd;                  /** @brief Random device */
  std::mt19937 gen;                       /** @brief Random number generation engine */
  std::normal_distribution<double> dist;  /** @brief Gaussian distribution number generator */
};

/**
 * @brief Robot representation using DH parameters
 */
class DHRobot
{
public:
  DHRobot(std::vector<DHTransform::Ptr> transforms)
    : transforms_(std::move(transforms))
  {
  }

  /**
   * @brief Calculates forward kinematics for the robot with the joints provided.
   * Note: the transform to the n-th link is calculated, where n is the size of @ref joint_values
   * @param joint_values - The joint values with which to calculate forward kinematics.
   * @return
   * @throws Exception if the size of joint values is larger than the number of DH transforms in the robot
   */
  Eigen::Isometry3d getFK(const std::vector<double> &joint_values)
  {
    Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
    for (std::size_t i = 0; i < joint_values.size(); ++i)
    {
      transform = transform * transforms_.at(i)->createRelativeTransform(joint_values.at(i));
    }
    return transform;
  }

  /**
   * @brief Override function of @ref getFK but using a data pointer for easier integration with Ceres
   * @param joint_values
   * @param n_joints
   * @return
   */
  Eigen::Isometry3d getFK(const double *joint_values, const std::size_t n_joints)
  {
    return getFK(std::vector<double>(joint_values, joint_values + n_joints));
  }

  Eigen::Isometry3d createUniformlyRandomPose()
  {
    std::vector<double> joints;
    joints.reserve(transforms_.size());
    for (std::size_t i = 0; i < transforms_.size(); ++i)
    {
      joints.push_back(transforms_.at(i)->createRandomJointValue());
    }
    return getFK(joints);
  }

protected:
  std::vector<DHTransform::Ptr> transforms_;
};

} // namespace rct_optimizations

#endif // DH_PARAMETER_H
