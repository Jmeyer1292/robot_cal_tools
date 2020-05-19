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

  DHTransform(std::array<double, 4> params_, DHJointType type_)
    : params(std::move(params_))
    , type(type_)
  {
  }


  virtual ~DHTransform() = default;

  /**
   * @brief Creates the homogoneous transformation from the previous link to the current link
   * @param joint_value
   * @return
   */
  virtual Eigen::Isometry3d createRelativeTransform(const double joint_value) const
  {
    Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());

    // Create aliases for the d and theta parameters such that the joint value could be added to them appropriately
    double d = params[0];
    double theta = params[1];
    double r = params[2];
    double alpha = params[3];
    switch (type)
    {
    case DHJointType::LINEAR:
      d += joint_value;
      break;
    case DHJointType::REVOLUTE:
      theta += joint_value;
      break;
    default:
      break;
    }

    // Perform the DH transformations
    transform.translate(Eigen::Vector3d(0.0, 0.0, d));
    transform.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
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
  GaussianNoiseDHTransform(DHJointType type_, double mean_, double std_dev_)
    : DHTransform(type_)
    , mean(mean_)
    , std_dev(std_dev_)
  {
  }

  GaussianNoiseDHTransform(std::array<double, 4> params_,
                           DHJointType type_,
                           double mean_,
                           double std_dev_)
    : DHTransform(params_, type_)
    , mean(mean_)
    , std_dev(std_dev_)
  {
  }

  virtual Eigen::Isometry3d createRelativeTransform(const double joint) const override
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(mean, std_dev);
    return DHTransform::createRelativeTransform(joint + dist(gen));
  }

  double mean;
  double std_dev;
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
  Eigen::Isometry3d getFK(const std::vector<double> &joint_values) const
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
  Eigen::Isometry3d getFK(const double *joint_values, const std::size_t n_joints) const
  {
    return getFK(std::vector<double>(joint_values, joint_values + n_joints));
  }

  Eigen::Isometry3d createUniformlyRandomPose() const
  {
    std::vector<double> joints;
    joints.reserve(transforms_.size());
    for (const auto& t : transforms_)
    {
      joints.push_back(t->createRandomJointValue());
    }
    return getFK(joints);
  }

protected:
  std::vector<DHTransform::Ptr> transforms_;
};

} // namespace rct_optimizations

#endif // DH_PARAMETER_H
