#include <rct_optimizations/dh_chain.h>
#include <random>

namespace rct_optimizations
{
// DH Transform

DHTransform::DHTransform(std::array<double, 4> params_, DHJointType type_)
  : params(std::move(params_))
  , type(type_)
{
}

Eigen::Isometry3d DHTransform::createRelativeTransform(const double joint_value) const
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

double DHTransform::createRandomJointValue() const
{
  std::mt19937 mt_rand = std::mt19937(std::random_device{}());
  std::uniform_real_distribution<double> dist(min, max);
  return (dist(mt_rand));
}

// Gaussian Noise DH Transform

GaussianNoiseDHTransform::GaussianNoiseDHTransform(std::array<double, 4> params_,
                                                   DHJointType type_,
                                                   double mean_,
                                                   double std_dev_)
  : DHTransform(params_, type_)
  , mean(mean_)
  , std_dev(std_dev_)
{
}

Eigen::Isometry3d GaussianNoiseDHTransform::createRelativeTransform(const double joint) const
{
  std::mt19937 mt_rand(std::random_device{}());
  std::normal_distribution<double> dist(mean, std_dev);
  return DHTransform::createRelativeTransform(joint + dist(mt_rand));
}

// DH Chain
DHChain::DHChain(std::vector<DHTransform::Ptr> transforms)
  : transforms_(std::move(transforms))
{
}

Eigen::Isometry3d DHChain::getFK(const std::vector<double> &joint_values) const
{
  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
  for (std::size_t i = 0; i < joint_values.size(); ++i)
  {
    transform = transform * transforms_.at(i)->createRelativeTransform(joint_values.at(i));
  }
  return transform;
}

Eigen::Isometry3d DHChain::getFK(const double *joint_values, const std::size_t n_joints) const
{
  return getFK(std::vector<double>(joint_values, joint_values + n_joints));
}

Eigen::Isometry3d DHChain::createUniformlyRandomPose() const
{
  std::vector<double> joints;
  joints.reserve(transforms_.size());
  for (const auto& t : transforms_)
  {
    joints.push_back(t->createRandomJointValue());
  }
  return getFK(joints);
}

} // namespace rct_optimizations
