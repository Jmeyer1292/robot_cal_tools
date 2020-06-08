#pragma once
#include <rct_optimizations/dh_chain.h>
#include <random>

namespace rct_optimizations
{
// DH Transform

inline DHTransform::DHTransform(const Eigen::Vector4d& params_, DHJointType type_)
  : params(params_)
  , type(type_)
{
}

template<typename T>
Isometry3<T> DHTransform::createRelativeTransform(const T joint_value,
                                                  const Eigen::Matrix<T, 1, 4>& offsets) const
{
  Isometry3<T> transform(Isometry3<T>::Identity());

  // Create an DH parameter vector with offsets ([d, theta, r, alpha]
  Vector4<T> updated_params = params.cast<T>() + offsets.transpose();

  switch (type)
  {
  case DHJointType::LINEAR:
    // Add the joint value to d (index 0) if the joint is linear
    updated_params(0) += joint_value;
    break;
  case DHJointType::REVOLUTE:
    // Add the joint value to theta (index 1) if the joint is revolute
    updated_params(1) += joint_value;
    break;
  default:
    break;
  }

  // Perform the DH transformations
  transform *= Eigen::Translation<T, 3>(T(0.0), T(0.0), updated_params(0));
  transform.rotate(Eigen::AngleAxis<T>(updated_params(1), Vector3<T>::UnitZ()));
  transform *= Eigen::Translation<T, 3>(updated_params(2), T(0.0), T(0.0));
  transform.rotate(Eigen::AngleAxis<T>(updated_params(3), Vector3<T>::UnitX()));

  return transform;
}

template<typename T>
inline Isometry3<T> DHTransform::createRelativeTransform(const T joint) const
{
  Eigen::Matrix<T, 1, 4> offsets = Eigen::Matrix<T, 1, 4>::Zero();
  return DHTransform::createRelativeTransform(joint, offsets);
}

inline double DHTransform::createRandomJointValue() const
{
  std::mt19937 mt_rand = std::mt19937(std::random_device{}());
  std::uniform_real_distribution<double> dist(min, max);
  return dist(mt_rand);
}

// DH Chain
inline DHChain::DHChain(std::vector<DHTransform::Ptr> transforms)
  : transforms_(std::move(transforms))
{
}

template<typename T>
Isometry3<T> DHChain::getFK(const Eigen::Matrix<T, Eigen::Dynamic, 1> &joint_values) const
{
  Isometry3<T> transform(Isometry3<T>::Identity());
  for (Eigen::Index i = 0; i < joint_values.size(); ++i)
  {
    transform = transform * transforms_.at(i)->createRelativeTransform(joint_values[i]);
  }
  return transform;
}

template<typename T>
Isometry3<T> DHChain::getFK(const Eigen::Matrix<T, Eigen::Dynamic, 1>& joint_values,
                            const Eigen::Matrix<T, Eigen::Dynamic, 4>& offsets) const
{
  Isometry3<T> transform(Isometry3<T>::Identity());
  for (Eigen::Index i = 0; i < joint_values.size(); ++i)
  {
    const Eigen::Matrix<T, 1, 4> &offset = offsets.row(i);
    transform = transform * transforms_.at(i)->createRelativeTransform(joint_values[i], offset);
  }
  return transform;
}

inline Eigen::VectorXd DHChain::createUniformlyRandomPose() const
{
  Eigen::VectorXd joints(transforms_.size());
  for (std::size_t i = 0; i < transforms_.size(); ++i)
  {
    joints[i] = transforms_[i]->createRandomJointValue();
  }
  return joints;
}

} // namespace rct_optimizations
