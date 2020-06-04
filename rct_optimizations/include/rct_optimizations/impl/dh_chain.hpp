#pragma once
#include <rct_optimizations/dh_chain.h>
#include <random>

namespace rct_optimizations
{
// DH Transform

inline DHTransform::DHTransform(std::array<double, 4> params_, DHJointType type_)
  : params(std::move(params_))
  , type(type_)
{
}

template<typename T>
Isometry3<T> DHTransform::createRelativeTransform(const T joint_value,
                                                  const T* offsets) const
{
  Isometry3<T> transform(Isometry3<T>::Identity());

  // Create aliases for the d and theta parameters such that the joint value could be added to them appropriately
  T d = T(params[0]) + offsets[0];
  T theta = T(params[1]) + offsets[1];
  T r = T(params[2]) + offsets[2];
  T alpha = T(params[3]) + offsets[3];
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
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  transform *= Eigen::Translation<T, 3>(T(0.0), T(0.0), d);
  transform.rotate(Eigen::AngleAxis<T>(theta, Vector3::UnitZ()));
  transform *= Eigen::Translation<T, 3>(r, T(0.0), T(0.0));
  transform.rotate(Eigen::AngleAxis<T>(alpha, Vector3::UnitX()));

  return transform;
}

template<typename T>
inline Isometry3<T> DHTransform::createRelativeTransform(const T joint) const
{
  std::array<T, 4> offsets = {T(0.0), T(0.0), T(0.0), T(0.0)};
  return DHTransform::createRelativeTransform(joint, offsets.data());
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
                                   const T* const* offsets) const
{
  Isometry3<T> transform(Isometry3<T>::Identity());
  for (std::size_t i = 0; i < joint_values.size(); ++i)
  {
    transform = transform * transforms_.at(i)->createRelativeTransform(joint_values[i], offsets[i]);
  }
  return transform;
}

inline Eigen::Isometry3d DHChain::createUniformlyRandomPose() const
{
  Eigen::VectorXd joints(transforms_.size());
  for (std::size_t i = 0; i < transforms_.size(); ++i)
  {
    joints[i] = transforms_[i]->createRandomJointValue();
  }
  return getFK(joints);
}

} // namespace rct_optimizations
