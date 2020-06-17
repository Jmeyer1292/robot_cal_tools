#include <rct_optimizations/dh_chain.h>
#include <random>

namespace rct_optimizations
{
// DH Transform

DHTransform::DHTransform(const Eigen::Vector4d& params_, DHJointType type_)
  : params(params_)
  , type(type_)
{
}

double DHTransform::createRandomJointValue() const
{
  std::mt19937 mt_rand = std::mt19937(std::random_device{}());
  std::uniform_real_distribution<double> dist(min, max);
  return dist(mt_rand);
}

// DH Chain

DHChain::DHChain(std::vector<DHTransform> transforms)
  : transforms_(std::move(transforms))
{
}

Eigen::VectorXd DHChain::createUniformlyRandomPose() const
{
  Eigen::VectorXd joints(transforms_.size());
  for (std::size_t i = 0; i < transforms_.size(); ++i)
  {
    joints[i] = transforms_[i].createRandomJointValue();
  }
  return joints;
}

std::size_t DHChain::dof() const
{
  return transforms_.size();
}

Eigen::MatrixX4d DHChain::getDHTable() const
{
  Eigen::MatrixX4d out(dof(), 4);
  for (std::size_t i = 0; i < transforms_.size(); ++i)
  {
    out.row(i) = transforms_[i].params.transpose();
  }
  return out;
}

std::vector<DHJointType> DHChain::getJointTypes() const
{
  std::vector<DHJointType> out;
  out.reserve(transforms_.size());
  for (const auto &t : transforms_)
  {
    out.push_back(t.type);
  }
  return out;
}

} // namespace rct_optimizations
