#pragma once

#include <Eigen/Geometry>
#include <memory>
#include <utility>
#include <vector>

namespace rct_optimizations
{
/**
 * @brief The joint types for DH representation
 */
enum class DHJointType : unsigned
{
  LINEAR,
  REVOLUTE,
};

template<typename T>
using Isometry3 = Eigen::Transform<T, 3, Eigen::Isometry>;

template<typename T>
using Quaternion = Eigen::Quaternion<T>;

template<typename T>
using AngleAxis = Eigen::AngleAxis<T>;

template<typename T>
using Vector4 = Eigen::Matrix<T, 4, 1>;

template<typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template<typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;

/**
 * @brief Struct representing the DH parameters of a single transformation between adjacent links.
 * This struct follows the classical DH parameter convention: Trans[Zi-1](d) * Rot[Zi-1](theta) * Trans[Xi](r) * Rot[Xi](alpha)
 * See @link https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters for reference
 */
struct DHTransform
{
  DHTransform(const Eigen::Vector4d& params_, DHJointType type_);

  DHTransform(const Eigen::Vector4d& params_, DHJointType type_, const std::string& name_);

  /**
   * @brief Creates the homogoneous transformation from the previous link to the current link
   * @param joint_value - The joint value to apply when caluclating the transform
   * @param offsets - The DH parameter offsets to apply when calculating the transform
   * @return
   */
  template<typename T>
  Isometry3<T> createRelativeTransform(const T joint_value,
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
      throw std::runtime_error("Unknown DH joint type");
    }

    // Perform the DH transformations
    transform *= Eigen::Translation<T, 3>(T(0.0), T(0.0), updated_params(0));
    transform.rotate(Eigen::AngleAxis<T>(updated_params(1), Vector3<T>::UnitZ()));
    transform *= Eigen::Translation<T, 3>(updated_params(2), T(0.0), T(0.0));
    transform.rotate(Eigen::AngleAxis<T>(updated_params(3), Vector3<T>::UnitX()));

    return transform;
  }

  /**
   * @brief Creates the homogoneous transformation from the previous link to the current link without applying DH parameter offsets
   * @param joint_value - The joint value to apply when calculating the transform
   * @return
   */
  template<typename T>
  Isometry3<T> createRelativeTransform(const T joint_value) const
  {
    Eigen::Matrix<T, 1, 4> offsets = Eigen::Matrix<T, 1, 4>::Zero();
    return createRelativeTransform(joint_value, offsets);
  }

  double createRandomJointValue() const;

  std::array<std::string, 4> getParamLabels() const;

  /** @brief DH parameters
   *  d: The linear offset in Z
   *  theta: The rotational offset about Z
   *  r: The linear offset in X
   *  alpha: The rotational offset about X
   */
  const Eigen::Vector4d params;
  /** @brief The type of actuation of the joint */
  const DHJointType type;
  /** @brief Joint maximum value */
  double max = M_PI;
  /** @brief Joint minimum value */
  double min = -M_PI;
  /** @brief Label for this transform */
  std::string name;
};

/**
 * @brief Robot representation using DH parameters
 */
class DHChain
{
public:
  DHChain(std::vector<DHTransform> transforms,
          const Eigen::Isometry3d& base_offset = Eigen::Isometry3d::Identity());

  /**
   * @brief Calculates forward kinematics for the chain with the joints provided.
   * Note: the transform to the n-th link is calculated, where n is the size of @ref joint_values
   * @param joint_values - The joint values with which to calculate forward kinematics (size: [<= @ref dof()])
   * @return
   * @throws Exception if the size of joint values is larger than the number of DH transforms in the chain
   */
  template<typename T>
  Isometry3<T> getFK(const Eigen::Matrix<T, Eigen::Dynamic, 1> &joint_values) const
  {
    Eigen::Matrix<T, Eigen::Dynamic, 4> offsets = Eigen::Matrix<T, Eigen::Dynamic, 4>::Zero(dof(), 4);
    return getFK(joint_values, offsets);
  }

  /**
   * @brief Calculates the forward kinematics for the chain given a set of joint values and DH parameter offsets
   * Note: the transform to the n-th link is calculated, where n is the size of @ref joint_values
   * @param joint_values - The joint values with which to calculate the forward kinematics (size: [<= @ref dof()])
   * @param offsets - The DH parameter offsets to apply when calculating the forward kinematics (size: [@ref dof() x 4])
   * @return
   * @throws Exception if the size of @ref joint_values is larger than the number of DH transforms in the chain
   * or if the size of @ref joint_values is larger than the rows of DH offsets
   */
  template<typename T>
  Isometry3<T> getFK(const Eigen::Matrix<T, Eigen::Dynamic, 1>& joint_values,
                     const Eigen::Matrix<T, Eigen::Dynamic, 4>& offsets) const
  {
    if (joint_values.size() > dof())
    {
      std::stringstream ss;
      ss << "Joint values size (" << joint_values.size() << ") is larger than the chain DoF (" << dof() << ")";
      throw std::runtime_error(ss.str());
    }
    else if (joint_values.size() > offsets.rows())
    {
      std::stringstream ss;
      ss << "Joint values size (" << joint_values.size() << ") is larger than the rows of DH offsets ("
         << offsets.rows() << ")";
      throw std::runtime_error(ss.str());
    }

    Isometry3<T> transform(base_offset_.cast<T>());
    for (Eigen::Index i = 0; i < joint_values.size(); ++i)
    {
      const Eigen::Matrix<T, 1, 4> &offset = offsets.row(i);
      transform = transform * transforms_.at(i).createRelativeTransform(joint_values[i], offset);
    }
    return transform;
  }

  /**
   * @brief Creates a random joint pose by choosing a random uniformly distributed joint value for each joint in the chain
   * @return
   */
  Eigen::VectorXd createUniformlyRandomPose() const;

  /**
   * @brief Returns the number of degrees of freedom (i.e. DH transforms) of the chain
   * @return
   */
  std::size_t dof() const;

  /**
   * @brief Gets a @ref dof() x 4 matrix of the DH parameters
   * @return
   */
  Eigen::MatrixX4d getDHTable() const;

  /**
   * @brief Returns a the joint types of the kinematic chain in order
   * @return
   */
  std::vector<DHJointType> getJointTypes() const;

  /**
   * @brief Returns the labels of the DH parameters for each DH transform in the chain
   * @return
   */
  std::vector<std::array<std::string, 4>> getParamLabels() const;

protected:
  /** @brief The DH transforms that make up the chain */
  std::vector<DHTransform> transforms_;
  /** @brief Fixed transform offset to the beginning of the chain */
  Eigen::Isometry3d base_offset_;
};

} // namespace rct_optimizations
