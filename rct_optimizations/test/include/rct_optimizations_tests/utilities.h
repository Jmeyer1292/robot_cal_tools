#ifndef RCT_OPTIMIZATIONS_TESTS_UTILITIES_H
#define RCT_OPTIMIZATIONS_TESTS_UTILITIES_H

#include <rct_optimizations/types.h>
#include <rct_optimizations/dh_chain.h>

namespace rct_optimizations
{
namespace test
{

/**
 * @brief a test pin-hole camera that has intrinsics, but also image size data needed for generating simulated information
 */
struct Camera
{
  CameraIntrinsics intr;
  int width;
  int height;
};

/**
 * @brief Create a test camera set with kinect-like parameters
 * @return
 */
Camera makeKinectCamera();

/**
 * @brief A sample grid target for test purposes
 * Looking down at the target
 *   - The origin is in the lower left corner
 *   - The x coordinate defines the feature column
 *   - The y coordinate defines the feature row
 *   - The points are ordered row-wise from the top left corner to the bottom right corner
 */
struct Target
{
  Target() = default;

  Target(const unsigned rows, const unsigned cols, const double spacing)
    : origin_idx((rows - 1) * cols)
    , center(double(rows - 1) * spacing / 2.0, double(cols - 1) * spacing / 2.0, 0.0)
  {
    points.reserve(rows * cols);

    for (unsigned i = 1; i < (rows + 1); i++)
    {
      double y = (rows - i) * spacing;
      for (unsigned j = 0; j < cols; j++)
      {
        double x = j * spacing;
        Eigen::Vector3d point(x, y, 0.0);
        points.push_back(point);
      }
    }

    origin_idx = (rows - 1) * cols;
  }

  std::vector<Eigen::Vector3d> points;
  std::size_t origin_idx;
  Eigen::Vector3d center;
};

/**
 * @brief perturbPose
 * @param pose
 * @param spatial_noise
 * @param angle_noise
 * @return
 */
Eigen::Isometry3d perturbPose(const Eigen::Isometry3d &pose,
                              double spatial_noise,
                              double angle_noise);

/**
 * @brief Creates a DH parameter-based robot representation of an ABB IRB2400
 * @return
 */
inline DHChain createABBIRB2400()
{
  std::vector<DHTransform::Ptr> joints;
  joints.reserve(6);

  Eigen::Vector4d t1, t2, t3, t4, t5, t6;

  t1 << 0.615, 0.0, 0.100, -M_PI / 2.0;
  t2 << 0.0, -M_PI / 2.0, 0.705, 0.0;
  t3 << 0.0, 0.0, 0.135, -M_PI / 2.0;
  t4 << 0.755, 0.0, 0.0, M_PI / 2.0;
  t5 << 0.0, 0.0, 0.0, -M_PI / 2.0;
  t6 << 0.085, M_PI, 0.0, 0.0;

  joints.push_back(std::make_unique<DHTransform>(t1, DHJointType::REVOLUTE));
  joints.push_back(std::make_unique<DHTransform>(t2, DHJointType::REVOLUTE));
  joints.push_back(std::make_unique<DHTransform>(t3, DHJointType::REVOLUTE));
  joints.push_back(std::make_unique<DHTransform>(t4, DHJointType::REVOLUTE));
  joints.push_back(std::make_unique<DHTransform>(t5, DHJointType::REVOLUTE));
  joints.push_back(std::make_unique<DHTransform>(t6, DHJointType::REVOLUTE));

  return DHChain(std::move(joints));
}

inline DHChain perturbDHCHain(const DHChain &in, const double stddev)
{
  std::mt19937 mt_rand(std::random_device{}());
  std::normal_distribution<double> norm(0.0, stddev);

  // Get the joint types and nominal DH table
  std::vector<DHJointType> joint_types = in.getJointTypes();
  Eigen::MatrixX4d dh = in.getDHTable();

  // Perturb each value in the DH table randomly by the input standard deviation
  dh = dh.unaryExpr([&norm, &mt_rand](const double val) { return val + norm(mt_rand); });

  std::vector<DHTransform::Ptr> transforms;
  transforms.reserve(joint_types.size());
  for (std::size_t i = 0; i < joint_types.size(); ++i)
  {
    transforms.push_back(std::make_unique<DHTransform>(dh.row(i), joint_types[i]));
  }
  return DHChain(std::move(transforms));
}

} // namespace test
} // namespace rct_optimizations

#endif // RCT_OPTIMIZATIONS_TESTS_UTILITIES_H
