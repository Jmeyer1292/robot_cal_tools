#ifndef RCT_OPTIMIZATIONS_TESTS_UTILITIES_H
#define RCT_OPTIMIZATIONS_TESTS_UTILITIES_H

#include <rct_optimizations/types.h>
#include <rct_optimizations/experimental/dh_robot.h>

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
 * @brief Creates a DH parameter-based robot representation of an ABB IRB2400
 * @return
 */
//DHRobot createABBIRB2400();

/**
 * @brief A sample grid target for test purposes
 */
struct Target
{
  Target() = default;

  Target(const unsigned rows, const unsigned cols, const double spacing)
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
  }

  std::vector<Eigen::Vector3d> points;
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


inline DHRobot createABBIRB2400()
{
  std::vector<DHTransform::Ptr> joints;
  joints.reserve(6);


  std::array<double, 4> t1 = {0.615, 0.0, 0.100, -M_PI / 2.0};
  std::array<double, 4> t2 = {0.0, -M_PI / 2.0, 0.705, 0.0};
  std::array<double, 4> t3 = {0.0, 0.0, 0.135, -M_PI / 2.0};
  std::array<double, 4> t4 = {0.755, 0.0, 0.0, M_PI / 2.0};
  std::array<double, 4> t5 = {0.0, 0.0, 0.0, -M_PI / 2.0};
  std::array<double, 4> t6 = {0.085, M_PI, 0.0, 0.0};

  joints.push_back(std::make_unique<DHTransform>(t1, DHJointType::REVOLUTE));
  joints.push_back(std::make_unique<DHTransform>(t2, DHJointType::REVOLUTE));
  joints.push_back(std::make_unique<DHTransform>(t3, DHJointType::REVOLUTE));
  joints.push_back(std::make_unique<DHTransform>(t4, DHJointType::REVOLUTE));
  joints.push_back(std::make_unique<DHTransform>(t5, DHJointType::REVOLUTE));
  joints.push_back(std::make_unique<DHTransform>(t6, DHJointType::REVOLUTE));

  return DHRobot(std::move(joints));
}

inline DHRobot createABBIRB2400WithNoise()
{
  std::vector<DHTransform::Ptr> joints;
  joints.reserve(6);

  // Noise parameters: 1.0 degree standard deviation per joint, centered on 0.0
  double mean = 0.0;
  double std_dev = 1.0 * M_PI / 180.0;

  std::array<double, 4> t1 = {0.615, 0.0, 0.100, -M_PI / 2.0};
  std::array<double, 4> t2 = {0.0, -M_PI / 2.0, 0.705, 0.0};
  std::array<double, 4> t3 = {0.0, 0.0, 0.135, -M_PI / 2.0};
  std::array<double, 4> t4 = {0.755, 0.0, 0.0, M_PI / 2.0};
  std::array<double, 4> t5 = {0.0, 0.0, 0.0, -M_PI / 2.0};
  std::array<double, 4> t6 = {0.085, M_PI, 0.0, 0.0};

  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(t1, DHJointType::REVOLUTE, mean, std_dev));
  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(t2, DHJointType::REVOLUTE, mean, std_dev));
  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(t3, DHJointType::REVOLUTE, mean, std_dev));
  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(t4, DHJointType::REVOLUTE, mean, std_dev));
  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(t5, DHJointType::REVOLUTE, mean, std_dev));
  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(t6, DHJointType::REVOLUTE, mean, std_dev));

  return DHRobot(std::move(joints));
}

} // namespace test
} // namespace rct_optimizations

#endif // RCT_OPTIMIZATIONS_TESTS_UTILITIES_H
