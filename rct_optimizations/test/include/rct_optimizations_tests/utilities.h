#ifndef RCT_OPTIMIZATIONS_TESTS_UTILITIES_H
#define RCT_OPTIMIZATIONS_TESTS_UTILITIES_H

#include <rct_optimizations/types.h>


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

}
}

#endif // RCT_OPTIMIZATIONS_TESTS_UTILITIES_H
