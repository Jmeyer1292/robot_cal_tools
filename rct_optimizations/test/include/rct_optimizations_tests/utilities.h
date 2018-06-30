#ifndef RCT_OPTIMIZATIONS_TESTS_UTILITIES_H
#define RCT_OPTIMIZATIONS_TESTS_UTILITIES_H

#include <rct_optimizations/types.h>


namespace rct_optimizations
{
namespace test
{

// a test pin-hole camera that has intrinsics, but also image size data needed for generating
// simulated information
struct Camera
{
  CameraIntrinsics intr;
  int width;
  int height;
};

// Create a test camera set with kinect-like parameters
Camera makeKinectCamera();

// A sample grid target for test purposes
struct Target
{
  std::vector<Eigen::Vector3d> points;
};

Target makeTarget(int rows, int cols, double spacing);

// Helper
CorrespondenceSet zip(const Target& target, const std::vector<Eigen::Vector2d>& image_obs);

Eigen::Affine3d perturbPose(const Eigen::Affine3d& pose, double spatial_noise, double angle_noise);

}
}

#endif // RCT_OPTIMIZATIONS_TESTS_UTILITIES_H
