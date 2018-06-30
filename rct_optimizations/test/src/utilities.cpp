#include "rct_optimizations_tests/utilities.h"
#include <random>

static void makePoints(int rows, int cols, double spacing, std::vector<Eigen::Vector3d>& points)
{
  points.reserve(rows * cols);

  for (int i = 1; i < (rows + 1); i++)
  {
    double y = (rows - i) * spacing;
    for (int j = 0; j < cols; j++)
    {
      double x = j * spacing;
      Eigen::Vector3d point(x, y, 0.0);
      points.push_back(point);
    }
  }
}

rct_optimizations::test::Camera rct_optimizations::test::makeKinectCamera()
{
  CameraIntrinsics intr;
  intr.fx() = 550.0;
  intr.fy() = 550.0;
  intr.cx() = 640/2;
  intr.cy() = 480/2;

  Camera camera;
  camera.intr = intr;
  camera.width = 640;
  camera.height = 480;

  return camera;
}

rct_optimizations::test::Target rct_optimizations::test::makeTarget(int rows, int cols, double spacing)
{
  Target target;
  makePoints(rows, cols, spacing, target.points);
  return target;
}

rct_optimizations::CorrespondenceSet rct_optimizations::test::zip(const rct_optimizations::test::Target& target,
                                                                  const std::vector<Eigen::Vector2d>& image_obs)
{
  if (target.points.size() != image_obs.size()) throw std::invalid_argument("target points != image obs points");

  CorrespondenceSet out;
  for (std::size_t i = 0; i < target.points.size(); ++i)
  {
    Correspondence2D3D c;
    c.in_image = image_obs[i];
    c.in_target = target.points[i];
    out.push_back(c);
  }
  return out;
}

Eigen::Affine3d rct_optimizations::test::perturbPose(const Eigen::Affine3d& pose, double spatial_noise, double angle_noise)
{
  std::random_device dev;
  std::default_random_engine eng (dev());

  std::uniform_real_distribution<double> spatial_dist (-spatial_noise, spatial_noise);
  std::uniform_real_distribution<double> angle_dist (-angle_noise, angle_noise);
  std::uniform_real_distribution<double> one_to_one (-1, 1);

  Eigen::Vector3d translation (spatial_dist(eng), spatial_dist(eng), spatial_dist(eng));
  Eigen::Vector3d rot_axis (one_to_one(eng), one_to_one(eng), one_to_one(eng));
  rot_axis.normalize();

  double angle = angle_dist(eng);

  Eigen::Affine3d new_pose = pose * Eigen::Translation3d(translation) * Eigen::AngleAxisd(angle, rot_axis);

  return new_pose;
}
