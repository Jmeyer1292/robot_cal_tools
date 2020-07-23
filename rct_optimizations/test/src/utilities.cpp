#include "rct_optimizations_tests/utilities.h"
#include <random>

namespace rct_optimizations
{
namespace test
{
Camera makeKinectCamera()
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

Eigen::Isometry3d perturbPose(const Eigen::Isometry3d& pose, double spatial_noise, double angle_noise)
{
  auto mt_rand = std::mt19937(std::random_device{}());

  auto spatial_dist = std::bind(std::uniform_real_distribution<double>{-spatial_noise, spatial_noise}, mt_rand);
  auto angle_dist = std::bind(std::uniform_real_distribution<double>{-angle_noise, angle_noise}, mt_rand);
  auto one_to_one = std::bind(std::uniform_real_distribution<double>{-1, 1}, mt_rand);

  Eigen::Vector3d translation (spatial_dist(), spatial_dist(), spatial_dist());
  Eigen::Vector3d rot_axis (one_to_one(), one_to_one(), one_to_one());
  rot_axis.normalize();

  double angle = angle_dist();

  Eigen::Isometry3d new_pose = pose * Eigen::Translation3d(translation) * Eigen::AngleAxisd(angle, rot_axis);

  return new_pose;
}

DHChain createChain(const Eigen::MatrixXd &dh, const std::vector<DHJointType> &joint_types)
{
  if (dh.rows() != joint_types.size())
    throw OptimizationException("DH table rows does not match joint types size");

  std::vector<DHTransform> joints;
  joints.reserve(joint_types.size());
  for (std::size_t i = 0; i < joint_types.size(); ++i)
  {
    joints.push_back(DHTransform(dh.row(i), joint_types.at(i)));
  }

  return DHChain(joints);
}

DHChain createABBIRB2400()
{
  std::vector<DHTransform> joints;
  joints.reserve(6);

  Eigen::Vector4d t1, t2, t3, t4, t5, t6;

  t1 << 0.615, 0.0, 0.100, -M_PI / 2.0;
  t2 << 0.0, -M_PI / 2.0, 0.705, 0.0;
  t3 << 0.0, 0.0, 0.135, -M_PI / 2.0;
  t4 << 0.755, 0.0, 0.0, M_PI / 2.0;
  t5 << 0.0, 0.0, 0.0, -M_PI / 2.0;
  t6 << 0.085, M_PI, 0.0, 0.0;

  joints.push_back(DHTransform(t1, DHJointType::REVOLUTE, "j1"));
  joints.push_back(DHTransform(t2, DHJointType::REVOLUTE, "j2"));
  joints.push_back(DHTransform(t3, DHJointType::REVOLUTE, "j3"));
  joints.push_back(DHTransform(t4, DHJointType::REVOLUTE, "j4"));
  joints.push_back(DHTransform(t5, DHJointType::REVOLUTE, "j5"));
  joints.push_back(DHTransform(t6, DHJointType::REVOLUTE, "j6"));

  return DHChain(joints);
}

DHChain perturbDHCHain(const DHChain &in, const double stddev)
{
  std::mt19937 mt_rand(std::random_device{}());
  std::normal_distribution<double> norm(0.0, stddev);

  // Get the joint types and nominal DH table
  std::vector<DHJointType> joint_types = in.getJointTypes();
  Eigen::MatrixX4d dh = in.getDHTable();

  // Perturb each value in the DH table randomly by the input standard deviation
  dh = dh.unaryExpr([&norm, &mt_rand](const double val) { return val + norm(mt_rand); });

  std::vector<DHTransform> transforms;
  transforms.reserve(joint_types.size());
  for (std::size_t i = 0; i < joint_types.size(); ++i)
  {
    transforms.push_back(DHTransform(dh.row(i), joint_types[i]));
  }
  return DHChain(transforms);
}

} // namespace test
} // namespace rct_optimizations



