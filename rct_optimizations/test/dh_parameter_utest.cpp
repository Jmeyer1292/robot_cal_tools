#include <rct_optimizations/experimental/dh_robot.h>
#include <rct_optimizations_tests/robot_observation_creator.h>
#include <gtest/gtest.h>

using namespace rct_optimizations;

DHRobot createABBIRB2400()
{
  std::vector<DHTransform::Ptr> joints;
  joints.reserve(6);

  joints.push_back(
    std::make_unique<DHTransform>(DHJointType::REVOLUTE, 0.615, 0.0, 0.100, -M_PI / 2.0));
  joints.push_back(
    std::make_unique<DHTransform>(DHJointType::REVOLUTE, 0.0, -M_PI / 2.0, 0.705, 0.0));
  joints.push_back(
    std::make_unique<DHTransform>(DHJointType::REVOLUTE, 0.0, 0.0, 0.135, -M_PI / 2.0));
  joints.push_back(
    std::make_unique<DHTransform>(DHJointType::REVOLUTE, 0.755, 0.0, 0.0, M_PI / 2.0));
  joints.push_back(std::make_unique<DHTransform>(DHJointType::REVOLUTE, 0.0, 0.0, 0.0, -M_PI / 2.0));
  joints.push_back(std::make_unique<DHTransform>(DHJointType::REVOLUTE, 0.085, M_PI, 0.0, 0.0));

  return DHRobot(std::move(joints));
}

DHRobot createABBIRB2400WithNoise()
{
  std::vector<DHTransform::Ptr> joints;
  joints.reserve(6);

  // Noise parameters: 1.0 degree standard deviation per joint, centered on 0.0
  double mean = 0.0;
  double std_dev = 1.0 * M_PI / 180.0;

  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(
    DHJointType::REVOLUTE, 0.615, 0.0, 0.100, -M_PI / 2.0, mean, std_dev));
  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(
    DHJointType::REVOLUTE, 0.0, -M_PI / 2.0, 0.705, 0.0, mean, std_dev));
  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(
    DHJointType::REVOLUTE, 0.0, 0.0, 0.135, -M_PI / 2.0, mean, std_dev));
  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(
    DHJointType::REVOLUTE, 0.755, 0.0, 0.0, M_PI / 2.0, mean, std_dev));
  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(
    DHJointType::REVOLUTE, 0.0, 0.0, 0.0, -M_PI / 2.0, mean, std_dev));
  joints.push_back(std::make_unique<GaussianNoiseDHTransform>(
    DHJointType::REVOLUTE, 0.085, M_PI, 0.0, 0.0, mean, std_dev));

  return DHRobot(std::move(joints));
}

TEST(DHRobot, FKTest)
{
  DHRobot robot = createABBIRB2400();
  Eigen::Isometry3d transform = robot.getFK(std::vector<double>(6, 0.0));
  std::cout << transform.matrix() << std::endl;

  Eigen::Isometry3d desired(Eigen::Isometry3d::Identity());
  desired.translate(Eigen::Vector3d(0.940, 0.0, 1.455));
  desired.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
  std::cout << desired.matrix() << std::endl;

  EXPECT_TRUE(transform.isApprox(desired, std::numeric_limits<double>::epsilon()));
}

TEST(DHRobot, NoisyFKTest)
{
  DHRobot robot = createABBIRB2400WithNoise();
  Eigen::Isometry3d transform = robot.getFK(std::vector<double>(6, 0.0));
  std::cout << "FK transform with noise\n" << transform.matrix() << std::endl;

  Eigen::Isometry3d desired(Eigen::Isometry3d::Identity());
  desired.translate(Eigen::Vector3d(0.940, 0.0, 1.455));
  desired.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
  std::cout << "Desired FK transform \n" << desired.matrix() << std::endl;

  EXPECT_FALSE(transform.isApprox(desired, std::numeric_limits<double>::epsilon()));
}

TEST(DHRobot, FKWithJointSubsetTest)
{
  DHRobot robot = createABBIRB2400();
  for (std::size_t i = 0; i < 6; ++i)
  {
    EXPECT_NO_THROW(robot.getFK(std::vector<double>(i, 0.0)));
  }
  EXPECT_THROW(robot.getFK(std::vector<double>(7, 0.0)), std::out_of_range);
}

TEST(DHRobot, FKWithJointSubsetPointerTest)
{
  DHRobot robot = createABBIRB2400();
  for (std::size_t i = 0; i < 6; ++i)
  {
    std::vector<double> joints(i, 0.0);
    EXPECT_NO_THROW(robot.getFK(joints.data(), joints.size()));
  }
  std::vector<double> joints(7, 0.0);
  EXPECT_THROW(robot.getFK(joints.data(), joints.size()), std::out_of_range);
}

TEST(DHRobot, generateObservations3D)
{
  Eigen::Isometry3d camera_base_to_target_base(Eigen::Isometry3d::Identity());
  const std::size_t n = 100;
  auto observations = test::create(createABBIRB2400(),
                                   DHRobot({}),
                                   camera_base_to_target_base,
                                   test::Target(5, 5, 0.025),
                                   n);
  EXPECT_EQ(observations.size(), n);
}

TEST(DHRobot, generateObservations2D)
{
  // Create a transform to the tool0 of the robot at it's all-zero position
  Eigen::Isometry3d camera_base_to_target_base(Eigen::Isometry3d::Identity());
  camera_base_to_target_base.translate(Eigen::Vector3d(0.940, 0.0, 1.455));
  camera_base_to_target_base.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));

  // Place the target 0.5m in front of the robot's all-zero position, facing back at the robot
  camera_base_to_target_base.translate(Eigen::Vector3d(0.0, 0.0, 0.5));
  camera_base_to_target_base.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));

  const std::size_t n = 100;
  const test::Target target(5, 5, 0.025);
  const test::Camera camera = test::makeKinectCamera();
  Observation2D3D::Set observations;
  EXPECT_NO_THROW(observations = test::create(createABBIRB2400(),
                                              DHRobot({}),
                                              camera_base_to_target_base,
                                              test::Target(5, 5, 0.025),
                                              camera,
                                              n));
  EXPECT_GE(observations.size(), n);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
