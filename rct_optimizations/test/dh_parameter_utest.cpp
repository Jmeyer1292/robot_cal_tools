#include <rct_optimizations/dh_chain.h>
#include <rct_optimizations_tests/dh_chain_observation_creator.h>
#include <gtest/gtest.h>

using namespace rct_optimizations;

TEST(DHChain, FKTest)
{
  DHChain robot = test::createABBIRB2400();
  Eigen::Isometry3d transform = robot.getFK(std::vector<double>(6, 0.0));
  std::cout << transform.matrix() << std::endl;

  Eigen::Isometry3d desired(Eigen::Isometry3d::Identity());
  desired.translate(Eigen::Vector3d(0.940, 0.0, 1.455));
  desired.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
  std::cout << desired.matrix() << std::endl;

  EXPECT_TRUE(transform.isApprox(desired, std::numeric_limits<double>::epsilon()));
}

TEST(DHChain, NoisyFKTest)
{
  DHChain robot = test::createABBIRB2400WithNoise();
  Eigen::Isometry3d transform = robot.getFK(std::vector<double>(6, 0.0));
  std::cout << "FK transform with noise\n" << transform.matrix() << std::endl;

  Eigen::Isometry3d desired(Eigen::Isometry3d::Identity());
  desired.translate(Eigen::Vector3d(0.940, 0.0, 1.455));
  desired.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
  std::cout << "Desired FK transform \n" << desired.matrix() << std::endl;

  EXPECT_FALSE(transform.isApprox(desired, std::numeric_limits<double>::epsilon()));
}

TEST(DHChain, FKWithJointSubsetTest)
{
  DHChain robot = test::createABBIRB2400();
  for (std::size_t i = 0; i < 6; ++i)
  {
    EXPECT_NO_THROW(robot.getFK(std::vector<double>(i, 0.0)));
  }
  EXPECT_THROW(robot.getFK(std::vector<double>(7, 0.0)), std::out_of_range);
}

TEST(DHChain, FKWithJointSubsetPointerTest)
{
  DHChain robot = test::createABBIRB2400();
  for (std::size_t i = 0; i < 6; ++i)
  {
    std::vector<double> joints(i, 0.0);
    EXPECT_NO_THROW(robot.getFK(joints.data(), joints.size()));
  }
  std::vector<double> joints(7, 0.0);
  EXPECT_THROW(robot.getFK(joints.data(), joints.size()), std::out_of_range);
}

TEST(DHChain, generateObservations3D)
{
  const std::size_t n = 100;
  auto observations = test::create(test::createABBIRB2400(),
                                   DHChain({}),
                                   Eigen::Isometry3d::Identity(),
                                   Eigen::Isometry3d::Identity(),
                                   Eigen::Isometry3d::Identity(),
                                   test::Target(5, 5, 0.025),
                                   n);
  EXPECT_EQ(observations.size(), n);
}

TEST(DHChain, generateObservations2D)
{
  DHChain camera_robot = test::createABBIRB2400();
  DHChain target_robot({});

  // Create a transform to the tool0 of the robot at it's all-zero position
  Eigen::Isometry3d camera_base_to_target_base = camera_robot.getFK(std::vector<double>(6, 0.0));

  // Place the target 0.5m in front of the robot's all-zero position, facing back at the robot
  camera_base_to_target_base.translate(Eigen::Vector3d(0.0, 0.0, 0.5));
  camera_base_to_target_base.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));

  const std::size_t n = 100;
  const test::Target target(5, 5, 0.025);
  const test::Camera camera = test::makeKinectCamera();
  Observation2D3D::Set observations;
  EXPECT_NO_THROW(observations = test::create(camera_robot,
                                              target_robot,
                                              Eigen::Isometry3d::Identity(),
                                              Eigen::Isometry3d::Identity(),
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
