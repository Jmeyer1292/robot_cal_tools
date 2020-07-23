#include <gtest/gtest.h>
#include <rct_optimizations/dh_chain_kinematic_calibration.h>
#include <rct_optimizations_tests/dh_chain_observation_creator.h>
#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <rct_optimizations/solver.h>

using namespace rct_optimizations;

class DHChainKinematicCalibration : public ::testing::Test
{
  public:
  DHChainKinematicCalibration()
    : camera(test::makeKinectCamera())
    , target(5, 7, 0.025)
    , n_observations(30)
  {
    // Create the transform from the camera mount (i.e. robot wrist) to the camera
    camera_mount_to_camera = Eigen::Isometry3d::Identity();
    camera_mount_to_camera.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));

    // Create the transform from the target mount (in this case an empty kinematic chain) to the target
    target_mount_to_target = Eigen::Isometry3d::Identity();

    // Create the transform from the base of the camera kinematic chain to the base of the target kinematic chain
    camera_base_to_target_base = Eigen::Isometry3d::Identity();
    camera_base_to_target_base.translate(Eigen::Vector3d(0.940, 0.0, 0.0) + target.center);
  }

  test::Camera camera;
  test::Target target;
  Eigen::Isometry3d camera_mount_to_camera;
  Eigen::Isometry3d target_mount_to_target;
  Eigen::Isometry3d camera_base_to_target_base;
  std::size_t n_observations;
};

TEST_F(DHChainKinematicCalibration, TestCostFunction)
{
  // Create the kinematic chains
  DHChain camera_chain = test::createABBIRB2400();
  DHChain target_chain({});

  // Initialize the optimization variables
  // Camera mount to camera (cm_to_c) quaternion and translation
  Eigen::Vector3d t_cm_to_c(camera_mount_to_camera.translation());
  Eigen::AngleAxisd rot_cm_to_c(camera_mount_to_camera.rotation());
  Eigen::Vector3d aa_cm_to_c(rot_cm_to_c.angle() * rot_cm_to_c.axis());

  // Target mount to target (cm_to_c) quaternion and translation
  Eigen::Vector3d t_tm_to_t(target_mount_to_target.translation());
  Eigen::AngleAxisd rot_tm_to_t(target_mount_to_target.rotation());
  Eigen::Vector3d aa_tm_to_t(rot_tm_to_t.angle() * rot_tm_to_t.axis());

  // Camera chain base to target_chain_base (ccb_to_tcb) quaternion and translation
  Eigen::Vector3d t_ccb_to_tcb(camera_base_to_target_base.translation());
  Eigen::AngleAxisd rot_ccb_to_tcb(camera_base_to_target_base.rotation());
  Eigen::Vector3d aa_ccb_to_tcb(rot_ccb_to_tcb.angle() * rot_ccb_to_tcb.axis());

  // Create containers for the kinematic chain DH offsets
  // Ceres will not work with parameter blocks of size zero, so create a dummy set of DH offsets for chains with DoF == 0
  Eigen::MatrixX4d camera_chain_dh_offsets = Eigen::MatrixX4d::Zero(camera_chain.dof(), 4);
  Eigen::MatrixX4d target_chain_dh_offsets = Eigen::MatrixX4d::Zero(target_chain.dof(), 4);

  // Create a vector of the pointers to the optimization variables in the order that the cost function expects them
  std::vector<double *> parameters
    = DualDHChainCost2D3D::constructParameters(camera_chain_dh_offsets,
                                               target_chain_dh_offsets,
                                               t_cm_to_c,
                                               aa_cm_to_c,
                                               t_tm_to_t,
                                               aa_tm_to_t,
                                               t_ccb_to_tcb,
                                               aa_ccb_to_tcb);

  // Create observations
  KinObservation2D3D::Set observations = test::createKinematicObservations(camera_chain,
                                                                           target_chain,
                                                                           camera_mount_to_camera,
                                                                           target_mount_to_target,
                                                                           camera_base_to_target_base,
                                                                           target,
                                                                           camera,
                                                                           n_observations);

  // Test the cost function
  for (const auto &obs : observations)
  {
    for (const auto &corr : obs.correspondence_set)
    {
      DualDHChainCost2D3D cost(corr.in_image,
                               corr.in_target,
                               camera.intr,
                               camera_chain,
                               target_chain,
                               obs.camera_chain_joints,
                               obs.target_chain_joints);

      double residual[2];
      EXPECT_TRUE(cost(parameters.data(), residual));
      EXPECT_LT(residual[0], 1.0e-12);
      EXPECT_LT(residual[1], 1.0e-12);
    }
  }
}

TEST_F(DHChainKinematicCalibration, TestCalibrationPerfectGuessPerfectDH)
{
  // Create the problem with the camera chain and target chain
  // Camera chain: 6-DOF robot
  // Target chain: empty
  KinematicCalibrationProblem2D3D problem(test::createABBIRB2400(), DHChain({}));
  problem.intr = camera.intr;

  problem.observations = test::createKinematicObservations(problem.camera_chain,
                                                           problem.target_chain,
                                                           camera_mount_to_camera,
                                                           target_mount_to_target,
                                                           camera_base_to_target_base,
                                                           target,
                                                           camera,
                                                           n_observations);

  // Give the problem perfect guesses regarding the transforms
  problem.camera_mount_to_camera_guess = camera_mount_to_camera;
  problem.target_mount_to_target_guess = target_mount_to_target;
  problem.camera_base_to_target_base_guess = camera_base_to_target_base;

  // Solve the problem
  KinematicCalibrationResult result = optimize(problem);

  EXPECT_TRUE(result.converged);
  EXPECT_LT(result.initial_cost_per_obs, 1.0e-15);
  EXPECT_LT(result.final_cost_per_obs, 1.0e-15);

  EXPECT_TRUE(result.camera_mount_to_camera.isApprox(camera_mount_to_camera));
  EXPECT_TRUE(result.target_mount_to_target.isApprox(target_mount_to_target));
  EXPECT_TRUE(result.camera_base_to_target_base.isApprox(camera_base_to_target_base));

  // Expect the resulting DH parameter offsets to be the same shape as the input
  // This is an important check of the case where the cost function modifies for 0 DoF chains
  EXPECT_EQ(problem.camera_chain.dof(), result.camera_chain_dh_offsets.rows());
  EXPECT_EQ(problem.target_chain.dof(), result.target_chain_dh_offsets.rows());

  // Expect the camera DH offsets to remain the same as the input (i.e. all zeros)
  for (Eigen::Index row = 0; row < result.camera_chain_dh_offsets.rows(); ++row)
  {
    for (Eigen::Index col = 0; col < result.camera_chain_dh_offsets.cols(); ++col)
    {
      EXPECT_LT(result.camera_chain_dh_offsets(row, col), 1.0e-15);
    }
  }

  std::cout << result.covariance.toString() << std::endl;
  std::cout << result.covariance.printCorrelationCoeffAboveThreshold(0.3) << std::endl;
}

TEST_F(DHChainKinematicCalibration, TestCalibrationPerfectGuessPerturbedDH)
{
  // Create the problem with the camera chain and target chain
  // Camera chain: 6-DOF robot
  // Target chain: empty
  DHChain nominal_chain = test::createABBIRB2400();

  KinematicCalibrationProblem2D3D problem(test::perturbDHCHain(nominal_chain, 1.0e-3), DHChain({}));
  problem.intr = camera.intr;
  problem.observations = test::createKinematicObservations(nominal_chain,
                                                           problem.target_chain,
                                                           camera_mount_to_camera,
                                                           target_mount_to_target,
                                                           camera_base_to_target_base,
                                                           target,
                                                           camera,
                                                           n_observations);

  // Give the problem perfect guesses regarding the transforms
  problem.camera_mount_to_camera_guess = camera_mount_to_camera;
  problem.target_mount_to_target_guess = target_mount_to_target;
  problem.camera_base_to_target_base_guess = camera_base_to_target_base;

  // Solve the problem
  KinematicCalibrationResult result = optimize(problem);

  EXPECT_TRUE(result.converged);

  // Expect the final residual to be less than 0.5 pixels
  EXPECT_LT(result.final_cost_per_obs, 0.5*0.5);

  // Test the result by moving the robot around to a lot of positions and seeing of the results match
  DHChain optimized_chain = test::createChain(problem.camera_chain.getDHTable() + result.camera_chain_dh_offsets,
                                              problem.camera_chain.getJointTypes());

  namespace ba = boost::accumulators;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> pos_acc;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> ori_acc;

  for (std::size_t i = 0; i < 30; ++i)
  {
    Eigen::VectorXd random_pose = nominal_chain.createUniformlyRandomPose();

    // Nominal chain
    Eigen::Isometry3d nominal_fk = nominal_chain.getFK(random_pose) * camera_mount_to_camera;

    // Optimized chain
    Eigen::Isometry3d optimized_fk = optimized_chain.getFK(random_pose)
                                     * result.camera_mount_to_camera;

    // Compare
    Eigen::Isometry3d diff = nominal_fk.inverse() * optimized_fk;
    pos_acc(diff.translation().norm());
    ori_acc(Eigen::Quaterniond(nominal_fk.linear()).angularDistance(Eigen::Quaterniond(optimized_fk.linear())));
  }

  std::cout << "Pos. Mean: " << ba::mean(pos_acc) << std::endl;
  std::cout << "Pos. Stdev: " << std::sqrt(ba::variance(pos_acc)) << std::endl;

  std::cout << "Ori. Mean: " << ba::mean(ori_acc) << std::endl;
  std::cout << "Ori. Stdev: " << std::sqrt(ba::variance(ori_acc)) << std::endl;
}

TEST_F(DHChainKinematicCalibration, TestCalibrationPerturbedGuessPerturbedDH)
{
  // Create the problem with the camera chain and target chain
  // Camera chain: 6-DOF robot
  // Target chain: empty
  DHChain nominal_chain = test::createABBIRB2400();

  KinematicCalibrationProblem2D3D problem(test::perturbDHCHain(nominal_chain, 1.0e-3), DHChain({}));
  problem.intr = camera.intr;
  problem.observations = test::createKinematicObservations(nominal_chain,
                                                           problem.target_chain,
                                                           camera_mount_to_camera,
                                                           target_mount_to_target,
                                                           camera_base_to_target_base,
                                                           target,
                                                           camera,
                                                           n_observations);

  // Give the problem perfect guesses regarding the transforms
  problem.camera_mount_to_camera_guess = test::perturbPose(camera_mount_to_camera, 0.05, 0.05);
  problem.target_mount_to_target_guess = test::perturbPose(target_mount_to_target, 0.05, 0.05);
  problem.camera_base_to_target_base_guess = test::perturbPose(camera_base_to_target_base, 0.05, 0.05);

  // Solve the problem
  KinematicCalibrationResult result = optimize(problem);

  EXPECT_TRUE(result.converged);

  // Expect the final residual to be less than 0.5 pixels
  EXPECT_LT(result.final_cost_per_obs, 0.5*0.5);

  // Test the result by moving the robot around to a lot of positions and seeing of the results match
  DHChain optimized_chain = test::createChain(problem.camera_chain.getDHTable() + result.camera_chain_dh_offsets,
                                              problem.camera_chain.getJointTypes());

  namespace ba = boost::accumulators;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> pos_acc;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> ori_acc;

  for (std::size_t i = 0; i < 30; ++i)
  {
    Eigen::VectorXd random_pose = nominal_chain.createUniformlyRandomPose();

    // Nominal chain
    Eigen::Isometry3d nominal_fk = nominal_chain.getFK(random_pose) * camera_mount_to_camera;

    // Optimized chain
    Eigen::Isometry3d optimized_fk = optimized_chain.getFK(random_pose)
                                     * result.camera_mount_to_camera;

    // Compare
    Eigen::Isometry3d diff = nominal_fk.inverse() * optimized_fk;
    pos_acc(diff.translation().norm());
    ori_acc(Eigen::Quaterniond(nominal_fk.linear()).angularDistance(Eigen::Quaterniond(optimized_fk.linear())));
  }

  std::cout << "Pos. Mean: " << ba::mean(pos_acc) << std::endl;
  std::cout << "Pos. Stdev: " << std::sqrt(ba::variance(pos_acc)) << std::endl;

  std::cout << "Ori. Mean: " << ba::mean(ori_acc) << std::endl;
  std::cout << "Ori. Stdev: " << std::sqrt(ba::variance(ori_acc)) << std::endl;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
