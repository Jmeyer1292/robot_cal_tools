#include <rct_optimizations/dh_chain_kinematic_calibration.h>
#include <rct_optimizations/local_parameterization.h>
#include <rct_optimizations_tests/dh_chain_observation_creator.h>
#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <gtest/gtest.h>

using namespace rct_optimizations;

class DHChainMeasurementTest : public ::testing::Test
{
public:
  DHChainMeasurementTest()
    : camera_chain(test::createABBIRB2400())
    , target_chain({})
    , problem(camera_chain, target_chain)
  {
  }

  void SetUp() override
  {
    setActualData();
    setInitialGuess();
    setObservations();
//    applyMasks();
  }

  virtual void setActualData()
  {
    n_observations = 150;
    residual_threshold_sqr = 1.0e-12;

//    camera = test::makeKinectCamera();
//    target = test::Target(3, 4, 0.025);
//    problem.intr = camera.intr;

    // Create the transform from the camera mount (i.e. robot wrist) to the camera
    camera_mount_to_camera = Eigen::Isometry3d::Identity();
    camera_mount_to_camera.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));

    // Create the transform from the target mount (in this case an empty kinematic chain) to the target
    target_mount_to_target = Eigen::Isometry3d::Identity();

    // Create the transform from the base of the camera kinematic chain to the base of the target kinematic chain
    camera_base_to_target_base = Eigen::Isometry3d::Identity();
    camera_base_to_target_base.translate(Eigen::Vector3d(0.940, 0.0, 0.0));
  }

  virtual void setInitialGuess()
  {
    problem.camera_mount_to_camera_guess = camera_mount_to_camera;
    problem.target_mount_to_target_guess = target_mount_to_target;
    problem.camera_base_to_target_base_guess = camera_base_to_target_base;
  }

  virtual void setObservations()
  {
    EXPECT_NO_THROW(problem.observations = test::createKinematicMeasurements(problem.camera_chain,
                                                                             problem.target_chain,
                                                                             camera_mount_to_camera,
                                                                             target_mount_to_target,
                                                                             camera_base_to_target_base,
                                                                             n_observations));
  }

//  virtual void applyMasks()
//  {
//  }

  virtual void analyzeResults(const KinematicCalibrationResult& result)
  {
    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.final_cost_per_obs, residual_threshold_sqr);

    // Expect the resulting DH parameter offsets to be the same shape as the input
    // This is an important check of the case where the cost function modifies for 0 DoF chains
    EXPECT_EQ(problem.camera_chain.dof(), result.camera_chain_dh_offsets.rows());
    EXPECT_EQ(problem.target_chain.dof(), result.target_chain_dh_offsets.rows());

    // Test the result by moving the robot around to a lot of positions and seeing of the results match
    DHChain optimized_chain = test::createChain(problem.camera_chain.getDHTable() + result.camera_chain_dh_offsets,
                                                problem.camera_chain.getJointTypes());

    namespace ba = boost::accumulators;
    ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> pos_acc;
    ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> ori_acc;

    const std::size_t n = 1000;
    for (std::size_t i = 0; i < n; ++i)
    {
      Eigen::VectorXd random_pose = problem.camera_chain.createUniformlyRandomPose();

      // Nominal chain
      Eigen::Isometry3d nominal_fk = problem.camera_chain.getFK(random_pose) * camera_mount_to_camera;

      // Optimized chain
      Eigen::Isometry3d optimized_fk = optimized_chain.getFK(random_pose)
                                       * result.camera_mount_to_camera;

      // Compare
      Eigen::Isometry3d diff = nominal_fk.inverse() * optimized_fk;
      pos_acc(diff.translation().norm());
      ori_acc(Eigen::Quaterniond(nominal_fk.linear()).angularDistance(Eigen::Quaterniond(optimized_fk.linear())));
    }

    std::cout << "Position Difference Mean: " << ba::mean(pos_acc) << std::endl;
    std::cout << "Position Difference Std. Dev.: " << std::sqrt(ba::variance(pos_acc)) << std::endl;

    std::cout << "Orientation Difference Mean: " << ba::mean(ori_acc) << std::endl;
    std::cout << "Orientation difference Std. Dev.: " << std::sqrt(ba::variance(ori_acc)) << std::endl;
  }

  double residual_threshold_sqr;
  std::size_t n_observations;

//  test::Camera camera;
//  test::Target target;

  KinematicMeasurement::Set observations;

  const DHChain camera_chain;
  const DHChain target_chain;
  Eigen::Isometry3d camera_mount_to_camera;
  Eigen::Isometry3d target_mount_to_target;
  Eigen::Isometry3d camera_base_to_target_base;

  KinematicCalibrationProblemPose3D problem;
};

/**
 * @brief Tests the Dual DH Chain kinematic calibration algorithm with perfect initial conditions
 */
class DHChainCalTest_PerfectInitial : public DHChainMeasurementTest
{
public:
  using DHChainMeasurementTest::DHChainMeasurementTest;

  virtual void analyzeResults(const KinematicCalibrationResult& result) override
  {
    // Check the nominal expectations
    DHChainMeasurementTest::analyzeResults(result);

    EXPECT_LT(result.initial_cost_per_obs, 1.0e-15);

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
  }
};

/**
 * @brief Tests the Dual DH Chain kinematic calibration algorithm with slightly modified DH parameters
 */
class DHChainCalTest_PerturbedDH : public DHChainMeasurementTest
{
public:
  using DHChainMeasurementTest::DHChainMeasurementTest;

  virtual void setActualData() override
  {
    DHChainMeasurementTest::setActualData();

    // Set the residual squared error threshold to 0.5 pixels
    residual_threshold_sqr = 0.5 * 0.5;
  }

  virtual void setObservations() override
  {
    EXPECT_NO_THROW(problem.observations = test::createKinematicMeasurements(
                        test::perturbDHChain(problem.camera_chain, 1.0e-3), problem.target_chain,
                        camera_mount_to_camera, target_mount_to_target, camera_base_to_target_base,
                        n_observations));
  }

//  virtual void applyMasks() override
//  {
//    // Mask a few DH parameters
//    {
//      Eigen::Matrix<bool, Eigen::Dynamic, 4> mask =
//          Eigen::Matrix<bool, Eigen::Dynamic, 4>::Constant(problem.camera_chain.dof(), 4, false);

//      // Mask the last row because they duplicate the camera mount to camera transform
//      mask.bottomRows(1) << true, true, true, true;

//      // Mask the joint 2 "r" parameter
//      // It seems to have a high correlation with target_mount_to_target_y
//      mask(1, 2) = true;

//      // Add the mask to the problem
//      problem.mask.at(0) = createDHMask(mask);
//    }

//    /* Mask the camera base to target base transform (duplicated by target mount to target transform when
//     * the target chain has no joints */
//    problem.mask.at(6) = { 0, 1, 2 };
//    problem.mask.at(7) = { 0, 1, 2 };
//  }

  virtual void analyzeResults(const KinematicCalibrationResult& result) override
  {
    DHChainMeasurementTest::analyzeResults(result);

    // Expect the masked variables not to have changed
    // TODO: turn on when masking PR is merged in
//    EXPECT_TRUE(result.camera_chain_dh_offsets.bottomRows(1).isApproxToConstant(0.0));
//    EXPECT_DOUBLE_EQ(result.camera_chain_dh_offsets(1, 2), 0.0);
//    EXPECT_TRUE(result.camera_base_to_target_base.isApprox(problem.camera_base_to_target_base_guess));
  }
};

/**
 * @brief Tests the Dual DH Chain kinematic calibration algorithm with
 * slightly modified DH parameters and initial transform guesses
 */
class DHChainCalTest_PerturbedDH_PertubedGuess : public DHChainCalTest_PerturbedDH
{
public:
  using DHChainCalTest_PerturbedDH::DHChainCalTest_PerturbedDH;

  virtual void setInitialGuess() override
  {
    problem.camera_mount_to_camera_guess = test::perturbPose(camera_mount_to_camera, 0.05, 0.05);
    problem.target_mount_to_target_guess = test::perturbPose(target_mount_to_target, 0.05, 0.05);
    problem.camera_base_to_target_base_guess = test::perturbPose(camera_base_to_target_base, 0.05, 0.05);
  }
};

TEST_F(DHChainMeasurementTest, TestCostFunction)
{
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
      = DualDHChainCostPose3D::constructParameters(camera_chain_dh_offsets,
                                                 target_chain_dh_offsets,
                                                 t_cm_to_c,
                                                 aa_cm_to_c,
                                                 t_tm_to_t,
                                                 aa_tm_to_t,
                                                 t_ccb_to_tcb,
                                                 aa_ccb_to_tcb);

  // Create observations
  KinematicMeasurement::Set observations = test::createKinematicMeasurements(camera_chain,
                                                                             target_chain,
                                                                             camera_mount_to_camera,
                                                                             target_mount_to_target,
                                                                             camera_base_to_target_base,
                                                                             n_observations);

  // Test the cost function
  for (const auto &obs : observations)
  {

      DualDHChainCostPose3D cost(obs,
                                 camera_chain,
                                 target_chain,
                                 obs.camera_chain_joints,
                                 obs.target_chain_joints);

      double residual[9];
      EXPECT_TRUE(cost(parameters.data(), residual));

      EXPECT_NEAR(residual[0], 0, 1.0e-12);  // x
      EXPECT_NEAR(residual[1], 0, 1.0e-12);  // y
      EXPECT_NEAR(residual[2], 0, 1.0e-12);  // z

      EXPECT_NEAR(residual[3], 0, 1.0e-12);  // p1_x
      EXPECT_NEAR(residual[4], 0, 1.0e-12);  // p1_y
      EXPECT_NEAR(residual[5], 0, 1.0e-12);  // p1_z

      EXPECT_NEAR(residual[6], 0, 1.0e-12);  // p2_x
      EXPECT_NEAR(residual[7], 0, 1.0e-12);  // p2_y
      EXPECT_NEAR(residual[8], 0, 1.0e-12);  // p2_z
  }
}

TEST_F(DHChainCalTest_PerfectInitial, PerfectInitialConditions)
{
  KinematicCalibrationResult result = optimize(problem);
  std::cout << result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;
  analyzeResults(result);
}

TEST_F(DHChainCalTest_PerturbedDH, PerfectGuessPerturbedDH)
{
  KinematicCalibrationResult result = optimize(problem);
  std::cout << result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;
  analyzeResults(result);
}

TEST_F(DHChainCalTest_PerturbedDH_PertubedGuess, PerturbedDHPerturbedGuess)
{
  KinematicCalibrationResult result = optimize(problem);
  std::cout << result.covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;
  analyzeResults(result);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
