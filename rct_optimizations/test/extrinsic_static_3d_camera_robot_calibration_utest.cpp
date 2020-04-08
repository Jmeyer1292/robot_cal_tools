#include <gtest/gtest.h>
#include <rct_optimizations/extrinsic_static_3d_camera_robot_calibration.h>
#include <rct_optimizations/ceres_math_utilities.h>


const static double SCALE_POSITION = 100.0;

void makePoints(std::size_t rows, std::size_t cols, double spacing, std::vector<Eigen::Vector3d>& points)
{
  points.reserve(rows * cols);

  for (std::size_t i = 1; i < (rows + 1); i++)
  {
    double y = (rows - i) * spacing;
    for (std::size_t j = 0; j < cols; j++)
    {
      double x = j * spacing;
      Eigen::Vector3d point(x, y, 0.0);
      points.push_back(point);
    }
  }
}

void printResults(const rct_optimizations::ExtrinsicStatic3DCameraRobotCalibrationResult &opt_result)
{
  // Report results
  std::cout << "Did converge?: " << opt_result.converged << "\n";
  std::cout << "Initial cost?: " << opt_result.initial_cost_per_obs << "\n";
  std::cout << "Final cost?: " << opt_result.final_cost_per_obs << "\n";

  Eigen::Isometry3d t = opt_result.wrist_to_target;
  t.translation() *= (1.0 / SCALE_POSITION);
  Eigen::Isometry3d c = opt_result.base_to_camera;
  c.translation() *= (1.0 / SCALE_POSITION);

  std::cout << "Wrist to Target:\n";
  std::cout << t.matrix() << "\n\n";

  std::cout << "Base to Camera:\n";
  std::cout << c.matrix() << "\n\n";

  std::cout << "**************************************************************\n\n";
  for (std::size_t j = 0; j < opt_result.robot_joint_delta.size(); ++j)
  {
    // Load the data set path from ROS param
    std::string param_base = "joint_" + std::to_string(j);
    Eigen::Isometry3d d = opt_result.robot_joint_delta[j];
    d.translation() *= (1.0 / SCALE_POSITION);

    std::cout << "Joint Delta (" + param_base + "):\n";
    std::cout << d.matrix() << "\n\n";

    std::cout << "--- URDF Format Joint Delta (" + param_base + ") ---\n";
    Eigen::Vector3d rpy = d.rotation().eulerAngles(2, 1, 0);
    std::cout << "xyz=\"" << d.translation()(0) / SCALE_POSITION  << " " << d.translation()(1) / SCALE_POSITION << " " << d.translation()(2) / SCALE_POSITION << "\"\n";
    std::cout << "rpy=\"" << rpy(2) << "(" << rpy(2) * 180/M_PI << " deg) " << rpy(1) << "(" << rpy(1) * 180/M_PI << " deg) " << rpy(0) << "(" << rpy(0) * 180/M_PI << " deg)\"\n";
    std::cout << "**************************************************************\n\n";
  }

  std::cout << opt_result.summary.FullReport() << std::endl;
}

std::vector<Eigen::Isometry3d> getRobotTransforms(const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  std::vector<Eigen::Isometry3d> robot_transforms;
  robot_transforms.reserve(6);

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  t1 = t1 * Eigen::Translation3d(0.0, 0.0, SCALE_POSITION * 0.615) * Eigen::AngleAxisd(joint_values(0), Eigen::Vector3d::UnitZ());
  robot_transforms.push_back(t1);

  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2 = t2 * Eigen::Translation3d(SCALE_POSITION * 0.1, 0.0, 0.0) * Eigen::AngleAxisd(-1.0 * joint_values(1), Eigen::Vector3d::UnitY());
  robot_transforms.push_back(t2);

  Eigen::Isometry3d t3 = Eigen::Isometry3d::Identity();
  t3 = t3 * Eigen::Translation3d(0.0, 0.0, SCALE_POSITION * 0.705) * Eigen::AngleAxisd(-1.0 *joint_values(2), Eigen::Vector3d::UnitY());
  robot_transforms.push_back(t3);

  Eigen::Isometry3d t4 = Eigen::Isometry3d::Identity();
  t4 = t4 * Eigen::Translation3d(0.0, 0.0, SCALE_POSITION * 0.1) * Eigen::AngleAxisd(joint_values(3), Eigen::Vector3d::UnitX());
  robot_transforms.push_back(t4);

  Eigen::Isometry3d t5 = Eigen::Isometry3d::Identity();
  t5 = t5 * Eigen::Translation3d(SCALE_POSITION * 0.755, 0.0, 0.0) * Eigen::AngleAxisd(joint_values(4), Eigen::Vector3d::UnitY());
  robot_transforms.push_back(t5);

  Eigen::Isometry3d t6 = Eigen::Isometry3d::Identity();
  t6 = t6 * Eigen::Translation3d(SCALE_POSITION * 0.085, 0.0, 0.0) * Eigen::AngleAxisd(joint_values(5), Eigen::Vector3d::UnitX());
  robot_transforms.push_back(t6);

  return robot_transforms;
}

void addObservations(const std::vector<Eigen::Vector3d> &target_points,
                     const Eigen::Isometry3d &base_to_camera,
                     const Eigen::Isometry3d &wrist_to_target,
                     rct_optimizations::ExtrinsicStatic3DCameraRobotCalibrationProblem &problem_def)
{
  for (long i = 0; i < problem_def.joint_poses.rows(); ++i)
  {
    std::vector<Eigen::Isometry3d> rb_tf = problem_def.robot_calculator(problem_def.joint_poses.row(i));
    Eigen::Isometry3d base_to_wrist = Eigen::Isometry3d::Identity();
    for (auto t : rb_tf)
      base_to_wrist = base_to_wrist * t;

    Eigen::Isometry3d camera_to_target = base_to_camera.inverse() * (base_to_wrist * wrist_to_target);

    rct_optimizations::Correspondence3DSet obs_set;
    for (std::size_t j = 0; j < target_points.size(); ++j)
    {
      rct_optimizations::Correspondence3D3D pair;
      Eigen::Vector3d in_camera = camera_to_target * target_points[j];

      pair.in_image = in_camera;
      pair.in_target = target_points[j];

      obs_set.push_back(pair);
    }
    problem_def.image_observations.push_back(obs_set);
  }
}

TEST(ExtrinsicStatic3DCameraRobotCalibration, observer_cost)
{
  rct_optimizations::ExtrinsicStatic3DCameraRobotCalibrationProblem problem_def;

  // Assign robot calculator
  problem_def.robot_calculator = getRobotTransforms;

  // Create Target Points
  std::vector<Eigen::Vector3d> target_points;
  makePoints(5, 5, SCALE_POSITION * 0.03, target_points);

  // Create Joint Poses
  int num_joint_poses = 100;
  problem_def.joint_poses.resize(num_joint_poses, 6);
  problem_def.joint_poses.row(0) = Eigen::VectorXd::Zero(6);
  for (int i = 1; i < num_joint_poses; ++i)
  {
    for (int j = 0; j < problem_def.joint_poses.cols(); ++j)
      problem_def.joint_poses(i, j) = (-30 + ( std::rand() % ( 30 - (-30) + 1 ) )) * (M_PI / 180.0);
  }

  Eigen::Isometry3d base_to_camera = Eigen::Translation3d(SCALE_POSITION * 2.0, 0.0, 0.0) * Eigen::Isometry3d(Eigen::AngleAxisd(-1.0 * M_PI_2, Eigen::Vector3d::UnitY()));
  Eigen::Isometry3d wrist_to_target = Eigen::Translation3d(0.0, 0.0, SCALE_POSITION * 0.25) * Eigen::Isometry3d(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY()));

  problem_def.base_to_camera_guess = base_to_camera;
  problem_def.wrist_to_target_guess = wrist_to_target;

  addObservations(target_points, base_to_camera, wrist_to_target, problem_def);

  rct_optimizations::Pose6d wt = rct_optimizations::poseEigenToCal(wrist_to_target);
  std::vector<double> jt(6*6, 0.0);
  for (std::size_t i = 0; i < problem_def.joint_poses.rows(); ++i) // For each joint pose / image set
  {
    std::vector<Eigen::Isometry3d> robot_chain_transforms = problem_def.robot_calculator(problem_def.joint_poses.row(i));

    for (std::size_t j = 0; j < problem_def.image_observations[i].size(); ++j) // For each 3D point seen in the 2D image
    {
      // Define
      const auto& img_obs = problem_def.image_observations[i][j].in_image;
      const auto& point_in_target = problem_def.image_observations[i][j].in_target;

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      rct_optimizations::detail::ObservationCost cost_fn(img_obs,
                                                         problem_def.base_to_camera_guess,
                                                         robot_chain_transforms,
                                                         point_in_target);


      double res[3];
      cost_fn(wt.values.data(), jt.data(), res);

      EXPECT_NEAR(res[0], 0, 1e-5);
      EXPECT_NEAR(res[1], 0, 1e-5);
      EXPECT_NEAR(res[2], 0, 1e-5);

    }
  } // for each wrist pose
}

TEST(ExtrinsicStatic3DCameraRobotCalibration, single_camera_exact_guess)
{
  rct_optimizations::ExtrinsicStatic3DCameraRobotCalibrationProblem problem_def;

  // Assign robot calculator
  problem_def.robot_calculator = getRobotTransforms;

  // Create Target Points
  std::vector<Eigen::Vector3d> target_points;
  makePoints(5, 5, SCALE_POSITION * 0.03, target_points);

  // Create Joint Poses
  int num_joint_poses = 50;
  problem_def.joint_poses.resize(num_joint_poses, 6);
  problem_def.joint_poses.row(0) = Eigen::VectorXd::Zero(6);
  for (int i = 1; i < num_joint_poses; ++i)
  {
    for (int j = 0; j < problem_def.joint_poses.cols(); ++j)
      problem_def.joint_poses(i, j) = (-45 + ( std::rand() % ( 45 - (-45) + 1 ) )) * (M_PI / 180.0);
  }

  Eigen::Isometry3d base_to_camera = Eigen::Translation3d(SCALE_POSITION * 2.0, 0.0, 0.0) * Eigen::Isometry3d(Eigen::AngleAxisd(-1.0 * M_PI_2, Eigen::Vector3d::UnitY()));
  Eigen::Isometry3d wrist_to_target = Eigen::Translation3d(SCALE_POSITION * 0.1, 0.0, SCALE_POSITION * 0.25) * Eigen::Isometry3d(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY()));

  problem_def.base_to_camera_guess = base_to_camera;
  problem_def.wrist_to_target_guess = wrist_to_target;

  addObservations(target_points, base_to_camera, wrist_to_target, problem_def);

  // Run optimization
  rct_optimizations::ExtrinsicStatic3DCameraRobotCalibrationResult opt_result = rct_optimizations::optimize<6>(problem_def);

  printResults(opt_result);

}

TEST(ExtrinsicStatic3DCameraRobotCalibration, single_camera)
{
  rct_optimizations::ExtrinsicStatic3DCameraRobotCalibrationProblem problem_def;

  // Assign robot calculator
  problem_def.robot_calculator = getRobotTransforms;

  // Create Target Points
  std::vector<Eigen::Vector3d> target_points;
  makePoints(5, 5, SCALE_POSITION * 0.03, target_points);

  // Create Joint Poses
  int num_joint_poses = 500;
  problem_def.joint_poses.resize(num_joint_poses, 6);
  problem_def.joint_poses.row(0) = Eigen::VectorXd::Zero(6);
  for (int i = 1; i < num_joint_poses; ++i)
  {
    for (int j = 0; j < problem_def.joint_poses.cols(); ++j)
      problem_def.joint_poses(i, j) = (-45 + ( std::rand() % ( 45 - (-45) + 1 ) )) * (M_PI / 180.0);
  }

  Eigen::Isometry3d base_to_camera = Eigen::Translation3d(SCALE_POSITION * 2.0, 0.0, 0.0) * Eigen::Isometry3d(Eigen::AngleAxisd((-1.0 * M_PI_2) - (M_PI_4 / 2.0), Eigen::Vector3d::UnitY()));
  Eigen::Isometry3d wrist_to_target = Eigen::Translation3d(SCALE_POSITION * 0.1, 0.0, SCALE_POSITION * 0.25) * Eigen::Isometry3d(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY()));

  problem_def.base_to_camera_guess = Eigen::Translation3d(SCALE_POSITION * 1.99, SCALE_POSITION * 0.001, SCALE_POSITION * 0.001) * Eigen::Isometry3d(Eigen::AngleAxisd((-1.0 * M_PI_2) - (M_PI_4 / 2.0) - 0.1, Eigen::Vector3d::UnitY()));
  problem_def.wrist_to_target_guess = Eigen::Translation3d(SCALE_POSITION * 0.101, SCALE_POSITION * 0.001, SCALE_POSITION * 0.251) * Eigen::Isometry3d(Eigen::AngleAxisd(M_PI_4 + 0.01, Eigen::Vector3d::UnitY()));

  addObservations(target_points, base_to_camera, wrist_to_target, problem_def);

  // Run optimization
  problem_def.do_compute_pose_covariance_ = true;
  rct_optimizations::ExtrinsicStatic3DCameraRobotCalibrationResult opt_result = rct_optimizations::optimize<6>(problem_def);
  printResults(opt_result);

  // Run optimization
  rct_optimizations::ExtrinsicStatic3DCameraRobotCalibrationResult opt_result2 = rct_optimizations::optimize2<6>(problem_def);
  printResults(opt_result2);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
