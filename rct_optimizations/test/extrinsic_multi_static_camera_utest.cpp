#include <gtest/gtest.h>
#include <rct_optimizations/extrinsic_multi_static_camera.h>
#include <rct_optimizations/ceres_math_utilities.h>

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

void printResults(const rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetResult &opt_result)
{
  // Report results
  std::cout << "Did converge?: " << opt_result.converged << "\n";
  std::cout << "Initial cost?: " << opt_result.initial_cost_per_obs << "\n";
  std::cout << "Final cost?: " << opt_result.final_cost_per_obs << "\n";

  Eigen::Affine3d t = opt_result.wrist_to_target;

  std::cout << "Wrist to Target:\n";
  std::cout << t.matrix() << "\n";

  for (std::size_t c = 0; c < opt_result.base_to_camera.size(); ++c)
  {
    // Load the data set path from ROS param
    std::string param_base = "camera_" + std::to_string(c);
    Eigen::Affine3d t = opt_result.base_to_camera[c];

    std::cout << "Base to Camera (" + param_base + "):\n";
    std::cout << t.matrix() << "\n";

    std::cout << "--- URDF Format Base to Camera (" + param_base + ") ---\n";
    Eigen::Vector3d rpy = t.rotation().eulerAngles(2, 1, 0);
    std::cout << "xyz=\"" << t.translation()(0) << " " << t.translation()(1) << " " << t.translation()(2) << "\"\n";
    std::cout << "rpy=\"" << rpy(2) << "(" << rpy(2) * 180/M_PI << " deg) " << rpy(1) << "(" << rpy(1) * 180/M_PI << " deg) " << rpy(0) << "(" << rpy(0) * 180/M_PI << " deg)\"\n";
  }
}

void addObservations(const Eigen::Vector3d& axis,
                     const std::vector<Eigen::Vector3d> &target_points,
                     const std::vector<Eigen::Affine3d> &base_to_camera,
                     const Eigen::Affine3d &wrist_to_target,
                     rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetProblem &problem_def)
{
  for (int i = -10; i <= 10; ++i)
  {
    Eigen::Affine3d base_to_wrist;

    Eigen::Affine3d link_1 = Eigen::Affine3d::Identity();
    link_1.translation() = Eigen::Vector3d(0, 0, 0.25);

    Eigen::Affine3d link_2(Eigen::AngleAxisd(i * M_PI/180.0, axis));
    link_2.translation() = Eigen::Vector3d(0, 0, 0.25);

    base_to_wrist = link_1 * link_2;

    for (std::size_t c = 0; c < problem_def.intr.size(); ++c)
    {
      problem_def.wrist_poses[c].push_back(base_to_wrist);

      rct_optimizations::CorrespondenceSet obs_set;
      for (std::size_t j = 0; j < target_points.size(); ++j)
      {
        rct_optimizations::Correspondence2D3D pair;

        Eigen::Affine3d target_to_camera = wrist_to_target.inverse() * base_to_wrist.inverse() * base_to_camera[c];
        Eigen::Vector3d in_camera = target_to_camera.inverse() * target_points[j];

        double uv[2];
        rct_optimizations::projectPoint(problem_def.intr[c], in_camera.data(), uv);
        pair.in_image = Eigen::Vector2d(uv[0], uv[1]);
        pair.in_target = target_points[j];

        obs_set.push_back(pair);
      }
      problem_def.image_observations[c].push_back(obs_set);
    }
  }
}

TEST(ExtrinsicMultiStaticCamera, single_camera)
{
  rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetProblem problem_def;
  std::vector<Eigen::Vector3d> target_points;
  makePoints(5, 5, 0.015, target_points);

  std::vector<Eigen::Affine3d> base_to_camera;
  base_to_camera.push_back(Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())));
  base_to_camera[0].translation() = Eigen::Vector3d(0, 0, 2.0);

  Eigen::Affine3d wrist_to_target = Eigen::Affine3d::Identity();
  wrist_to_target.translation() = Eigen::Vector3d(0, 0, 0.25);

  problem_def.intr.resize(1);
  problem_def.base_to_camera_guess.resize(1);
  problem_def.wrist_poses.resize(1);
  problem_def.image_observations.resize(1);

  problem_def.base_to_camera_guess[0] = Eigen::Affine3d(Eigen::AngleAxisd(M_PI + 0.01, Eigen::Vector3d::UnitX()));
  problem_def.base_to_camera_guess[0].translation() = Eigen::Vector3d(0.01, 0.001, 1.99);

  problem_def.wrist_to_target_guess = Eigen::Affine3d::Identity();
  problem_def.wrist_to_target_guess.translation() = Eigen::Vector3d(0.001, 0.001, 0.26);

  problem_def.intr[0].fx() = 1411.0;
  problem_def.intr[0].fy() = 1408.0;
  problem_def.intr[0].cx() = 807.2;
  problem_def.intr[0].cy() = 615.0;

  addObservations(Eigen::Vector3d::UnitY(), target_points, base_to_camera, wrist_to_target, problem_def);
  addObservations(Eigen::Vector3d::UnitX(), target_points, base_to_camera, wrist_to_target, problem_def);

  // Run optimization
  rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetResult
      opt_result = rct_optimizations::optimize(problem_def);

  EXPECT_TRUE(opt_result.converged);
  EXPECT_TRUE(opt_result.final_cost_per_obs < 1e-15);
  EXPECT_TRUE(opt_result.wrist_to_target.isApprox(wrist_to_target, 1e-8));
  for (std::size_t c = 0; c < opt_result.base_to_camera.size(); ++c)
  {
    EXPECT_TRUE(opt_result.base_to_camera[c].isApprox(base_to_camera[c], 1e-8));
  }

  printResults(opt_result);


}

TEST(ExtrinsicMultiStaticCamera, two_cameras)
{
  rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetProblem problem_def;
  std::vector<Eigen::Vector3d> target_points;
  makePoints(5, 5, 0.015, target_points);

  std::vector<Eigen::Affine3d> base_to_camera;
  Eigen::Affine3d wrist_to_target = Eigen::Affine3d::Identity();

  base_to_camera.resize(2);
  base_to_camera[0] = Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  base_to_camera[0].translation() = Eigen::Vector3d(-0.1, 0, 2.0);
  base_to_camera[1] = Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  base_to_camera[1].translation() = Eigen::Vector3d(0.1, 0, 2.0);

  wrist_to_target.translation() = Eigen::Vector3d(0, 0, 0.25);

  problem_def.intr.resize(2);
  problem_def.base_to_camera_guess.resize(2);
  problem_def.wrist_poses.resize(2);
  problem_def.image_observations.resize(2);

  problem_def.base_to_camera_guess[0] = Eigen::Affine3d(Eigen::AngleAxisd(M_PI + 0.01, Eigen::Vector3d::UnitX()));
  problem_def.base_to_camera_guess[0].translation() = Eigen::Vector3d(-0.101, 0.001, 1.99);
  problem_def.base_to_camera_guess[1] = Eigen::Affine3d(Eigen::AngleAxisd(M_PI + 0.02, Eigen::Vector3d::UnitX()));
  problem_def.base_to_camera_guess[1].translation() = Eigen::Vector3d(0.101, 0.001, 1.99);

  problem_def.wrist_to_target_guess = Eigen::Affine3d::Identity();
  problem_def.wrist_to_target_guess.translation() = Eigen::Vector3d(0.001, 0.001, 0.26);

  problem_def.intr[0].fx() = 1411.0;
  problem_def.intr[0].fy() = 1408.0;
  problem_def.intr[0].cx() = 807.2;
  problem_def.intr[0].cy() = 615.0;
  problem_def.intr[1] = problem_def.intr[0];

  addObservations(Eigen::Vector3d::UnitY(), target_points, base_to_camera, wrist_to_target, problem_def);
  addObservations(Eigen::Vector3d::UnitX(), target_points, base_to_camera, wrist_to_target, problem_def);

  // Run optimization
  rct_optimizations::ExtrinsicMultiStaticCameraMovingTargetResult
      opt_result = rct_optimizations::optimize(problem_def);

  EXPECT_TRUE(opt_result.converged);
  EXPECT_TRUE(opt_result.final_cost_per_obs < 1e-15);
  EXPECT_TRUE(opt_result.wrist_to_target.isApprox(wrist_to_target, 1e-8));
  for (std::size_t c = 0; c < opt_result.base_to_camera.size(); ++c)
  {
    EXPECT_TRUE(opt_result.base_to_camera[c].isApprox(base_to_camera[c], 1e-8));
  }

  printResults(opt_result);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
