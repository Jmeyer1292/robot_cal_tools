#include <rct_optimizations/maximum_likelihood.h>

#include <ceres/problem.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

using namespace rct_optimizations;

TEST(MaximumLikelihoodTests, NaiveTest)
{
  double mean_val = 0.0;
  double stdev_val = 0.1;

  // Create matrices of mean and standard deviation values for each parameter
  Eigen::ArrayXXd mean(Eigen::ArrayXXd::Constant(6, 4, mean_val));
  Eigen::ArrayXXd stdev(Eigen::ArrayXXd::Constant(6, 4, stdev_val));

  // Create a matrix of random values scaled up from a range of [0, 1]
  const double scale = 10.0 * stdev_val;
  Eigen::ArrayXXd rand(Eigen::ArrayXXd::Random(6, 4) * scale);

  auto *ml = new MaximumLikelihood(mean, stdev);

  // Check the cost function
  {
    Eigen::ArrayXXd residual(Eigen::ArrayXXd::Zero(6, 4));
    std::vector<double *> params = {rand.data()};

    bool success = ml->operator()(params.data(), residual.data());
    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isApprox((rand - mean) / stdev));
  }

  // Run an optimization to see if we can drive the random numbers to the mean
  ceres::Problem problem;
  auto *cost_block = new ceres::DynamicAutoDiffCostFunction<MaximumLikelihood>(ml);
  cost_block->AddParameterBlock(rand.size());
  cost_block->SetNumResiduals(rand.size());
  problem.AddResidualBlock(cost_block, nullptr, rand.data());

  // Solve the optimization
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  EXPECT_TRUE(summary.termination_type == ceres::CONVERGENCE);

  // Check element-wise that absolute difference between the mean matrix and optimized random matrix is almost zero
  Eigen::ArrayXXd diff = (rand - mean).abs();
  for (auto row = 0; row < diff.rows(); ++row)
  {
    for (auto col = 0; col < diff.cols(); ++col)
    {
      EXPECT_LT(diff(row, col), 1.0e-12);
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
