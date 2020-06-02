#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <functional>

// Optimization for tests
#include <rct_optimizations/circle_fit.h>

template <typename T>
T perturb_normal(T val, T std_dev)
{
  return std::bind(std::normal_distribution<T>{val, std_dev}, std::mt19937(std::random_device{}()))();
}

template <typename T>
T perturb_random(T mean, T offset)
{
  return std::bind(std::uniform_real_distribution<T>{mean - offset, mean + offset}, std::mt19937(std::random_device{}()))();
}

class CircleFitUnit : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setActualData();
    setInitialGuess();
    setObservations();

    problem.x_center_initial = center_x_initial;
    problem.y_center_initial = center_y_initial;
    problem.radius_initial = radius_initial;
    problem.observations = observations;
  }

  virtual void setActualData() = 0;

  virtual void setInitialGuess() = 0;

  virtual void setObservations() = 0;

  static void printResults(const rct_optimizations::CircleFitResult& r)
  {
    std::cout << "X: " << r.x_center << std::endl;
    std::cout << "Y: " << r.y_center << std::endl;
    std::cout << "R: " << r.radius << std::endl;
    std::cout << "Covariance Matrix:\n" << r.covariance.matrix() << std::endl;
  }

  std::double_t center_x_actual, center_y_actual, radius_actual;

  rct_optimizations::CircleFitProblem problem;
  std::double_t center_x_initial, center_y_initial, radius_initial;
  std::vector<Eigen::Vector2d> observations;
};


class CircleFitUnit_PerfectObservations_RandomGuess : public CircleFitUnit
{
protected:
  void setActualData()
  {
    center_x_actual = 0.0;
    center_y_actual = 0.0;
    radius_actual = 1.0;
  }

  void setInitialGuess()
  {
    center_x_initial = perturb_random<std::double_t>(0.0, 1.0);
    center_y_initial = perturb_random<std::double_t>(0.0, 1.0);
    radius_initial = perturb_random<std::double_t>(1.0, 0.9);
  }

  void setObservations()
  {
    std::size_t n_obs_target = 50;
    std::double_t obs_step = 2 * M_PI / n_obs_target;

    for (std::double_t angle = 0; angle < 2 * M_PI; angle += obs_step)
    {
      std::double_t point_x = radius_actual * cos(angle) + center_x_actual;
      std::double_t point_y = radius_actual * sin(angle) + center_y_actual;
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_SlightlyPerturbed : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::size_t n_obs_target = 50;
    std::double_t obs_step = 2 * M_PI / n_obs_target;
    std::double_t std_dev = radius_actual * 0.01;

    for (std::double_t angle = 0; angle < 2 * M_PI; angle += obs_step)
    {
      std::double_t point_x = perturb_normal<std::double_t>(radius_actual * cos(angle) + center_x_actual, std_dev);
      std::double_t point_y = perturb_normal<std::double_t>(radius_actual * sin(angle) + center_y_actual, std_dev);
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_VeryPerturbed : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::size_t n_obs_target = 50;
    std::double_t obs_step = 2 * M_PI / n_obs_target;
    std::double_t std_dev = radius_actual * 0.1;

    for (std::double_t angle = 0; angle < 2 * M_PI; angle += obs_step)
    {
      std::double_t point_x = perturb_normal<std::double_t>(radius_actual * cos(angle) + center_x_actual, std_dev);
      std::double_t point_y = perturb_normal<std::double_t>(radius_actual * sin(angle) + center_y_actual, std_dev);
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_AnglePerturbed : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::size_t n_obs_target = 50;
    std::double_t obs_step = 2 * M_PI / n_obs_target;
    std::double_t std_dev = 0.0;

    for (std::double_t angle = 0; angle < 2 * M_PI; angle += perturb_normal<std::double_t>(obs_step, 0.3))
    {
      std::double_t point_x = perturb_normal<std::double_t>(radius_actual * cos(angle) + center_x_actual, std_dev);
      std::double_t point_y = perturb_normal<std::double_t>(radius_actual * sin(angle) + center_y_actual, std_dev);
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};


class CircleFitUnit_ClusteredObservations : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::vector<std::double_t> angles {-0.02, -0.01, 0.0, 0.01, 0.02, M_PI - 0.02, M_PI - 0.01, M_PI, M_PI + 0.01, M_PI + 0.02};

    for (auto angle : angles)
    {
      std::double_t point_x = radius_actual * cos(angle) + center_x_actual;
      std::double_t point_y = radius_actual * sin(angle) + center_y_actual;
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_ClusteredObservationsPerturbed : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::vector<std::double_t> angles {-0.02, -0.01, 0.0, 0.01, 0.02, M_PI - 0.02, M_PI - 0.01, M_PI, M_PI + 0.01, M_PI + 0.02};

    for (auto angle : angles)
    {
      std::double_t point_x = perturb_normal<std::double_t>(radius_actual * cos(angle) + center_x_actual, 0.05);
      std::double_t point_y = perturb_normal<std::double_t>(radius_actual * sin(angle) + center_y_actual, 0.05);
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_ParallelLines : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    for (std::double_t y = -2.0; y <= 2.0; y += 0.2)
    {
      observations.emplace_back(Eigen::Vector2d(-1.0, y));
      observations.emplace_back(Eigen::Vector2d(1.0, y));
    }
  }
};

class CircleFitUnit_TwoObsX : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    observations.emplace_back(Eigen::Vector2d(-1.0, 0.0));
    observations.emplace_back(Eigen::Vector2d(1.0, 0.0));
  }
};

class CircleFitUnit_TwoObsY : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    observations.emplace_back(Eigen::Vector2d(0.0, -1.0));
    observations.emplace_back(Eigen::Vector2d(0.0, 1.0));
  }
};

class CircleFitUnit_ThreeObs : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    std::size_t n_obs_target = 3;
    std::double_t obs_step = 2 * M_PI / n_obs_target;

    for (std::double_t angle = 0; angle < 2 * M_PI; angle += obs_step)
    {
      std::double_t point_x = radius_actual * cos(angle) + center_x_actual;
      std::double_t point_y = radius_actual * sin(angle) + center_y_actual;
      observations.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
};

class CircleFitUnit_OneObs : public CircleFitUnit_PerfectObservations_RandomGuess
{
protected:
  void setObservations() override
  {
    observations.emplace_back(Eigen::Vector2d(0.0, 0.0));
  }
};

TEST_F(CircleFitUnit_PerfectObservations_RandomGuess, FitCircleToPerfectObs)
{
  rct_optimizations::CircleFitResult result;
  EXPECT_NO_THROW(result = rct_optimizations::optimize(problem));
  printResults(result);

  // assert that the optimization converged and expect that the results are close to the real values
  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(center_x_actual, result.x_center, 1e-5);
  EXPECT_NEAR(center_y_actual, result.y_center, 1e-5);
  EXPECT_NEAR(radius_actual, result.radius, 1e-5);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.rows(), 3);
  ASSERT_EQ(result.covariance.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance(0, 1), result.covariance(1, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(0, 2), result.covariance(2, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(1, 2), result.covariance(2, 1), std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance(0, 0), 0.0);
  EXPECT_GE(result.covariance(1, 1), 0.0);
  EXPECT_GE(result.covariance(2, 2), 0.0);

  // expect off-diagonal elements to be near-zero
  EXPECT_NEAR(result.covariance(0, 1), 0.0, 1e-5);
  EXPECT_NEAR(result.covariance(0, 2), 0.0, 1e-5);
  EXPECT_NEAR(result.covariance(1, 2), 0.0, 1e-5);

  // with perfect evenly-spaced observations we expect the standard deviations of X and Y to be near-identical
  EXPECT_NEAR(result.covariance(0, 0), result.covariance(1, 1), 1e-5);
}

TEST_F(CircleFitUnit_ClusteredObservations, FitCircleToClusteredObs)
{
  rct_optimizations::CircleFitResult result;
  EXPECT_NO_THROW(result = rct_optimizations::optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.rows(), 3);
  ASSERT_EQ(result.covariance.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance(0, 1), result.covariance(1, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(0, 2), result.covariance(2, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(1, 2), result.covariance(2, 1), std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance(0, 0), 0.0);
  EXPECT_GE(result.covariance(1, 1), 0.0);
  EXPECT_GE(result.covariance(2, 2), 0.0);

  // std dev of Y-coord should be greater than std dev of X-coord
  EXPECT_GT(result.covariance(1, 1), result.covariance(0, 0));
}

TEST_F(CircleFitUnit_ClusteredObservationsPerturbed, FitCircleToClusteredObsPerturbed)
{
  rct_optimizations::CircleFitResult result;
  EXPECT_NO_THROW(result = rct_optimizations::optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.rows(), 3);
  ASSERT_EQ(result.covariance.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance(0, 1), result.covariance(1, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(0, 2), result.covariance(2, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(1, 2), result.covariance(2, 1), std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance(0, 0), 0.0);
  EXPECT_GE(result.covariance(1, 1), 0.0);
  EXPECT_GE(result.covariance(2, 2), 0.0);

  // std dev of Y-coord should be greater than std dev of X-coord
  EXPECT_GT(result.covariance(1, 1), result.covariance(0, 0));
}

TEST_F(CircleFitUnit_ParallelLines, FitCircleToParallelLines)
{
  rct_optimizations::CircleFitResult result;
  EXPECT_NO_THROW(result = rct_optimizations::optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.rows(), 3);
  ASSERT_EQ(result.covariance.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance(0, 1), result.covariance(1, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(0, 2), result.covariance(2, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(1, 2), result.covariance(2, 1), std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance(0, 0), 0.0);
  EXPECT_GE(result.covariance(1, 1), 0.0);
  EXPECT_GE(result.covariance(2, 2), 0.0);
}

// Do optimization to fit a circle to two observed points.
TEST_F(CircleFitUnit_TwoObsX, FitCircleToTwoPoints)
{
  rct_optimizations::CircleFitResult result;
  EXPECT_NO_THROW(result = rct_optimizations::optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.rows(), 3);
  ASSERT_EQ(result.covariance.cols(), 3);

  // expect matrix to be symmetric
  // BUG: flaky way to evaluate, since difference is sometimes (but rarely) greater than epsilon
  EXPECT_NEAR(result.covariance(0, 1), result.covariance(1, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(0, 2), result.covariance(2, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(1, 2), result.covariance(2, 1), std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance(0, 0), 0.0);
  EXPECT_GE(result.covariance(1, 1), 0.0);
  EXPECT_GE(result.covariance(2, 2), 0.0);

  // expect covariance between Y and R to be close to 1
  EXPECT_NEAR(abs(result.covariance(1, 2)), 1.0, 1e-10);

  // expect covariance between X and R to be somewhat close to 0
  // TODO: fix, not always close to zero due to random initial conditions of problem
//  EXPECT_NEAR(result.covariance(0, 2), 0.0, 1e-2);
}

// Do optimization to fit a circle to two observed points.
TEST_F(CircleFitUnit_TwoObsY, FitCircleToTwoPoints)
{
  rct_optimizations::CircleFitResult result;
  EXPECT_NO_THROW(result = rct_optimizations::optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.rows(), 3);
  ASSERT_EQ(result.covariance.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance(0, 1), result.covariance(1, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(0, 2), result.covariance(2, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(1, 2), result.covariance(2, 1), std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance(0, 0), 0.0);
  EXPECT_GE(result.covariance(1, 1), 0.0);
  EXPECT_GE(result.covariance(2, 2), 0.0);

  // expect covariance between X and R to be close to 1
  EXPECT_NEAR(abs(result.covariance(0, 2)), 1.0, 1e-10);

  // expect covariance between Y and R to be somewhat close to 0
  // TODO: fix, not always close to zero due to random initial conditions of problem
//  EXPECT_NEAR(result.covariance(1, 2), 0.0, 1e-2);
}

// Do optimization to fit a circle to just one observed point.
TEST_F(CircleFitUnit_OneObs, FitCircleToOnePoint)
{
  rct_optimizations::CircleFitResult result;
  EXPECT_NO_THROW(result = rct_optimizations::optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.rows(), 3);
  ASSERT_EQ(result.covariance.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance(0, 1), result.covariance(1, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(0, 2), result.covariance(2, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(1, 2), result.covariance(2, 1), std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance(0, 0), 0.0);
  EXPECT_GE(result.covariance(1, 1), 0.0);
  EXPECT_GE(result.covariance(2, 2), 0.0);

  // expect magnitudes of all off-diagonal elements to be close to 1
  EXPECT_NEAR(abs(result.covariance(0, 1)), 1.0, 1e-5);
  EXPECT_NEAR(abs(result.covariance(0, 2)), 1.0, 1e-5);
  EXPECT_NEAR(abs(result.covariance(1, 2)), 1.0, 1e-5);
}

TEST_F(CircleFitUnit_ThreeObs, FitCircleToThreePoints)
{
  rct_optimizations::CircleFitResult result;
  EXPECT_NO_THROW(result = rct_optimizations::optimize(problem));
  printResults(result);

  // assert 3x3 covariance matrix
  ASSERT_EQ(result.covariance.rows(), 3);
  ASSERT_EQ(result.covariance.cols(), 3);

  // expect matrix to be symmetric
  EXPECT_NEAR(result.covariance(0, 1), result.covariance(1, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(0, 2), result.covariance(2, 0), std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.covariance(1, 2), result.covariance(2, 1), std::numeric_limits<double>::epsilon());

  // expect diagonal elements to be positive
  EXPECT_GE(result.covariance(0, 0), 0.0);
  EXPECT_GE(result.covariance(1, 1), 0.0);
  EXPECT_GE(result.covariance(2, 2), 0.0);

  // expect off-diagonal elements to be close to 0
  EXPECT_NEAR(result.covariance(0, 1), 0.0, 1e-5);
  EXPECT_NEAR(result.covariance(0, 2), 0.0, 1e-5);
  EXPECT_NEAR(result.covariance(1, 2), 0.0, 1e-5);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
