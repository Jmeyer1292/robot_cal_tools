#pragma once

#include <vector>
#include <Eigen/Dense>
#include <rct_optimizations/types.h>

namespace rct_optimizations
{

/**
 * @brief Problem setup info for the circle fit optimization.
 */
struct CircleFitProblem
{
  /**
   * @brief Observations around the edge of the circle. 2D points in the form (x, y).
   */
  std::vector<Eigen::Vector2d> observations;

  /**
   * @brief Estimate for the initial x-coordinate of the center of the circle.
   */
  double x_center_initial;

  /**
   * @brief Estimate for the initial y-coordinate of the center of the circle.
   */
  double y_center_initial;

  /**
   * @brief Estimate for the initial radius of the circle.
   */
  double radius_initial;
};

/**
 * @brief Results for the circle fit optimization
 */
struct CircleFitResult
{
  /**
   * @brief True if converged, False if any other result.
   */
  bool converged;

  /**
   * @brief Initial average error per observation.
   */
  double initial_cost_per_obs;

  /**
   * @brief Final average error per observation.
   */
  double final_cost_per_obs;

  /**
   * @brief The covariance matrix for the problem. A square 3x3 matrix giving covariance between each pair of elements. Order of elements is [x, y, radius].
   */
  Eigen::MatrixXd covariance;

  /**
   * @brief Calculated circle center x-coord.
   */
  double x_center;

  /**
   * @brief Calculated circle center y-coord.
   */
  double y_center;

  /**
   * @brief Calculated circle radius.
   */
  double radius;
};

/**
 * @brief Function that solves the circle fit problem.
 * @param Input observations and guesses.
 * @return Output results.
 */
CircleFitResult optimize(const CircleFitProblem& params);

}  // namespace rct_optimizations
