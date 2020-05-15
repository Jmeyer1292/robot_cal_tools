#include <ceres/ceres.h>
#include <rct_optimizations/circle_fit.h>
#include <rct_optimizations/covariance_analysis.h>

namespace
{

/**
 * @brief Cost function for the error between a point and a circle.
 */
class CircleDistCost
{
public:
  CircleDistCost(const double& x, const double& y)
    : x_(x), y_(y)
  {}


  /**
   * @brief One formulation of a residual function for error relative to the center of a circle.
   * Presented to the solver as three size-1 double parameters.
   * @param sample A double pointer which contains {x, y, r_sqrt}.
   */
  template<typename T>
  bool operator()(const T* const sample,
                  T* residual) const
  {
    T r = sample[2] * sample[2];

    T x_pos = x_ - sample[0];
    T y_pos = y_ - sample[1];

    residual[0] = r * r - x_pos * x_pos - y_pos * y_pos;
    return true;
  }

  /**
   * @brief An alternative formulation of a residual function for error relative to the center of a circle.
   * Here, the inputs (x, y, r_sqrt) are provided as separate parameters. This is mathematically identical to the previous version,
   * but the number and size of the parameters are presented differently to the solver (one size-3 double parameter).
   * @param x_sample The x-coordinate of the center of the circle.
   * @param y_sample The y-coordinate of the center of the circle.
   * @param r_sqrt_sample The square root of the radius of the circle.
   */
  template<typename T>
  bool operator()(const T* const x_sample,
                  const T* const y_sample,
                  const T* const r_sqrt_sample,
                  T* residual) const
  {
    T r = r_sqrt_sample * r_sqrt_sample;  // Using sqrt(radius) prevents the radius from ending up negative and improves numerical stability.

    T x_pos = x_ - x_sample;
    T y_pos = y_ - y_sample;

    residual[0] = r * r - x_pos * x_pos - y_pos * y_pos;
    return true;
  }

private:
  double x_, y_;
};

}

rct_optimizations::CircleFitResult
rct_optimizations::optimize(const rct_optimizations::CircleFitProblem& params)
{
  double x = params.x_center_initial;
  double y = params.y_center_initial;
  double r = params.radius_initial;

  double r_sqrt = sqrt(r);

  std::vector<double> circle_params(3, 0.0);
  circle_params[0] = x;
  circle_params[1] = y;
  circle_params[2] = r_sqrt;

  rct_optimizations::CircleFitResult result;

  ceres::Problem problem;

  ceres::LossFunction* loss_fn = NULL;

  for (auto obs : params.observations)
  {
    auto* cost_fn = new CircleDistCost(obs.x(), obs.y());

    auto* cost_block = new ceres::AutoDiffCostFunction<CircleDistCost, 1, 3>(cost_fn);
    problem.AddResidualBlock(cost_block, loss_fn, circle_params.data());
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);


  std::cout << summary.BriefReport() << std::endl;

  result.converged = summary.termination_type == ceres::TerminationType::CONVERGENCE;
  result.x_center = circle_params[0];
  result.y_center = circle_params[1];
  result.radius = pow(circle_params[2], 2);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  try
  {
    result.covariance = rct_optimizations::computeDVCovariance(problem, circle_params.data(), 3);
  }
  catch(std::runtime_error &e)
  {
    std::cout << "Failed to compute covariance: " << e.what() << std::endl;
  }

  return result;
}
