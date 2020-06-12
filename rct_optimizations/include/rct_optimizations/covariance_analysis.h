#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <iostream>
#include <rct_optimizations/types.h>

namespace rct_optimizations
{

/**
 * @brief DefaultCovarianceOptions An instance of ceres::Covariance::Options with values better suited to the optimizations in this library.
 */
struct DefaultCovarianceOptions : ceres::Covariance::Options
{
  DefaultCovarianceOptions()
  {
    this->algorithm_type = ceres::DENSE_SVD;  // DENSE_SVD can handle both full-rank and rank-deficient Jacobians
  }
};

/**
 * @brief Given a double array of Ceres covariance results, calculate standard deviations and correlation coefficients and format as an Eigen MatrixXd.
 * Each diagonal element is the standard deviation of a parameter, and its units match the units of that parameter.
 * Each off-diagonal element is the correlation coefficient of a pair of parameters, which is unitless and in the range [-1, 1].
 * @param cov Points to a double array containing covariance results from Ceres.
 * @param num_vars Number of elements in @ref cov.
 * @return A square matrix with size (num_vars x num_vars) containing standard deviations (diagonal elements) and correlation coefficients (off-diagonal elements).
 * Given that @ref cov contains the covariance matrix for parameter block A with parameters [a_1 ... a_n] where n = num_vars,
 * the output matrix will be arranged like this:
 *    |a_1|a_2|...|a_n
 * ---|---------------
 * a_1|
 * a_2|
 * ...|
 * a_n|
 */
Eigen::MatrixXd covToEigenCorr(const double* cov, const std::size_t num_vars);

/**
 * @brief Arrange the Ceres covariance results as an Eigen matrix.
 * @param cov Points to a double array containing covariance results from Ceres.
 * @param num_vars Number of elements in @ref cov.
 * @return A square matrix with size (num_vars x num_vars) containing variance (diagonal elements) and covariance (off-diagonal elements).
 * Given that @ref cov contains the covariance matrix for parameter block A with parameters [a_1 ... a_n] where n = num_vars,
 * the output matrix will be arranged like this:
 *    |a_1|a_2|...|a_n
 * ---|---------------
 * a_1|
 * a_2|
 * ...|
 * a_n|
 */
Eigen::MatrixXd covToEigenCov(const double* cov, const std::size_t num_vars);


/**
 * @brief Calculate the off-diagonal correlation matrix given Ceres covariances.
 * @param cov_d1d1 Pointer to a double array containing covariance for the first parameter block.
 * @param num_vars1 Number of elements in cov_d1d1
 * @param cov_d2d2 Pointer to a double array containing covariance for the second parameter block.
 * @param num_vars2 Number of elements in cov_d1d2
 * @param cov_d1d2 Pointer to a double array containing correlation coefficients for the first parameter block relative to the second parameter block.
 * Must contain (num_vars1 x num_vars2 elements).
 * @return A matrix with size (num_vars1 x num_vars2) containing the off-diagonal covariance.
 * Each row corresponds to a parameter in the first block, and each column corresponds to a parameter in the second block.
 * Given that @ref cov_d1d1 contains the covariance matrix for parameter block A with parameters [a_1, a_2 ... a_n1] where n1 = num_vars1,
 * and that @ref cov_d2d2 contains the covariance matrix for parameter block B with parameters [b_1, b_2 ... b_n2] where n2 = num_vars2,
 * the output matrix will be arranged like this:
 *     |b_1|b_2|...|b_n2
 * ----|---------------
 * a_1 |
 * a_2 |
 * ... |
 * a_n1|
 */
Eigen::MatrixXd covToEigenOffDiagCorr(const double* cov_d1d1, const std::size_t num_vars1, const double* cov_d2d2, const std::size_t num_vars2, const double* cov_d1d2);

/**
 * @brief Compute variance and covariance for a given problem and Pose6d parameter. Uses @ref computeDVCovariance.
 * @param The Ceres problem (after optimization).
 * @param pose Pose6d parameter
 * @param options Options to use when calculating covariance. Use default if not explicitly set by user.
 * @return A square matrix with size (6 x 6) containing variance (diagonal elements) and covariance (off-diagonal elements).
 * Given that @ref pose contains the parameters [x, y, z, rx, ry, rz], the output matrix will be arranged like this:
 *   |x|y|z|rx|ry|rz
 * --|--------------
 * x |
 * y |
 * z |
 * rx|
 * ry|
 * rz|
 * @throw CovarianceException if computation of covariance fails for any pair of parameter blocks, or if GetCovarianceBlock returns false.
 */
Eigen::MatrixXd computePoseCovariance(ceres::Problem& problem, const Pose6d& pose,
                                      const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute off-diagonal covariance between a Pose6d parameter and a double array parameter. Uses @ref computeDV2DVCovariance.
 * @param The Ceres problem (after optimization).
 * @param pose First Pose6d parameter.
 * @param dptr Second double array parameter.
 * @param num_vars Number of elements in dptr.
 * @param options Options to use when calculating covariance. Use default if not explicitly set by user.
 * @return A matrix with size (6 x num_vars) containing the off-diagonal covariance.
 * Each row corresponds to one of the six parameters in the Pose6d parameter block, and each column corresponds to a parameter in the second parameter block.
 * Given that @ref pose contains the parameters [x, y, z, rx, ry, rz], and that @ref dptr contains the parameters [a_1 ... a_n] where n = num_vars,
 * the output matrix will be arranged like this:
 *   |a_1|a_2|...|a_n
 * --|---------------
 * x |
 * y |
 * z |
 * rx|
 * ry|
 * rz|
 * @throw CovarianceException if computation of covariance fails for any pair of parameter blocks, or if GetCovarianceBlock returns false.
 */
Eigen::MatrixXd computePose2DVCovariance(ceres::Problem &problem, const Pose6d &pose, const double* dptr, const std::size_t num_vars, const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute off-diagonal covariance between two Pose6d parameters. Uses @ref computeDV2DVCovariance.
 * @param The Ceres problem (after optimization).
 * @param p1 First pose parameter block.
 * @param p2 Second pose parameter block.
 * @param options Options to use when calculating covariance. Use default if not explicitly set by user.
 * @return A square matrix with size (6 x 6) containing the off-diagonal covariance.
 * The matrix rows correspond to the parameters in the first Pose6d parameter block, and the matrix columns correspond to the parameters in the second Pose6d parameter block.
 * Given that @ref p1 contains the parameters [x1, y1, z1, rx1, ry1, rz1] and @ref p2 contains the parameters [x2, y2, z2, rx2, ry2, rz2],
 * the output matrix will be arranged like this:
 *    |x2|y2|z2|rx2|ry2|rz2
 * ---|--------------------
 * x1 |
 * y1 |
 * z1 |
 * rx1|
 * ry1|
 * rz1|
 * @throw CovarianceException if computation of covariance fails for any pair of parameter blocks, or if GetCovarianceBlock returns false.
 */
Eigen::MatrixXd computePose2PoseCovariance(ceres::Problem &problem, const Pose6d &p1, const Pose6d &p2, const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute standard deviations and correlation coefficients for a given problem and parameter block. Uses @ref computeDV2DVCovariance.
 * @param problem The Ceres problem (after optimization).
 * @param dptr Ceres parameter block.
 * @param num_vars Number of parameters in dptr.
 * @param options Options to use when calculating covariance. Use default if not explicitly set by user.
 * @return A square matrix with size (num_vars x num_vars) containing standard deviations (diagonal elements) and correlation coefficients (off-diagonal elements).
 * Given that @ref dptr contains the parameters [a_1 ... a_n] where n = num_vars, the output matrix will be arranged like this:
 *    |a_1|a_2|...|a_n
 * ---|---------------
 * a_1|
 * a_2|
 * ...|
 * a_n|
 * @throw CovarianceException if computation of covariance fails for any pair of parameter blocks, or if GetCovarianceBlock returns false.
 */
Eigen::MatrixXd computeDVCovariance(ceres::Problem &problem, const double* dptr, const std::size_t& num_vars, const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute off-diagonal covariance for a given problem and parameters.
 * @param problem The Ceres problem (after optimization).
 * @param dptr1 First parameter block.
 * @param num_vars1 Number of parameters in dptr1.
 * @param dptr2 Second parameter block.
 * @param num_vars2 Number of parameters in dptr2.
 * @return A matrix with size (num_vars1 x num_vars2) containing the off-diagonal covariance.
 * Given that @ref dptr1 contains the parameters [a_1, a_2 ... a_n1] where n1 = num_vars1 and @ref dptr2 contains the parameters [b_1, b_2 ... b_n2] where n2 = num_vars2,
 * the output matrix will be arranged like this:
 *     |b_1|b_2|...|b_n2
 * ----|----------------
 * a_1 |
 * a_2 |
 * ... |
 * a_n1|
 * @throw CovarianceException if computation of covariance fails for any pair of parameter blocks, or if GetCovarianceBlock returns false.
 */
Eigen::MatrixXd computeDV2DVCovariance(ceres::Problem &problem, const double* dptr1, const std::size_t num_vars1, const double* dptr2, const std::size_t num_vars2, const ceres::Covariance::Options& options = DefaultCovarianceOptions());

}  // namespace rct_optimizations
