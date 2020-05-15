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
    this->algorithm_type = ceres::DENSE_SVD;
  }
};

/**
 * @brief Calculate standard deviations and correlation coefficients given a double array covariance. Each diagonal element is the standard deviation of
 * a parameter, and its units match the units of that parameter. Each off-diagonal element is the correlation coefficients of a pair of parameters, which
 * is unitless and in the range [-1, 1].
 * @param cov Points to a double array containing covariance results from Ceres.
 * @param num_vars Number of elements in cov.
 * @return A square matrix with size (num_vars x num_vars) containing standard deviations (diagonal elements) and correlation coefficients (off-diagonal elements).
 */
Eigen::MatrixXd covToEigenCorr(double* cov, std::size_t num_vars);

/**
 * @brief Arrange the covariance results from Ceres as an Eigen matrix.
 * @param cov Points to a double array containing covariance results from Ceres.
 * @param num_vars Number of elements in cov.
 * @return A square matrix with size (num_vars x num_vars) containing variance (diagonal elements) and covariance (off-diagonal elements).
 */
Eigen::MatrixXd covToEigenCov(double* cov, std::size_t num_vars);


/**
 * @brief Calculate the off-diagonal correlation matrix given Ceres covariances.
 * @param cov_d1d1 Pointer to a double array containing covariance for the first parameter block.
 * @param num_vars1 Number of elements in cov_d1d1
 * @param cov_d2d2 Pointer to a double array containing covariance for the second parameter block.
 * @param num_vars2 Number of elements in cov_d1d2
 * @param cov_d1d2 Pointer to a double array containing correlation coefficients for the first parameter block relative to the second parameter block.
 * Assumed to contain (num_vars1 x num_vars2 elements).
 * @return A matrix with size (num_vars1 x num_vars2) containing the off-diagonal covariance.
 */
Eigen::MatrixXd covToEigenOffDiagCorr(double* cov_d1d1, std::size_t num_vars1, double* cov_d2d2, std::size_t num_vars2, double* cov_d1d2);

/**
 * @brief Compute variance and covariance for a given problem and Pose6d parameter.
 * @param The Ceres problem (after optimization).
 * @param pose Pose6d parameter
 * @param options Options to use when calculating covariance. Use default if not explicitly set by user.
 * @return A square matrix with size (6 x 6) containing variance (diagonal elements) and covariance (off-diagonal elements).
 */
Eigen::MatrixXd computePoseCovariance(ceres::Problem& problem, Pose6d& pose, const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute off-diagonal covariance between a Pose6d parameter and a double array parameter.
 * @param The Ceres problem (after optimization).
 * @param pose First Pose6d parameter.
 * @param dptr Second double array parameter.
 * @param num_vars Number of elements in dptr.
 * @param options Options to use when calculating covariance. Use default if not explicitly set by user.
 * @return A matrix with size (6 x num_vars) containing the off-diagonal covariance.
 */
Eigen::MatrixXd computePose2DVCovariance(ceres::Problem &problem, Pose6d &pose, double* dptr, std::size_t num_vars, const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute off-diagonal covariance between two Pose6d parameters.
 * @param The Ceres problem (after optimization).
 * @param p1 First pose parameter.
 * @param p2 Second pose parameter.
 * @param options Options to use when calculating covariance. Use default if not explicitly set by user.
 * @return A square matrix with size (6 x 6) containing the off-diagonal covariance.
 */
Eigen::MatrixXd computePose2PoseCovariance(ceres::Problem &problem, Pose6d &p1, Pose6d &p2, const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute standard deviations and correlation coefficients for a given problem and parameter block.
 * @param problem The Ceres problem (after optimization).
 * @param dptr Ceres parameter block.
 * @param num_vars Number of parameters in dptr.
 * @param options Options to use when calculating covariance. Use default if not explicitly set by user.
 * @return A square matrix with size (num_vars x num_vars) containing standard deviations (diagonal elements) and correlation coefficients (off-diagonal elements).
 */
Eigen::MatrixXd computeDVCovariance(ceres::Problem &problem, double *dptr, std::size_t num_vars, const ceres::Covariance::Options& options = DefaultCovarianceOptions());

/**
 * @brief Compute off-diagonal covariance for a given problem and parameters.
 * @param problem The Ceres problem (after optimization).
 * @param dptr1 First parameter block.
 * @param num_vars1 Number of parameters in dptr1.
 * @param dptr2 Second parameter block.
 * @param num_vars2 Number of parameters in dptr2.
 * @return A matrix with size (num_vars1 x num_vars2) containing the off-diagonal covariance.
 */
Eigen::MatrixXd computeDV2DVCovariance(ceres::Problem &problem, double* dptr1, std::size_t num_vars1, double* dptr2, std::size_t num_vars2, const ceres::Covariance::Options& options = DefaultCovarianceOptions());

}  // namespace rct_optimizations
