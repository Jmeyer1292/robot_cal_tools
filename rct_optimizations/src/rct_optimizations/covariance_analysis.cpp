#include <rct_optimizations/covariance_analysis.h>

namespace rct_optimizations
{

Eigen::MatrixXd covToEigenCorr(const double* cov, const std::size_t num_vars)
{
  Eigen::MatrixXd out(num_vars, num_vars);

  for (std::size_t i = 0; i < num_vars; i++)
  {
    double sigma_i = sqrt(fabs(cov[i * num_vars + i]));  // standard deviation at the diagonal element in row i

    for (std::size_t j = 0; j < num_vars; j++)
    {
      double sigma_j = sqrt(fabs(cov[j * num_vars + j]));  // standard deviation at the diagonal element in column j

      if (i == j)  // if out(i,j) is a diagonal element
        out(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = sigma_i;
      else
      {
        if (sigma_i < std::numeric_limits<double>::epsilon()) sigma_i = 1;
        if (sigma_j < std::numeric_limits<double>::epsilon()) sigma_j = 1;

        out(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = cov[i * num_vars + j] / (sigma_i * sigma_j);
      }
    }
  }
  return out;
}

Eigen::MatrixXd covToEigenCov(const double* cov, const std::size_t num_vars)
{
  Eigen::MatrixXd out(num_vars, num_vars);

  for (std::size_t i = 0; i < num_vars; i++)
  {
    for (std::size_t j = 0; j < num_vars; j++)
    {
      out(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = cov[i * num_vars + j];
    }
  }
  return out;
}

Eigen::MatrixXd covToEigenOffDiagCorr(const double* cov_d1d1, const std::size_t num_vars1, const double* cov_d2d2, const std::size_t num_vars2, const double* cov_d1d2)
{
  Eigen::MatrixXd out(num_vars1, num_vars2);

  for (std::size_t i = 0; i < num_vars1; i++)
  {
    double sigma_i = sqrt(fabs(cov_d1d1[i * num_vars1 + i]));  // standard deviation at the diagonal element in row i of cov_d1d1
    for (std::size_t j = 0; j < num_vars2; j++)
    {
      double sigma_j = sqrt(fabs(cov_d2d2[j * num_vars2 + j]));  // standard deviation at the diagonal element in column j of cov_d2d2

      if (sigma_i < std::numeric_limits<double>::epsilon()) sigma_i = 1;
      if (sigma_j < std::numeric_limits<double>::epsilon()) sigma_j = 1;

      out(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = cov_d1d2[i * num_vars1 + j] / (sigma_i * sigma_j);
    }
  }
  return out;
}

Eigen::MatrixXd computePoseCovariance(ceres::Problem& problem, const Pose6d& pose, const ceres::Covariance::Options& options)
{
  return computeDVCovariance(problem, pose.values.data(), 6, options);
}

Eigen::MatrixXd computePose2DVCovariance(ceres::Problem &problem, const Pose6d &pose, const double* dptr, std::size_t num_vars, const ceres::Covariance::Options& options)
{
  return computeDV2DVCovariance(problem, pose.values.data(), 6, dptr, num_vars, options);
}

Eigen::MatrixXd computePose2PoseCovariance(ceres::Problem &problem, const Pose6d &p1, const Pose6d &p2, const ceres::Covariance::Options& options)
{
  return computeDV2DVCovariance(problem, p1.values.data(), 6, p2.values.data(), 6, options);
}

Eigen::MatrixXd computeDVCovariance(ceres::Problem &problem, const double * dptr, const std::size_t& num_vars, const ceres::Covariance::Options& options)
{
  ceres::Covariance covariance(options);

  std::vector<std::pair<const double*, const double*> > covariance_pairs;
  covariance_pairs.push_back(std::make_pair(dptr, dptr));

  if(!covariance.Compute(covariance_pairs, &problem))
    throw CovarianceException("Could not compute covariance in computeDVCovariance()");

  double cov[num_vars*num_vars];
  if(!covariance.GetCovarianceBlockInTangentSpace(dptr, dptr, cov))
    throw CovarianceException("GetCovarianceBlock failed in computeDVCovariance()");

  return covToEigenCorr(cov, num_vars);
}

Eigen::MatrixXd computeDV2DVCovariance(ceres::Problem &P, const double* dptr1, const std::size_t num_vars1, const double* dptr2, const std::size_t num_vars2, const ceres::Covariance::Options& options)
{
  ceres::Covariance covariance(options);

  std::vector<std::pair<const double*, const double*> > covariance_pairs;
  covariance_pairs.push_back(std::make_pair(dptr1, dptr1));
  covariance_pairs.push_back(std::make_pair(dptr2, dptr2));
  covariance_pairs.push_back(std::make_pair(dptr1, dptr2));

  if(!covariance.Compute(covariance_pairs, &P))
    throw CovarianceException("Could not compute covariance in computeDV2DVCovariance()");

  double cov_d1d1[num_vars1*num_vars2], cov_d2d2[num_vars2*num_vars2], cov_d1d2[num_vars1*num_vars2];
  if(!(covariance.GetCovarianceBlockInTangentSpace(dptr1, dptr1, cov_d1d1) &&
       covariance.GetCovarianceBlockInTangentSpace(dptr2, dptr2, cov_d2d2) &&
       covariance.GetCovarianceBlockInTangentSpace(dptr1, dptr2, cov_d1d2)))
    throw CovarianceException("GetCovarianceBlock failed in computeDV2DVCovariance()");

  return covToEigenOffDiagCorr(cov_d1d1, num_vars1, cov_d2d2, num_vars2, cov_d1d2);
}

Eigen::MatrixXd computeFullDV2DVCovariance(ceres::Problem &problem,
                                           const double *dptr1,
                                           const std::size_t num_vars1,
                                           const double *dptr2,
                                           const std::size_t num_vars2,
                                           const ceres::Covariance::Options &options)
{
  // Calculate the individual covariance matrices
  // Covariance of parameter 1 with itself
  Eigen::MatrixXd cov_p1 = computeDVCovariance(problem, dptr1, num_vars1, options);
  // Covariance of parameter 2 with itself
  Eigen::MatrixXd cov_p2 = computeDVCovariance(problem, dptr2, num_vars2, options);
  // Covariance of parameter 1 with parameter 2
  Eigen::MatrixXd cov_p1p2
    = computeDV2DVCovariance(problem, dptr1, num_vars1, dptr2, num_vars2, options);

  // Total covariance matrix
  Eigen::MatrixXd cov;
  const std::size_t n = num_vars1 + num_vars2;
  cov.resize(n, n);

  /*    |     P1    |     P2    |
   * ---|-----------|-----------|
   * P1 | C(p1, p1) | C(p1, p2) |
   * P2 | C(p2, p1) | C(p2, p2) |
   */
  cov.block(0, 0, num_vars1, num_vars1) = cov_p1;
  cov.block(num_vars1, num_vars1, num_vars2, num_vars2) = cov_p2;
  cov.block(0, num_vars1, num_vars1, num_vars2) = cov_p1p2;
  cov.block(num_vars1, 0, num_vars2, num_vars1) = cov_p1p2.transpose();

  return cov;
}

}  // namespace rct_optimizations
