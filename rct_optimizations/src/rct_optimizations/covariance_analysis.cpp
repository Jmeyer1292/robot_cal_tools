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

Eigen::MatrixXd computePoseCovariance(ceres::Problem& problem, const Eigen::Vector3d& t, Eigen::Quaterniond& q,
                                      const ceres::Covariance::Options& options)
{
  const ceres::LocalParameterization* q_loc_param = problem.GetParameterization(q.coeffs().data());
  if (q_loc_param->LocalSize() >= q.coeffs().size())
    throw CovarianceException("Locally parameterized size of the quaternion is not smaller than its original size");

  // Calculate the individual covariance matrices
  // Covariance of locally parameterized quaternion with itself
  Eigen::MatrixXd q_cov = computeDVCovariance(problem, q.coeffs().data(), q_loc_param->LocalSize());
  // Covariance of position with itself
  Eigen::MatrixXd t_cov = computeDVCovariance(problem, t.data(), t.size());
  // Covariance of locally parameterized quaternion covariance with position
  Eigen::MatrixXd qt_cov =
      computeDV2DVCovariance(problem, q.coeffs().data(), q_loc_param->LocalSize(), t.data(), t.size());

  Eigen::Index n = q_loc_param->LocalSize() + t.size();
  Eigen::MatrixXd cov;
  cov.resize(n, n);
  /* Total covariance matrix
   *      Q        P
   * Q | C(q,q) | C(q, p) |
   * P | C(p,q) | C(p, p) |
   */
  cov.resize(6, 6);
  cov.block<3, 3>(0, 0) = q_cov;
  cov.block<3, 3>(3, 3) = t_cov;
  cov.block<3, 3>(0, 3) = qt_cov;
  cov.block<3, 3>(3, 0) = qt_cov.transpose();

  return cov;
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

}  // namespace rct_optimizations
