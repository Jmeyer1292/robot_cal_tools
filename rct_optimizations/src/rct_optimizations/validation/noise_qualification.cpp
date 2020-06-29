#include <Eigen/Eigenvalues>
#include <rct_optimizations/validation/noise_qualification.h>
#include <rct_optimizations/pnp.h>

//accumulators
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <cassert>

namespace rct_optimizations
{

Eigen::Quaterniond computeQuaternionMean(const std::vector<Eigen::Quaterniond>& quaterns)
{
 /* Mean quaternion is found using method described by Markley et al: Quaternion Averaging
  * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
  *
  * M = sum(w_i * q_i * q_i^T)    Eq. 12
  * q_bar = argmax(q^T * M * q)   Eq. 13
  *
  * The solution of this maximization problem is well known. The average quaternion is
  * the eigenvector of M corresponding to the maximum eigenvalue.
  *
  * In the above equations, w_i is the weight of the ith quaternion.
  * In this case, all quaternions are equally weighted (i.e. w_i = 1)
  */

  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

  for(const Eigen::Quaterniond& q : quaterns)
  {
    M += q.coeffs() * q.coeffs().transpose();
  }

  // Eigenvectors,values should be strictly real
  Eigen::EigenSolver<Eigen::Matrix4d> E(M, true);

  // Each column of 4x4 vectors is an eigenvector; desired mean has max
  Eigen::Index idx_max_ev; // Index of the largest eigenvalue

  //find maximium eigenvalue, and store its index in max_evi
  E.eigenvalues().real().maxCoeff(&idx_max_ev);

  Eigen::Quaterniond q;
  q.coeffs() << E.eigenvectors().real().col(idx_max_ev);

  assert(std::isnan(q.w()) == false &&
         std::isnan(q.x()) == false &&
         std::isnan(q.y()) == false &&
         std::isnan(q.z()) == false);

  return q;
};

QuaternionStats computeQuaternionStats(const std::vector<Eigen::Quaterniond> &quaternions)
{
  QuaternionStats q_stats;
  q_stats.mean = computeQuaternionMean(quaternions);

  double q_var = 0.0;
  for (const Eigen::Quaterniond &q : quaternions)
  {
    q_var += std::pow(q_stats.mean.angularDistance(q), 2.0);
  }
  q_var /= static_cast<double>(quaternions.size());
  q_stats.stdev = std::sqrt(q_var);

  return q_stats;
}

PnPNoiseStat qualifyNoise2D(const std::vector<PnPProblem>& params)
{
  PnPNoiseStat output;
  std::size_t count = params.size();

  std::vector<Eigen::Isometry3d> solution_transforms;
  solution_transforms.reserve(count);

  std::vector<Eigen::Vector3d> translations;
  translations.reserve(count);

  std::vector<Eigen::Quaterniond> orientations;
  orientations.reserve(count);

  namespace ba = boost::accumulators;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> x_acc;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> y_acc;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> z_acc;

  for (auto& prob : params)
  {
    rct_optimizations::PnPResult result;

    result = rct_optimizations::optimize(prob);

    if (result.converged)
    {
      //we will save the full result here for debugging purposes
      solution_transforms.push_back(result.camera_to_target);
      translations.push_back(solution_transforms.back().translation());

      x_acc(result.camera_to_target.translation()(0));
      y_acc(result.camera_to_target.translation()(1));
      z_acc(result.camera_to_target.translation()(2));

      orientations.push_back(Eigen::Quaterniond(result.camera_to_target.rotation()));
    }
  }

  output.p_stat.mean << ba::mean(x_acc), ba::mean(y_acc), ba::mean(z_acc);
  output.p_stat.stdev << std::sqrt(ba::variance(x_acc)), std::sqrt(ba::variance(y_acc)), std::sqrt(ba::variance(z_acc));
  output.q_stat = computeQuaternionStats(orientations);

  return output;
}

PnPNoiseStat qualifyNoise3D(const std::vector<PnPProblem3D>& params)
{
  PnPNoiseStat output;
  std::size_t count = params.size();

  std::vector<Eigen::Isometry3d> solution_transforms;
  solution_transforms.reserve(count);

  std::vector<Eigen::Vector3d> translations;
  translations.reserve(count);

  std::vector<Eigen::Quaterniond> orientations;
  orientations.reserve(count);

  namespace ba = boost::accumulators;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> x_acc;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> y_acc;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> z_acc;

  for (auto& prob : params)
  {
    rct_optimizations::PnPResult result;

    result = rct_optimizations::optimize(prob);

    if (result.converged)
    {
      //we will save the full result here for debugging purposes
      solution_transforms.push_back(result.camera_to_target);
      translations.push_back(solution_transforms.back().translation());

      x_acc(result.camera_to_target.translation()(0));
      y_acc(result.camera_to_target.translation()(1));
      z_acc(result.camera_to_target.translation()(2));

      orientations.push_back(Eigen::Quaterniond(result.camera_to_target.rotation()));
    }
  }

  output.p_stat.mean << ba::mean(x_acc), ba::mean(y_acc), ba::mean(z_acc);
  output.p_stat.stdev << std::sqrt(ba::variance(x_acc)), std::sqrt(ba::variance(y_acc)), std::sqrt(ba::variance(z_acc));
  output.q_stat = computeQuaternionStats(orientations);

  return output;
}

}//rct_optimizations
