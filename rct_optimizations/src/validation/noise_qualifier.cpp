#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rct_optimizations/validation/noise_qualifier.h>
#include <rct_optimizations/experimental/pnp.h>

//accumulators
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

namespace rct_optimizations
{

 PnPNoiseStat qualifyNoise2D(const std::vector<PnPProblem>& params)
 {

  PnPNoiseStat output;

  std::vector<Eigen::Isometry3d> solution_transforms;
  std::vector<Eigen::Vector3d> translations;

  using namespace boost::accumulators;
  accumulator_set<double, stats<tag::mean, tag::variance>> x_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> y_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> z_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> i_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> j_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> k_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> w_acc;


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

      Eigen::Quaterniond q;
      Eigen::Matrix3d m = result.camera_to_target.rotation();
      q = m;

      i_acc(q.x());
      j_acc(q.y());
      k_acc(q.z());
      w_acc(q.w());
    }
  }

  output.x.mean = boost::accumulators::mean(x_acc);
  output.y.mean = boost::accumulators::mean(y_acc);
  output.z.mean = boost::accumulators::mean(z_acc);
  output.i.mean = boost::accumulators::mean(i_acc);
  output.j.mean = boost::accumulators::mean(j_acc);
  output.k.mean = boost::accumulators::mean(k_acc);
  output.w.mean = boost::accumulators::mean(w_acc);

  output.x.std_dev = sqrt(boost::accumulators::variance(x_acc));
  output.y.std_dev = sqrt(boost::accumulators::variance(y_acc));
  output.z.std_dev = sqrt(boost::accumulators::variance(z_acc));
  output.i.std_dev = sqrt(boost::accumulators::variance(i_acc));
  output.j.std_dev = sqrt(boost::accumulators::variance(j_acc));
  output.k.std_dev = sqrt(boost::accumulators::variance(k_acc));
  output.w.std_dev = sqrt(boost::accumulators::variance(w_acc));

  //Output: mean & standard deviation of x,y,z,i,j,k,w
  return output;
}

 PnPNoiseStat qualifyNoise3D(const std::vector<PnPProblem3D>& params)
 {

  PnPNoiseStat output;

  std::vector<Eigen::Isometry3d> solution_transforms;

  using namespace boost::accumulators;
  accumulator_set<double, stats<tag::mean, tag::variance>> x_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> y_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> z_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> i_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> j_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> k_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> w_acc;


  for (auto& prob : params)
  {
    rct_optimizations::PnPResult result;

    result = rct_optimizations::optimize(prob);

    if (result.converged)
    {
      //we will save the full result here for debugging purposes
      solution_transforms.push_back(result.camera_to_target);

      x_acc(result.camera_to_target.translation()(0));
      y_acc(result.camera_to_target.translation()(1));
      z_acc(result.camera_to_target.translation()(2));

      Eigen::Quaterniond q;
      Eigen::Matrix3d m = result.camera_to_target.rotation();
      q = m;

      i_acc(q.x());
      j_acc(q.y());
      k_acc(q.z());
      w_acc(q.w());
    }
  }

  output.x.mean = boost::accumulators::mean(x_acc);
  output.y.mean = boost::accumulators::mean(y_acc);
  output.z.mean = boost::accumulators::mean(z_acc);
  output.i.mean = boost::accumulators::mean(i_acc);
  output.j.mean = boost::accumulators::mean(j_acc);
  output.k.mean = boost::accumulators::mean(k_acc);
  output.w.mean = boost::accumulators::mean(w_acc);

  output.x.std_dev = sqrt(boost::accumulators::variance(x_acc));
  output.y.std_dev = sqrt(boost::accumulators::variance(y_acc));
  output.z.std_dev = sqrt(boost::accumulators::variance(z_acc));
  output.i.std_dev = sqrt(boost::accumulators::variance(i_acc));
  output.j.std_dev = sqrt(boost::accumulators::variance(j_acc));
  output.k.std_dev = sqrt(boost::accumulators::variance(k_acc));
  output.w.std_dev = sqrt(boost::accumulators::variance(w_acc));

  //Output: mean & standard deviation of x,y,z,i,j,k,w
  return output;
}

}//rct_optimizations
