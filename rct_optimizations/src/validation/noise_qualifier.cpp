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
  accumulator_set<double, stats<tag::mean, tag::variance>> r_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> p_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> yw_acc;


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

      //Eigen::Vector3d ea = result.camera_to_target.linear().eulerAngles(0,1,2);
      Eigen::AngleAxisd aa;
      Eigen::Matrix3d m = result.camera_to_target.rotation();
      aa = m;

      r_acc(aa.axis()(0));
      p_acc(aa.axis()(1));
      yw_acc(aa.axis()(2));
    }
  }

  output.x.mean = boost::accumulators::mean(x_acc);
  output.y.mean = boost::accumulators::mean(y_acc);
  output.z.mean = boost::accumulators::mean(z_acc);
  output.r.mean = boost::accumulators::mean(r_acc);
  output.p.mean = boost::accumulators::mean(p_acc);
  output.yw.mean = boost::accumulators::mean(yw_acc);

  output.x.std_dev = sqrt(boost::accumulators::variance(x_acc));
  output.y.std_dev = sqrt(boost::accumulators::variance(y_acc));
  output.z.std_dev = sqrt(boost::accumulators::variance(z_acc));
  output.r.std_dev = sqrt(boost::accumulators::variance(r_acc));
  output.p.std_dev = sqrt(boost::accumulators::variance(p_acc));
  output.yw.std_dev = sqrt(boost::accumulators::variance(yw_acc));

  //Output: mean & standard deviation of x,y,z,roll,pitch,yaw
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
  accumulator_set<double, stats<tag::mean, tag::variance>> r_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> p_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> yw_acc;


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

      Eigen::AngleAxisd aa;
      Eigen::Matrix3d m = result.camera_to_target.rotation();
      aa = m;

      r_acc(aa.axis()(0));
      p_acc(aa.axis()(1));
      yw_acc(aa.axis()(2));
    }
  }

  output.x.mean = boost::accumulators::mean(x_acc);
  output.y.mean = boost::accumulators::mean(y_acc);
  output.z.mean = boost::accumulators::mean(z_acc);
  output.r.mean = boost::accumulators::mean(r_acc);
  output.p.mean = boost::accumulators::mean(p_acc);
  output.yw.mean = boost::accumulators::mean(yw_acc);

  output.x.std_dev = sqrt(boost::accumulators::variance(x_acc));
  output.y.std_dev = sqrt(boost::accumulators::variance(y_acc));
  output.z.std_dev = sqrt(boost::accumulators::variance(z_acc));
  output.r.std_dev = sqrt(boost::accumulators::variance(r_acc));
  output.p.std_dev = sqrt(boost::accumulators::variance(p_acc));
  output.yw.std_dev = sqrt(boost::accumulators::variance(yw_acc));

  //Output: mean & standard deviation of x,y,z,roll,pitch,yaw
  return output;
}

}//rct_optimizations
