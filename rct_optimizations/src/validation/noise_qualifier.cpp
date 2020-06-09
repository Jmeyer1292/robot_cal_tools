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

 std::vector<NoiseStatistics> qualifyNoise2D(const std::vector<PnPProblem>& params)
 {

  std::vector<NoiseStatistics> output;
  output.reserve(6);

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

      Eigen::Vector3d ea = result.camera_to_target.linear().eulerAngles(0,1,2);
      r_acc(ea(0));
      p_acc(ea(1));
      yw_acc(ea(2));
    }
  }

  output[0].mean = boost::accumulators::mean(x_acc);
  output[1].mean = boost::accumulators::mean(y_acc);
  output[2].mean = boost::accumulators::mean(z_acc);
  output[3].mean = boost::accumulators::mean(r_acc);
  output[4].mean = boost::accumulators::mean(p_acc);
  output[5].mean = boost::accumulators::mean(yw_acc);

  output[0].std_dev = sqrt(boost::accumulators::variance(x_acc));
  output[1].std_dev = sqrt(boost::accumulators::variance(y_acc));
  output[2].std_dev = sqrt(boost::accumulators::variance(z_acc));
  output[3].std_dev = sqrt(boost::accumulators::variance(r_acc));
  output[4].std_dev = sqrt(boost::accumulators::variance(p_acc));
  output[5].std_dev = sqrt(boost::accumulators::variance(yw_acc));

  //Output: mean & standard deviation of x,y,z,roll,pitch,yaw
  return output;
}

 std::vector<NoiseStatistics> qualifyNoise3D(const std::vector<PnPProblem3D>& params)
 {

  std::vector<NoiseStatistics> output;
  output.reserve(6);

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

      //similar transforms should give similar eas (caveat: tranforms near singularities are vulnerable)
      Eigen::Vector3d ea = result.camera_to_target.linear().eulerAngles(0,1,2);
      r_acc(ea(0));
      p_acc(ea(1));
      yw_acc(ea(2));
    }
  }

  output[0].mean = boost::accumulators::mean(x_acc);
  output[1].mean = boost::accumulators::mean(y_acc);
  output[2].mean = boost::accumulators::mean(z_acc);
  output[3].mean = boost::accumulators::mean(r_acc);
  output[4].mean = boost::accumulators::mean(p_acc);
  output[5].mean = boost::accumulators::mean(yw_acc);

  output[0].std_dev = sqrt(boost::accumulators::variance(x_acc));
  output[1].std_dev = sqrt(boost::accumulators::variance(y_acc));
  output[2].std_dev = sqrt(boost::accumulators::variance(z_acc));
  output[3].std_dev = sqrt(boost::accumulators::variance(r_acc));
  output[4].std_dev = sqrt(boost::accumulators::variance(p_acc));
  output[5].std_dev = sqrt(boost::accumulators::variance(yw_acc));

  //Output: mean & standard deviation of x,y,z,roll,pitch,yaw
  return output;
}

}//rct_optimizations
