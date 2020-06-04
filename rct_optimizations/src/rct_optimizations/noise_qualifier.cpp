#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rct_optimizations/noise_qualifier.h>
#include <rct_optimizations/experimental/pnp.h>

//accumulators
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>



namespace rct_optimizations
{

 NoiseStatistics qualifyNoise2D(const NoiseQualParams2D3D& params)
 {

  rct_optimizations::NoiseStatistics output;
  Eigen::VectorXd tempmean(6);
  Eigen::VectorXd tempdev(6);

  std::vector<Eigen::Isometry3d> solution_transforms;

  using namespace boost::accumulators;
  accumulator_set<double, stats<tag::mean, tag::variance>> x_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> y_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> z_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> r_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> p_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> yw_acc;


  for (auto& ob : params.observations)
  {
    rct_optimizations::PnPResult result;
    rct_optimizations::PnPProblem pnpParams;
    pnpParams.intr = params.intr;


    pnpParams.correspondences = ob.correspondence_set;
    pnpParams.camera_to_target_guess = params.camera_guess;

    result = rct_optimizations::optimize(pnpParams);

    //we will save the full result here for debugging purposes
    solution_transforms.push_back(result.camera_to_target);

    //alternatively, find matrix norm & std dev for each matrix?

    x_acc(result.camera_to_target.translation()(0));
    y_acc(result.camera_to_target.translation()(1));
    z_acc(result.camera_to_target.translation()(2));

//    Need a good way to track the rotational values in the position (use poseXd)
//    similar transforms should give similar eas (caveat: tranforms near singularities are vulnerable)
    Eigen::Vector3d ea = result.camera_to_target.linear().eulerAngles(0,1,2);
    r_acc(ea(0));
    p_acc(ea(1));
    yw_acc(ea(2));

  }

  tempmean(0) = boost::accumulators::mean(x_acc);
  tempmean(1) = boost::accumulators::mean(y_acc);
  tempmean(2) = boost::accumulators::mean(z_acc);
  tempmean(3) = boost::accumulators::mean(r_acc);
  tempmean(4) = boost::accumulators::mean(p_acc);
  tempmean(5) = boost::accumulators::mean(yw_acc);

  tempdev(0) = sqrt(boost::accumulators::variance(x_acc));
  tempdev(1) = sqrt(boost::accumulators::variance(y_acc));
  tempdev(2) = sqrt(boost::accumulators::variance(z_acc));
  tempdev(3) = sqrt(boost::accumulators::variance(r_acc));
  tempdev(4) = sqrt(boost::accumulators::variance(p_acc));
  tempdev(5) = sqrt(boost::accumulators::variance(yw_acc));

  //Output: mean & standard deviation of x,y,z,roll,pitch,yaw
  output.mean = tempmean;
  output.std_dev = tempdev;
  return output;
}

 rct_optimizations::NoiseStatistics qualifyNoise3D(const NoiseQualParams3D3D &params)
 {

  rct_optimizations::NoiseStatistics output;

  std::vector<Eigen::Isometry3d> solution_transforms;

  using namespace boost::accumulators;
  accumulator_set<double, stats<tag::mean, tag::variance>> x_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> y_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> z_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> r_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> p_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> yw_acc;


  for (auto& ob : params.observations)
  {
    rct_optimizations::PnPResult result;
    rct_optimizations::PnPProblem3D pnpParams;

    pnpParams.correspondences = ob.correspondence_set;
    pnpParams.camera_to_target_guess = params.camera_guess;

    result = rct_optimizations::optimize(pnpParams);

    //we will save the full result here for debugging purposes
    solution_transforms.push_back(result.camera_to_target);

    x_acc(result.camera_to_target.translation()(0));
    y_acc(result.camera_to_target.translation()(1));
    z_acc(result.camera_to_target.translation()(2));

//    Need a good way to track the rotational values in the position
//    similar transforms should give similar eas (caveat: tranforms near singularities are vulnerable)
    Eigen::Vector3d ea = result.camera_to_target.linear().eulerAngles(0,1,2);
    r_acc(ea(0));
    p_acc(ea(1));
    yw_acc(ea(2));

  }

  output.mean(0) = boost::accumulators::mean(x_acc);
  output.mean(1) = boost::accumulators::mean(y_acc);
  output.mean(2) = boost::accumulators::mean(z_acc);
  output.mean(3) = boost::accumulators::mean(r_acc);
  output.mean(4) = boost::accumulators::mean(p_acc);
  output.mean(5) = boost::accumulators::mean(yw_acc);

  output.std_dev(0) = sqrt(boost::accumulators::variance(x_acc));
  output.std_dev(1) = sqrt(boost::accumulators::variance(y_acc));
  output.std_dev(2) = sqrt(boost::accumulators::variance(z_acc));
  output.std_dev(3) = sqrt(boost::accumulators::variance(r_acc));
  output.std_dev(4) = sqrt(boost::accumulators::variance(p_acc));
  output.std_dev(5) = sqrt(boost::accumulators::variance(yw_acc));

  //Output: mean & standard deviation of x,y,z,roll,pitch,yaw
  return output;
}

}//rct_optimizations
