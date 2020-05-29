#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rct_image_tools/noise_qualifier.h>
#include <rct_optimizations/experimental/pnp.h>

//accumulators
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>



namespace rct_image_tools
{

//need to find correct pathing/namespace for michael's observations
//can use a fixed/ known setup to avoid having to use CV to derive an initial guess

 rct_image_tools::NoiseStatistics qualifyNoise(const NoiseQualParams2D3D &params)
 {

  rct_image_tools::NoiseStatistics output;

  std::vector<Eigen::Isometry3d> solution_transforms;

  //This could be the completely wrong approach for this problem
  //Ideal: a vector of accumulator_sets to handle everything very cleanly
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


    pnpParams.correspondences = ob.correspondence_set; //Need to pull a correspondence set outta the observation set? Does not follow heigharchy from pr comment
    pnpParams.camera_to_target_guess = params.camera_guess;

    result = rct_optimizations::optimize(pnpParams);

    //we will save the full result here for debugging purposes
    solution_transforms.push_back(result.camera_to_target);

    //alternatively, find matrix norm & std dev for each matrix?

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


//3D-3D, doesn't work bc there is no PnP to use

// rct_image_tools::NoiseStatistics qualifyNoise(const NoiseQualParams3D3D &params)
// {

//  rct_image_tools::NoiseStatistics output;

//  std::vector<Eigen::Isometry3d> solution_transforms;

//  //This could be the completely wrong approach for this problem
//  //Ideal: a vector of accumulator_sets to handle everything very cleanly
//  using namespace boost::accumulators;
//  accumulator_set<double, stats<tag::mean, tag::variance>> x_acc;
//  accumulator_set<double, stats<tag::mean, tag::variance>> y_acc;
//  accumulator_set<double, stats<tag::mean, tag::variance>> z_acc;
//  accumulator_set<double, stats<tag::mean, tag::variance>> r_acc;
//  accumulator_set<double, stats<tag::mean, tag::variance>> p_acc;
//  accumulator_set<double, stats<tag::mean, tag::variance>> yw_acc;


//  for (auto& ob : params.observations)
//  {
//    rct_optimizations::PnPResult result;
//    rct_optimizations::PnPProblem pnpParams;
//    pnpParams.intr = params.intr;


//    pnpParams.correspondences = ob.correspondence_set; //Need to pull a correspondence set outta the observation set? Does not follow heigharchy from pr comment
//    pnpParams.camera_to_target_guess = params.camera_guess;

//    result = rct_optimizations::optimize(pnpParams);

//    //we will save the full result here for debugging purposes
//    solution_transforms.push_back(result.camera_to_target);

//    x_acc(result.camera_to_target.translation()(0));
//    y_acc(result.camera_to_target.translation()(1));
//    z_acc(result.camera_to_target.translation()(2));

////    Need a good way to track the rotational values in the position
////    similar transforms should give similar eas (caveat: tranforms near singularities are vulnerable)
//    Eigen::Vector3d ea = result.camera_to_target.linear().eulerAngles(0,1,2);
//    r_acc(ea(0));
//    p_acc(ea(1));
//    yw_acc(ea(2));

//  }


//  output.mean(0) = boost::accumulators::mean(x_acc);
//  output.mean(1) = boost::accumulators::mean(y_acc);
//  output.mean(2) = boost::accumulators::mean(z_acc);
//  output.mean(3) = boost::accumulators::mean(r_acc);
//  output.mean(4) = boost::accumulators::mean(p_acc);
//  output.mean(5) = boost::accumulators::mean(yw_acc);

//  output.std_dev(0) = sqrt(boost::accumulators::variance(x_acc));
//  output.std_dev(1) = sqrt(boost::accumulators::variance(y_acc));
//  output.std_dev(2) = sqrt(boost::accumulators::variance(z_acc));
//  output.std_dev(3) = sqrt(boost::accumulators::variance(r_acc));
//  output.std_dev(4) = sqrt(boost::accumulators::variance(p_acc));
//  output.std_dev(5) = sqrt(boost::accumulators::variance(yw_acc));

//  //Output: mean & standard deviation of x,y,z,roll,pitch,yaw
//  return output;
//}

}//rct_image_tools
