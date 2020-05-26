#include <rct_image_tools/noise_qualifier.h>
#include <rct_optimizations/experimental/pnp.h>


namespace rct_image_tools
{

//need to find correct pathing/namespace for michael's observations
//can use a fixed/ known setup to avoid having to use CV to derive an initial guess

rct_image_tools::statisitcs qualifyNoise(const rct_optimizations::CameraIntrinsics& intr, const rct_optimizations::Observations& obs)
{


  rct_image_tools::statistics output;
  std::vector<Eigen::Isometry3d> solution_transforms;

  //each accumulator handles a variable in the isometry
  std::vector<accumulator_set<double, stats<tag::mean, tag::variance>>> acc(6);

  //The initial guess for the target/transition may be needed to seed
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();

  for (auto& ob : obs)
  {
    rct_optimizations::PnPResult result;
    rct_optimizations::PnPProblem params;
    params.intr = intr;

    //NOT READY
    params.correspondences; //Need to pull a correspondence set outta the observation set
    //THIS NEEDS TO BE PASSED IN
    params.camera_to_target_guess = init_guess;

    result = rct_optimizations::optimize(params);

    //we mostly care about the isometry
    //we will save the full thing here for debugging purposes
    solution_transforms.push_back(result.camera_to_target);
    for (std::size_t i = 0; i < 6; ++i)
    {
      //this could be right
      acc(i)(result.camera_to_target(i));
    }

  }

  for (std::size_t i = 0; i < 6; ++i)
  {
    output.mean(i) = boost::accumulators::mean(acc[i]);
    output.std_dev(i) = sqrt(boost  ::accumulators::variance(acc[i]));
  }

}

 //find information from pnp
}//rct_image_tools
