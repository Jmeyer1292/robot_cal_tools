#pragma once
#include <Eigen/Dense>
//include pnp

//psuedocode:

namespace rct_image_tools
{

 struct statisitcs
 {
   Eigen::Isometry3d mean;
   Eigen::Isometry3d std_dev;
 }

 //need to find correct pathing/namespace for michael's observations
 statistics qualifyNoise(const rct_optimizations::Observations, const rct_optimizations::test::Camera);

 //find information from pnp
}//rct_image_tools
