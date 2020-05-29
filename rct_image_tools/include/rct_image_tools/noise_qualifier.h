#pragma once
#include <Eigen/Dense>
#include "rct_optimizations/types.h"
//include pnp

//psuedocode:

namespace rct_image_tools
{

 struct NoiseStatistics
 {
   Eigen::VectorXd mean;
   Eigen::VectorXd std_dev;
 };

 struct NoiseQualParams2D3D
 {
   typename rct_optimizations::Observation2D3D::Set observations;
   rct_optimizations::CameraIntrinsics intr;
   Eigen::Isometry3d camera_guess;
 };

 struct NoiseQualParams3D3D
 {
   typename rct_optimizations::Observation3D3D::Set observations;
   rct_optimizations::CameraIntrinsics intr;
   Eigen::Isometry3d camera_guess;
 };

 //need to find correct pathing/namespace for michael's observations
 rct_image_tools::NoiseStatistics qualifyNoise(const NoiseQualParams2D3D &params);
 rct_image_tools::NoiseStatistics qualifyNoise(const NoiseQualParams3D3D &params);

}//rct_image_tools
