#pragma once
#include <Eigen/Dense>
#include "rct_optimizations/types.h"

namespace rct_optimizations
{

 struct NoiseStatistics
 {
   Eigen::VectorXd mean;
   Eigen::VectorXd std_dev;
 };

 struct NoiseQualParams2D3D
 {
   //NoiseQualParams2D3D(rct_optimizations::Observation2D3D::Set, rct_optimizations::CameraIntrinsics, Eigen::Isometry3d camera_guess)
   rct_optimizations::Observation2D3D::Set observations;
   rct_optimizations::CameraIntrinsics intr;
   Eigen::Isometry3d camera_guess;
 };

 struct NoiseQualParams3D3D
 {
   rct_optimizations::Observation3D3D::Set observations;
   Eigen::Isometry3d camera_guess;
 };

 NoiseStatistics qualifyNoise2D(const NoiseQualParams2D3D& params);
 NoiseStatistics qualifyNoise3D(const NoiseQualParams3D3D& params);

}//rct_optimizations
