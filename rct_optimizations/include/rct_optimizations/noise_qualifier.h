#pragma once
#include <Eigen/Dense>
#include "rct_optimizations/types.h"

namespace rct_optimizations
{

/**
  * @brief The NoiseStatistics struct output structure for image analysis data
  */
 struct NoiseStatistics
 {
   Eigen::VectorXd mean;
   Eigen::VectorXd std_dev;
 };

 /**
  * @brief The NoiseQualParams2D3D struct Input paramaters for 2D images: an observation set, the camera intrinsics, and the approximate camera position
  */
 struct NoiseQualParams2D3D
 {
   rct_optimizations::Observation2D3D::Set observations;
   rct_optimizations::CameraIntrinsics intr;
   Eigen::Isometry3d camera_guess;
 };

 /**
  * @brief The NoiseQualParams3D3D struct Input parameters for 3D images: an observation set and the approximate camera position
  */
 struct NoiseQualParams3D3D
 {
   rct_optimizations::Observation3D3D::Set observations;
   Eigen::Isometry3d camera_guess;
 };

 /**
  * @brief qualifyNoise2D This function qualifies 2d sensor noise
  * @param params 2D image parameters
  * @return Noise Statiscics: a vector of standard deviations and the mean pos
  */
 NoiseStatistics qualifyNoise2D(const NoiseQualParams2D3D& params);

 /**
  * @brief qualifyNoise3D This function qualifies 3d sensor noise
  * @param params 3D image parameters
  * @return Noise Statiscics: a vector of standard deviations and the mean pos
  */
 NoiseStatistics qualifyNoise3D(const NoiseQualParams3D3D& params);

}//rct_optimizations
