#pragma once
#include <Eigen/Dense>
#include "rct_optimizations/types.h"
#include "rct_optimizations/experimental/pnp.h"

namespace rct_optimizations
{

/**
  * @brief The NoiseStatistics struct output structure for image analysis data
  */
 struct NoiseStatistics
 {
   double mean;
   double std_dev;
 };

 /**
  * @brief qualifyNoise2D This function qualifies 2d sensor noise
  * @param params 2D image parameters
  * @return Noise Statiscics: a vector of standard deviations and the mean pos
  */
 std::vector<NoiseStatistics> qualifyNoise2D(const std::vector<PnPProblem>& params);

 /**
  * @brief qualifyNoise3D This function qualifies 3d sensor noise
  * @param params 3D image parameters
  * @return Noise Statiscics: a vector of standard deviations and the mean pos
  */
 std::vector<NoiseStatistics> qualifyNoise3D(const std::vector<PnPProblem3D>& params);

}//rct_optimizations
