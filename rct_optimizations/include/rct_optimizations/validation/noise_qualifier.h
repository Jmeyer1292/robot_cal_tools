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
  * @brief The RotationStat struct
  */
 struct RotationStat
 {
   NoiseStatistics qx;
   NoiseStatistics qy;
   NoiseStatistics qz;
   NoiseStatistics qw;
 };
 /**
  * @brief The PnPNoiseStat struct A collection of NoiseStatistics in a form
  * relevant to a position & orientation; xyz, quaternion
  */
 struct PnPNoiseStat
 {
   NoiseStatistics x;
   NoiseStatistics y;
   NoiseStatistics z;
   RotationStat q;
 };

 /**
  * @brief FindQuaternionMean
  * @param orientations
  * @return
  */
 RotationStat FindQuaternionMean(const std::vector<Eigen::Quaterniond>& orientations);

 /**
  * @brief qualifyNoise2D This function qualifies 2d sensor noise by
  * comparing PnP results from images taken with the same poses.
  * Sensor noise can be understood by inspecting the returned standard
  * deviations.
  * @param Sets of PnP 2D problem parameters
  * @return Noise Statistics: a vector of means & std devs
  */
PnPNoiseStat qualifyNoise2D(const std::vector<PnPProblem>& params);

 /**
  * @brief qualifyNoise3D This function qualifies 3d sensor noise by
  * comparing PnP results from scans taken with the same poses.
  * Sensor noise can be understood by inspecting the returned standard
  * deviations.
  * @param params 3D image parameters
  * @return Noise Statiscics: a vector of standard deviations and the mean pos
  */
 PnPNoiseStat qualifyNoise3D(const std::vector<PnPProblem3D>& params);

}//rct_optimizations
