#pragma once
#include <rct_optimizations/types.h>

namespace rct_optimizations
{
/**
 * @brief checkObservationSequence verifies that the circles on a calibration
 * target are sequenced correctly using Homography, the mapping of image
 * points to a plane in the image. This method assumes that the target is
 * planar and that all target points are visible in the observation. It is the
 *  user's responsibility to provide a reasonable error threshold.
 * @param intr Camera intrinsics
 * @param target Target desciption; rows, cols, and point spacing
 * @param ob Observation set
 * @param max_residual Maximium expected error
 * @return A boolian indicating wheather the target points are sequenced
 * corectly in the observation
 */
bool checkObservationSequence(const int& rows,
                              const int& cols,
                              const rct_optimizations::Observation2D3D& ob,
                              const double& max_residual);

}//rct_optimizations
