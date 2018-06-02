#ifndef RCT_EIGEN_CONVERSIONS_H
#define RCT_EIGEN_CONVERSIONS_H

#include "rct_optimizations/types.h"
#include <Eigen/Dense>

namespace rct_optimizations
{

Pose6d poseEigenToCal(const Eigen::Affine3d& pose);

Eigen::Affine3d poseCalToEigen(const Pose6d& pose);

}

#endif // RCT_EIGEN_CONVERSIONS_H
