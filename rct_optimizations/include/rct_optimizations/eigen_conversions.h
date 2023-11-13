#ifndef RCT_EIGEN_CONVERSIONS_H
#define RCT_EIGEN_CONVERSIONS_H

#include "rct_optimizations/types.h"
#include <Eigen/Dense>

namespace rct_optimizations
{
Pose6d poseEigenToCal(const Eigen::Isometry3d& pose);

Eigen::Isometry3d poseCalToEigen(const Pose6d& pose);

}  // namespace rct_optimizations

#endif  // RCT_EIGEN_CONVERSIONS_H
