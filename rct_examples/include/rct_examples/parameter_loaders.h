#ifndef RCT_PARAMETER_LOADERS_H
#define RCT_PARAMETER_LOADERS_H

#include <rct_image_tools/modified_circle_grid_target.h>
#include <rct_optimizations/types.h>
#include <ros/node_handle.h>

namespace rct_examples
{

bool loadTarget(const ros::NodeHandle& nh, const std::string& key, rct_image_tools::ModifiedCircleGridTarget& target);

bool loadIntrinsics(const ros::NodeHandle& nh, const std::string& key, rct_optimizations::CameraIntrinsics& intr);

}

#endif
