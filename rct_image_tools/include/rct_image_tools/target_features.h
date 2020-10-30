#pragma once

#include <Eigen/Core>
#include <map>
#include <vector>

namespace rct_image_tools
{
/** @brief Typedef for a container of target features from a calibration target.
 *  This definition allows for multiple features to be associated with a single unique identifier
 *  (such as the corners of an ArUco tag)
 */
using TargetFeatures = std::map<unsigned, std::vector<Eigen::Vector2d>>;

} // namespace rct_image_tools
