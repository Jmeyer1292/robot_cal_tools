#pragma once

#include <Eigen/Core>
#include <map>
#include <vector>

namespace rct_image_tools
{
/** @brief Typedef for an STL vector of Eigen vector objects. Use a specialized allocator in the case that types
 * divisible by 16 bytes are used, specifically Eigen::Vector2d */
template <Eigen::Index DIM>
using VectorEigenVector =
    std::vector<Eigen::Matrix<double, DIM, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1>>>;

/** Typedef for an STL map with an unsigned integer key and STL vector of Eigen vector value */
template <Eigen::Index DIM>
using MapVectorEigenVector = std::map<unsigned, VectorEigenVector<DIM>>;

/** @brief Typedef for a container of target features from a calibration target.
 *  This definition allows for multiple features to be associated with a single unique identifier
 *  (such as the corners of an ArUco tag)
 */
using TargetFeatures = MapVectorEigenVector<2>;

} // namespace rct_image_tools
