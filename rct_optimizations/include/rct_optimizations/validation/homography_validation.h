#pragma once
#include <rct_optimizations/types.h>

namespace rct_optimizations
{

/**
 * @brief A simple structure for choosing which correspondence indices to use for generating the homography matrix
 */
struct CorrespondenceSampler
{
  virtual std::vector<std::size_t> getSampleCorrespondenceIndices() const = 0;
};

/**
 * @brief A correspondence sampler specifically for grid targets
 */
struct GridCorrespondenceSampler : CorrespondenceSampler
{
  GridCorrespondenceSampler(const std::size_t rows_, const std::size_t cols_, const std::size_t stride_ = 1)
    : rows(rows_)
    , cols(cols_)
    , stride(stride_)
  {
  }

  virtual std::vector<std::size_t> getSampleCorrespondenceIndices() const final override
  {
    const std::size_t n_samples = 4;

    // Make sure there are at least two times as many points as the number of sample points
    if ((rows * cols / 2) < n_samples)
    {
      std::stringstream ss;
      ss << "Number of correspondences does not exceed minimum of " << n_samples * 2 << " (" << rows * cols << " provided)";
      throw std::runtime_error(ss.str());
    }

    std::vector<std::size_t> correspondence_indices;
    correspondence_indices.reserve(n_samples);

    // Sample points should be the corners of the grid, using the first element in the stride
    std::size_t upper_left_idx = 0;
    std::size_t upper_right_idx = (cols - 1) * stride;
    std::size_t lower_left_idx = (rows - 1) * (cols * stride);
    std::size_t lower_right_idx = (rows * cols * stride) - stride;

    correspondence_indices.push_back(upper_left_idx);
    correspondence_indices.push_back(upper_right_idx);
    correspondence_indices.push_back(lower_left_idx);
    correspondence_indices.push_back(lower_right_idx);

    return correspondence_indices;
  }

  /** @brief Number of rows in the target */
  const std::size_t rows;
  /** @brief Number of columns in the target */
  const std::size_t cols;
  /**
   * @brief Number elements associated with each grid point (i.e. depth)
   * ArUco grid targets, for example, have 4 observations associated with each element in the grid, so the stride would be 4
   */
  const std::size_t stride;
};

/**
 * @brief Computes the error between a set of corresponding planar data points (points on a planar target and their corresponding locations in the image plane)
 * This function creates a homography matrix which transforms data from one plane onto another plane (i.e. from the target plane to the image plane).
 * This matrix is created using only a subset of the correspondences. The matrix is then used to estimate the location of all target points in the image plane.
 * The error between these estimates and the actual values is a good measurement of how accurately a set of 2D camera measurements represent a calbration target with known geometry
 *
 * Assumptions:
 *  - Both sets of points lie on a plane (i.e. points on a planar calibration target and points on the image plane)
 *
 * @param correspondences - A set of corresponding points
 * @param correspondence_sampler - a struct for choosing correspondence indices to generate the homography matrix
 * @return A vector of homography errors for each correspondence
 * @throws Exception when number of correspondences does not exceed 2x the number of samples
 */
Eigen::VectorXd calculateHomographyError(const Correspondence2D3D::Set &correspondences,
                                         const CorrespondenceSampler &correspondence_sampler);

/**
 * @brief Calculates the homography error for correspondences of 3D planar points using @ref calculateHomographyError
 * @param correspondences - A set of corresponding points
 * @param correspondence_sampler - a struct for choosing correspondence indices to generate the homography matrix
 * @return A vector of homography errors for each correspondence
 */
Eigen::VectorXd calculateHomographyError(const Correspondence3D3D::Set &correspondences,
                                         const CorrespondenceSampler &correspondence_sampler);


} //rct_optimizations
