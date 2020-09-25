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
 * @brief A correspondence sampler specifically for modified circle grid targets
 */
struct ModifiedCircleGridCorrespondenceSampler : CorrespondenceSampler
{
  ModifiedCircleGridCorrespondenceSampler(const std::size_t rows_, const std::size_t cols_)
    : rows(rows_)
    , cols(cols_)
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

    // For a modified circle target grid, the sample points should be the corners of the grid
    correspondence_indices.push_back(0); // upper left point
    correspondence_indices.push_back(rows - 1); // upper right point
    correspondence_indices.push_back(rows * cols - cols - 1); // lower left point
    correspondence_indices.push_back(rows * cols - 1); // lower right point

    return correspondence_indices;
  }

  std::size_t rows;
  std::size_t cols;
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
