#pragma once
#include <rct_optimizations/types.h>
#include <random>

namespace rct_optimizations
{

/**
 * @brief A simple structure for choosing which correspondence indices to use for generating the homography matrix
 */
struct CorrespondenceSampler
{
  virtual ~CorrespondenceSampler() = default;
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

struct RandomCorrespondenceSampler : CorrespondenceSampler
{
  RandomCorrespondenceSampler(const std::size_t n_correspondences_, const std::size_t n_samples_)
    : n_correspondences(n_correspondences_)
    , n_samples(n_samples_)
  {
    assert(n_samples_ >= 4);
    assert(n_samples_ < n_correspondences_);
  }

  virtual std::vector<std::size_t> getSampleCorrespondenceIndices() const final override
  {
    // Create a random number generator with a uniform distribution across all indices
    std::mt19937 rand_gen(std::random_device{}());
    std::uniform_int_distribution<std::size_t> dist(0, n_correspondences - 1);
    auto fn = [&rand_gen, &dist]() -> std::size_t { return dist(rand_gen); };

    // Generate a vector of 4 random correspondence indices
    std::vector<std::size_t> output(n_samples);
    std::generate(output.begin(), output.end(), fn);

    return output;
  }

  /** @brief Number of total correspondences */
  const std::size_t n_correspondences;
  /** @brief Number of samples with which to calculate the homography transform. This number must be at least 4.
   * Typically a lower number of samples (i.e. 4) does not produce an accurate homography transform; one quarter to half the total number of correspondences is a good rule of thumb */
  const std::size_t n_samples;
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
