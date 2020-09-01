#include <rct_optimizations/validation/homography_validation.h>
#include <Eigen/Dense>
#include <rct_optimizations/types.h>

namespace rct_optimizations
{
Eigen::VectorXd calculateHomographyError(const Correspondence2D3D::Set &correspondences,
                                         const CorrespondenceSampler &correspondence_sampler)
{
  /* There is a 3x3 homography matrix, H, that can transform a point from one plane onto a different plane
   * | u | = k * | H00 H01 H02 | * | x |
   * | v |       | H10 H11 H12 |   | y |
   * | 1 |       | H20 H12  1  |   | 1 |
   *
   * In our case we have 2 sets of known corresponding planar points: points on the planar target, and points in the image plane
   * Therefore, there is some matrix, H, which can transform target points into the image plane.
   * If the target points and camera points actually match, we should be able to:
   *   1. Calculate H for a subset of corresponding points
   *   2. Transform the remaining target points by H to obtain estimates of their locations in the image plane
   *   3. Compare the calculated estimations to the actual image points to make sure they are very close. If they are not close, we know that the correspondences are not valid
   *
   * The matrix H has 8 unique values.
   * These 8 values of the homography matrix can be solved for, given a set of (at least) 8 corresponding planar vectors, by rearranging the above equations:
   *
   * A * H = b, where
   * H = inv(A) * b
   *   - A is matrix (size 2*n x 8), where n is the number of corresponding vectors
   *   - H is a vector (size 8 x 1) of the unknown elements of the homography matrix
   *   - B is a vector (size 2*n x 1) representing the elements of one set of planar vectors
   *
   *                  A               *              H    =    b
   * |-x0 -y0  -1   0    0    0   u0*x0   u0*y0 | * | H00 | = | -u0 |
   * | 0   0   0  -x0   -y0  -1   v0*x0   v0*y0 | * | H01 | = | -v0 |
   *                              ...
   * |-x7 -y7  -1   0    0    0   u7*x7   u7*y7 | * | H20 | = | -u7 |
   * | 0   0   0  -x7   -y7  -1   v7*x7   v7*y7 | * | H21 | = | -v7 |
   *
   */

  // Select the points that we want to use to create the H matrix
  std::vector<std::size_t> sample_correspondence_indices = correspondence_sampler
                                                             .getSampleCorrespondenceIndices();
  std::size_t n_samples = sample_correspondence_indices.size();

  // Ensure that there are enough points for testing outside of the sampled set
  if (correspondences.size() < 2 * n_samples)
  {
    std::stringstream ss;
    ss << "Correspondences size is not more than 2x sample size (" << correspondences.size()
       << " correspondences vs. " << n_samples << ")";
    throw std::runtime_error(ss.str());
  }

  // Create the A and b matrices
  Eigen::MatrixXd A(2 * n_samples, 8);
  Eigen::MatrixXd b(2 * n_samples, 1);

  // Fill the A and B matrices with data from the selected correspondences
  for (std::size_t i = 0; i < n_samples; ++i)
  {
    std::size_t corr_idx = sample_correspondence_indices.at(i);
    const Correspondence2D3D &corr = correspondences.at(corr_idx);

    //assign A row-th row:
    const double x = corr.in_target.x();
    const double y = corr.in_target.y();
    const double u = corr.in_image.x();
    const double v = corr.in_image.y();
    A.row(2 * i) << -x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y;
    A.row(2 * i + 1) << 0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y;

    b.block<2, 1>(2 * i, 0) = -1.0 * corr.in_image;
  }

  // Create the homography matrix
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> H = Eigen::Matrix3d::Ones();

  // Map the elements of the H matrix into a column vector and solve for the first 8
  {
    Eigen::Map<Eigen::VectorXd> hv(H.data(), 9);
    hv.head<8>() = A.fullPivLu().solve(b);
  }

  // Estimate the image locations of all the target observations and compare to the actual image locations
  Eigen::VectorXd error(correspondences.size());
  for (std::size_t i = 0; i < correspondences.size(); ++i)
  {
    const Correspondence2D3D &corr = correspondences[i];

    // Calculate the scaling factor
    double ki = 1.0 / (H(2, 0) * corr.in_target.x() + H(2, 1) * corr.in_target.y() + 1.0);

    // Replace the z-element of the point with 1
    Eigen::Vector3d xy(corr.in_target);
    xy(2) = 1.0;

    // Estimate the point in the image plane
    Eigen::Vector3d in_image_estimate = ki * H * xy;

    // Calculate the error
    Eigen::Vector2d image_error = corr.in_image - in_image_estimate.head<2>();
    error(i) = image_error.norm();
  }

  return error;
}

Eigen::VectorXd calculateHomographyError(const Correspondence3D3D::Set& correspondences,
                                         const CorrespondenceSampler& correspondence_sampler)
{
  // Convert the 3D correspondence points into 2D points by scaling the x and y components by the z component
  Correspondence2D3D::Set correspondences_2d;
  correspondences_2d.reserve(correspondences.size());

  // Store the z values for scaling later
  Eigen::VectorXd z_values(correspondences.size());

  for (std::size_t i = 0; i < correspondences.size(); ++i)
  {
    const auto& corr = correspondences[i];

    // Store the z value of the correspondence
    z_values[i] = corr.in_image.z();

    // Generate a scaled 2D correspondence
    Correspondence2D3D corr_2d;
    corr_2d.in_target = corr.in_target;
    corr_2d.in_image = (corr.in_image / corr.in_image.z()).head<2>();
    correspondences_2d.push_back(corr_2d);
  }

  // Calculate the homography error
  Eigen::VectorXd error = calculateHomographyError(correspondences_2d, correspondence_sampler);

  // Scale the errors again by the z values of the original correspondences
  return error.cwiseProduct(z_values);
}

} // namespace rct_optimizations

