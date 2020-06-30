#include <gtest/gtest.h>
#include <random>
#include <rct_optimizations/validation/homography_check.h>
#include <rct_optimizations_tests/observation_creator.h>
#include <rct_optimizations_tests/utilities.h>

using namespace rct_optimizations;

const unsigned TARGET_ROWS = 5;
const unsigned TARGET_COLS = 7;
const double SPACING = 0.025;

class HomographyTest : public ::testing::Test
{
  public:
  HomographyTest()
    : camera(test::makeKinectCamera())
    , target(TARGET_ROWS, TARGET_COLS, SPACING)
    , target_to_camera(Eigen::Isometry3d::Identity())
    , sampler(TARGET_ROWS, TARGET_COLS)
  {
    double x = static_cast<double>(TARGET_ROWS - 1) * SPACING / 2.0;
    double y = static_cast<double>(TARGET_COLS - 1) * SPACING / 2.0;
    target_to_camera.translate(Eigen::Vector3d(x, y, 0.5));
    target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  }

  test::Camera camera;
  test::Target target;
  Eigen::Isometry3d target_to_camera;
  ModifiedCircleGridCorrespondenceSampler sampler;
};

TEST_F(HomographyTest, PerfectInitialConditions)
{
  rct_optimizations::Correspondence2D3D::Set correspondence_set;
  EXPECT_NO_THROW(correspondence_set = test::getCorrespondences(target_to_camera,
                                                                Eigen::Isometry3d::Identity(),
                                                                camera,
                                                                target,
                                                                true));

  Eigen::VectorXd error = calculateHomographyError(correspondence_set, sampler);
  // Expect all of the errors to be extremely small
  EXPECT_LT(error.maxCoeff(), 1.0e-12);
}

TEST_F(HomographyTest, SwapCorrespondencesConditions)
{
  rct_optimizations::Correspondence2D3D::Set correspondence_set;
  EXPECT_NO_THROW(correspondence_set = test::getCorrespondences(target_to_camera,
                                                                Eigen::Isometry3d::Identity(),
                                                                camera,
                                                                target,
                                                                true));




  // Swap the image measurements between 2 arbitrary correspondences
  Correspondence2D3D &c1 = correspondence_set.at(10);
  Eigen::Vector2d in_image_1 = c1.in_image;
  Correspondence2D3D &c2 = correspondence_set.at(21);
  Eigen::Vector2d in_image_2 = c2.in_image;
  c1.in_image = in_image_2;
  c2.in_image = in_image_1;

  Eigen::VectorXd error = calculateHomographyError(correspondence_set, sampler);

  // Expect the error of the points used to create the homography matrix to be very small
  std::vector<std::size_t> sample_indices = sampler.getSampleCorrespondenceIndices();
  for (std::size_t idx : sample_indices)
  {
    EXPECT_LT(error(idx), 1.0e-12);
  }

  // Expect the homography to be off by more than one pixel because of the swap
  EXPECT_GT(error.maxCoeff(), 1.0);
}

TEST_F(HomographyTest, NoisyCorrespondences)
{
  rct_optimizations::Correspondence2D3D::Set correspondence_set;
  EXPECT_NO_THROW(correspondence_set = test::getCorrespondences(target_to_camera,
                                                                Eigen::Isometry3d::Identity(),
                                                                camera,
                                                                target,
                                                                true));

  // Add Gaussian noise to the image features
  std::mt19937 mt_rand(std::random_device{}());
  const double stdev = 1.0;
  std::normal_distribution<double> dist(0.0, stdev);
  for (Correspondence2D3D &corr : correspondence_set)
  {
    Eigen::Vector2d noise;
    noise << dist(mt_rand), dist(mt_rand);
    corr.in_image += noise;
  }

  Eigen::VectorXd error = calculateHomographyError(correspondence_set, sampler);

  // TODO: create an informed expectation for the max or average error. Initial testing indicates that the average error can occasionally be > 4 * stddev
  double stdev_mag = stdev * std::sqrt(2.0);
  EXPECT_GT(error.maxCoeff(), stdev_mag);
  EXPECT_GT(error.mean(), stdev_mag);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
