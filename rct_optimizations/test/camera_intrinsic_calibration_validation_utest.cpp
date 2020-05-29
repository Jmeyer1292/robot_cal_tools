#include <gtest/gtest.h>
#include <rct_optimizations/validation/camera_intrinsic_calibration_validation.h>
#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>

using namespace rct_optimizations;

TEST(CameraIntrinsicCalibrationValidation, GetInternalTargetTransformationTest)
{
  test::Target target(8, 6, 0.025);
  test::Camera camera = test::makeKinectCamera();

  // Create a camera pose directly above the target
  Eigen::Isometry3d base_to_target(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d base_to_camera = base_to_target * Eigen::Translation3d(0.0, 0.0, 1.0)
                                     * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  Correspondence2D3D::Set corr_set;
  ASSERT_NO_THROW(
    corr_set = test::getCorrespondences(base_to_camera, base_to_target, camera, target, true));

  Eigen::Isometry3d camera_to_target = base_to_camera.inverse() * base_to_target;

  /* Since `getInternalTargetTransformation` simply splits the correspondences in half,
     * creating two virtual targets with different features relative to the same origin,
     * the expected transformation from one virtual target to the other should be identity */
  const Eigen::Isometry3d expected(Eigen::Isometry3d::Identity());

  // Perfect camera intrinsics, perfect camera to target guess
  {
    Eigen::Isometry3d transform;
    EXPECT_NO_THROW(
      transform = getInternalTargetTransformation(corr_set, camera.intr, camera_to_target));

    // Expect the transform to be almost exactly as expected
    EXPECT_TRUE(transform.isApprox(expected));
  }

  // Create a slightly noisy camera to target guess (1 cm positional and ~5 degrees orientation noise)
  Eigen::Isometry3d camera_to_target_guess = test::perturbPose(camera_to_target, 0.01, 0.01);

  // Perfect camera intrinsics, imperfect camera to target guess
  {
    Eigen::Isometry3d transform;
    EXPECT_NO_THROW(
      transform = getInternalTargetTransformation(corr_set, camera.intr, camera_to_target_guess));

    // Expect the transform to be very close to its expected value, but with a small error from the PnP
    EXPECT_TRUE(transform.isApprox(expected, 1.0e-8));
  }

  // Bad camera intrinsics, imperfect camera to target guess
  {
    // Tweak the camera intrinsics slightly (+/- 1%)
    camera.intr.fx() += 0.01 * camera.intr.fx();
    camera.intr.fy() -= 0.01 * camera.intr.fy();
    camera.intr.cx() -= 0.01 * camera.intr.cx();
    camera.intr.cy() += 0.01 * camera.intr.cy();

    Eigen::Isometry3d transform;
    EXPECT_NO_THROW(
      transform = getInternalTargetTransformation(corr_set, camera.intr, camera_to_target_guess));

    // Expect the transform to be very close to its expected value, but with a small error from the PnP
    EXPECT_FALSE(transform.isApprox(expected, 1.0e-8));
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
