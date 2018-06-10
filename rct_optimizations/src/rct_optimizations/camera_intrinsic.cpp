#include "rct_optimizations/experimental/camera_intrinsic.h"

#include <rct_optimizations/ceres_math_utilities.h>
#include <ceres/ceres.h>
#include <rct_optimizations/eigen_conversions.h>

namespace
{

template <typename T>
struct CalibCameraIntrinsics
{
  const T* data;

  CalibCameraIntrinsics(const T* data) : data(data) {}

  const T& fx() const { return data[0]; }
  const T& fy() const { return data[1]; }
  const T& cx() const { return data[2]; }
  const T& cy() const { return data[3]; }

  const T& k1() const { return data[4]; }
  const T& k2() const { return data[5]; }
  const T& p1() const { return data[6]; }
  const T& p2() const { return data[7]; }
  const T& k3() const { return data[8]; }

  constexpr static std::size_t size(){ return 9; }
};

template <typename T>
struct MutableCalibCameraIntrinsics
{
  T* data;

  MutableCalibCameraIntrinsics(T* data) : data(data) {}

  const T& fx() const { return data[0]; }
  const T& fy() const { return data[1]; }
  const T& cx() const { return data[2]; }
  const T& cy() const { return data[3]; }

  const T& k1() const { return data[4]; }
  const T& k2() const { return data[5]; }
  const T& p1() const { return data[6]; }
  const T& p2() const { return data[7]; }
  const T& k3() const { return data[8]; }

   T& fx()  { return data[0]; }
   T& fy()  { return data[1]; }
   T& cx()  { return data[2]; }
   T& cy()  { return data[3]; }

   T& k1()  { return data[4]; }
   T& k2()  { return data[5]; }
   T& p1()  { return data[6]; }
   T& p2()  { return data[7]; }
   T& k3()  { return data[8]; }

  constexpr static std::size_t size(){ return 9; }
};

template <typename T>
void projectPoints2(const T* const camera_intr, const T* const pt_in_camera, T* pt_in_image)
{
  T xp1 = pt_in_camera[0];
  T yp1 = pt_in_camera[1];
  T zp1 = pt_in_camera[2];

  CalibCameraIntrinsics<T> intr (camera_intr);

  // Scale into the image plane by distance away from camera
  T xp;
  T yp;
  if (zp1 == T(0)) // Avoid dividing by zero.
  {
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  // Temporary variables for distortion model.
  T xp2 = xp * xp;    // x^2
  T yp2 = yp * yp;    // y^2
  T r2  = xp2 + yp2;  // r^2 radius squared
  T r4  = r2 * r2;    // r^4
  T r6  = r2 * r4;    // r^6

  // Apply the distortion coefficients to refine pixel location
  T xpp = xp
    + intr.k1() * r2 * xp    // 2nd order term
    + intr.k2() * r4 * xp    // 4th order term
    + intr.k3() * r6 * xp    // 6th order term
    + intr.p2() * (r2 + T(2.0) * xp2) // tangential
    + intr.p1() * xp * yp * T(2.0); // other tangential term

  T ypp = yp
    + intr.k1() * r2 * yp    // 2nd order term
    + intr.k2() * r4 * yp    // 4th order term
    + intr.k3() * r6 * yp    // 6th order term
    + intr.p1() * (r2 + T(2.0) * yp2) // tangential term
    + intr.p2() * xp * yp * T(2.0); // other tangential term

  // Perform projection using focal length and camera center into image plane
  pt_in_image[0] = intr.fx() * xpp + intr.cx();
  pt_in_image[1] = intr.fy() * ypp + intr.cy();
}


class IntrinsicCostFunction
{
public:
  IntrinsicCostFunction(const Eigen::Vector3d& in_target, const Eigen::Vector2d& in_image)
    : in_target_(in_target), in_image_(in_image)
  {}

  template<typename T>
  bool operator()(const T* const target_pose, const T* const camera_intr, T* const residual) const
  {
    const T* target_angle_axis = target_pose + 0;
    const T* target_position = target_pose + 3;

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(in_target_(0));
    target_pt[1] = T(in_target_(1));
    target_pt[2] = T(in_target_(2));

    T camera_point[3];  // Point in camera coordinates
    rct_optimizations::transformPoint(target_angle_axis, target_position, target_pt, camera_point);

    T xy_image[2];
    projectPoints2(camera_intr, camera_point, xy_image);

    residual[0] = xy_image[0] - in_image_.x();
    residual[1] = xy_image[1] - in_image_.y();

    return true;
  }

private:
  Eigen::Vector3d in_target_;
  Eigen::Vector2d in_image_;
};

}

static rct_optimizations::Pose6d guessInitialPose()
{
  Eigen::Affine3d guess = Eigen::Affine3d::Identity();
  guess = guess * Eigen::Translation3d(0,0,0.1) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  return rct_optimizations::poseEigenToCal(guess);
}

rct_optimizations::IntrinsicEstimationResult
rct_optimizations::optimize(const rct_optimizations::IntrinsicEstimationProblem& params)
{
  // Prepare data structure for the camera parameters to optimize
  std::array<double, CalibCameraIntrinsics<double>::size()> internal_intrinsics_data = {0.0};
  MutableCalibCameraIntrinsics<double> internal_intrinsics (internal_intrinsics_data.data());
  internal_intrinsics.fx() = params.intrinsics_guess.fx();
  internal_intrinsics.fy() = params.intrinsics_guess.fy();
  internal_intrinsics.cx() = params.intrinsics_guess.cx();
  internal_intrinsics.cy() = params.intrinsics_guess.cy();

  // Prepare space for the target poses to estimate (1 for each observation set)
  std::vector<Pose6d> internal_poses;
  internal_poses.reserve(params.image_observations.size());

  // All of the target poses are seeded to be "in front of" and "looking at" the camera
  for (std::size_t i = 0; i < params.image_observations.size(); ++i)
    internal_poses.push_back(guessInitialPose());

  ceres::Problem problem;

  // Create a set of cost functions for each observation set
  for (std::size_t i = 0; i < params.image_observations.size(); ++i)
  {
    // Create a cost for each 2D -> 3D image correspondence
    for (std::size_t j = 0; j < params.image_observations[i].size(); ++j)
    {
      const auto& point_in_target = params.image_observations[i][j].in_target;
      const auto& point_in_image = params.image_observations[i][j].in_image;

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new IntrinsicCostFunction(point_in_target, point_in_image);

      auto* cost_block = new ceres::AutoDiffCostFunction<IntrinsicCostFunction, 2, 6, 9>(cost_fn);

      problem.AddResidualBlock(cost_block, NULL, internal_poses[i].values.data(),
                               internal_intrinsics_data.data());
    }
  }

  // Solve
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Package results
  IntrinsicEstimationResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;

  result.intrinsics.fx() = internal_intrinsics.fx();
  result.intrinsics.fy() = internal_intrinsics.fy();
  result.intrinsics.cx() = internal_intrinsics.cx();
  result.intrinsics.cy() = internal_intrinsics.cy();

  result.distortions[0] = internal_intrinsics_data[4];
  result.distortions[1] = internal_intrinsics_data[5];
  result.distortions[2] = internal_intrinsics_data[6];
  result.distortions[3] = internal_intrinsics_data[7];
  result.distortions[4] = internal_intrinsics_data[8];

  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  return result;
}
