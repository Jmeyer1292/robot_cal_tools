#include "rct_optimizations/experimental/rail_intrinsic.h"
#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/eigen_conversions.h"
#include <ceres/ceres.h>

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


// Estimate the axis of motion as part of the optimization
class RailICal4
{
public:
  RailICal4(const Eigen::Vector2d image_obs, double rail_position, const Eigen::Vector3d& point)
    : obs_(image_obs)
    ,rail_position_(rail_position)
    , target_pt_(point)
  {}

  template<typename T>
  bool operator()(const T* const intrinsics, const T* const target_pose, const T* const rail_skew, T* residual) const
  {
    const T *target_aa(&target_pose[0]); // extract target's angle axis
    const T *target_tx(&target_pose[3]); // extract target's position

    // transform point into camera frame
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));

    T camera_point[3];  // point in camera coordinates
    rct_optimizations::transformPoint(target_aa, target_tx, target_pt, camera_point);

    // Now let's move the camera point by the rail travel
    // This involves two steps:

    // 1. Estimating the axis of motion (relative to camera z)
    T nominal_axis[3]; // Nominally we move back...
    nominal_axis[0] = T(0.0);
    nominal_axis[1] = T(0.0);
    nominal_axis[2] = T(1.0);

    T rotation_axis[3];
    rotation_axis[0] = rail_skew[0];
    rotation_axis[1] = rail_skew[1];
    rotation_axis[2] = T(0.0);

    T motion_axis[3];
    ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis, motion_axis);

    // 2. Moving the camera back by that distance
    camera_point[0] = camera_point[0] + T(rail_position_) * motion_axis[0];
    camera_point[1] = camera_point[1] + T(rail_position_) * motion_axis[1];
    camera_point[2] = camera_point[2] + T(rail_position_) * motion_axis[2];

    // compute project point into image plane and compute residual
    T xy_image[2];
    projectPoints2(intrinsics, camera_point, xy_image);
    residual[0] = xy_image[0] - obs_.x();
    residual[1] = xy_image[1] - obs_.y();

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector2d& image_obs, double rail_position, const Eigen::Vector3d& point)
  {
    return new ceres::AutoDiffCostFunction<RailICal4, 2, 9, 6, 2>(new RailICal4(image_obs, rail_position, point));
  }

  Eigen::Vector2d obs_;
  double rail_position_; /** location of camera along rail */
  Eigen::Vector3d target_pt_; /** point expressed in target coordinates */
};


}

rct_optimizations::RailIntrinsicResult rct_optimizations::optimize(const RailIntrinsicProblem& params)
{
  // Prepare data structure for the camera parameters to optimize
  std::array<double, CalibCameraIntrinsics<double>::size()> internal_intrinsics_data;
  for (std::size_t i = 0; i < 9; ++i) internal_intrinsics_data[i] = 0.0;

  MutableCalibCameraIntrinsics<double> internal_intrinsics (internal_intrinsics_data.data());
  internal_intrinsics.fx() = params.intrinsics_guess.fx();
  internal_intrinsics.fy() = params.intrinsics_guess.fy();
  internal_intrinsics.cx() = params.intrinsics_guess.cx();
  internal_intrinsics.cy() = params.intrinsics_guess.cy();

  // For each scene, prepare space for the initial target guess pose
  std::vector<Pose6d> internal_target_poses (params.extrinsic_guesses.size());
  for (std::size_t i = 0; i < internal_target_poses.size(); ++i)
  {
    internal_target_poses[i] = rct_optimizations::poseEigenToCal(params.extrinsic_guesses[i]);
  }

  // Allocate space for the skew parameters to be estimated
  std::array<double, 2> internal_skew_params;
  internal_skew_params[0] = params.skew_x_guess;
  internal_skew_params[1] = params.skew_y_guess;

  ceres::Problem problem;

  // For each scene
  for (std::size_t i = 0; i < params.image_observations.size(); ++i)
  {
    // For each "image" in the scene
    for (std::size_t j = 0; j < params.image_observations[i].size(); ++j)
    {
      // For each correspondence between target and image
      for (std::size_t k = 0; k < params.image_observations[i][j].size(); ++k)
      {
        const auto& point_in_target = params.image_observations[i][j][k].in_target;
        const auto& point_in_image = params.image_observations[i][j][k].in_image;
        const auto rail_position = params.rail_distances[i][j];
        auto* cost_block = RailICal4::Create(point_in_image, rail_position, point_in_target);
        problem.AddResidualBlock(cost_block, NULL,
                                 internal_intrinsics_data.data(),
                                 internal_target_poses[i].values.data(),
                                 internal_skew_params.data());
      }
    }
  }

  // Solve
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Package results
  RailIntrinsicResult result;
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

  result.skew_x = internal_skew_params[0];
  result.skew_y = internal_skew_params[1];

  for (const auto& pose : internal_target_poses)
  {
    result.target_transforms.push_back(poseCalToEigen(pose));
  }

  return result;
}
