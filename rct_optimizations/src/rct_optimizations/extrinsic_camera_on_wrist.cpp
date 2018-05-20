#include "rct_optimizations/extrinsic_camera_on_wrist.h"
#include "rct_optimizations/types.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <iostream>

using namespace rct_optimizations;

namespace
{

template<typename T> inline void transformPoint3D(const T angle_axis[3],
  const T tx[3], const std::vector<double> &point, T t_point[3])
{
  T point_[3];

  point_[0] = T(point[0]);
  point_[1] = T(point[1]);
  point_[2] = T(point[2]);

  ceres::AngleAxisRotatePoint(angle_axis, point_, t_point);

  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

template<typename T>
inline void poseTransformPoint(const Pose6d &pose, const T point[3], T t_point[3])
{
  T angle_axis[3];

  angle_axis[0]  = T(pose.rx());
  angle_axis[1]  = T(pose.ry());
  angle_axis[2]  = T(pose.rz());

  ceres::AngleAxisRotatePoint(angle_axis, point, t_point);

  t_point[0] = t_point[0] + T(pose.x());
  t_point[1] = t_point[1] + T(pose.y());
  t_point[2] = t_point[2] + T(pose.z());
}

template<typename T>
inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3])
{
  ceres::AngleAxisRotatePoint(angle_axis, point, t_point);

  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

template<typename T>
inline void cameraPointResidual(T point[3], T &fx, T &fy, T &cx, T &cy, T &ox, T &oy, T residual[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  // Scale into the image plane by distance away from camera
  T xp, yp;

  if (zp1 == T(0)) // Avoid divide by zero
  {
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  // Perform projection using focal length and camera optical center into image plane
  residual[0] = fx * xp + cx - ox;
  residual[1] = fy * yp + cy - oy;
}

template <typename T>
inline void projectPoint(const CameraIntrinsics& intr, const T point[3], T xy_image[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  // Scale into the image plane by distance away from camera
  T xp, yp;

  if (zp1 == T(0)) // Avoid divide by zero
  {
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  // Perform projection using focal length and camera optical center into image plane
  xy_image[0] = intr.fx() * xp + intr.cx();
  xy_image[1] = intr.fy() * yp + intr.cy();
}

class ReprojectionCost
{
public:
  /**
   * @brief A CERES cost function class that represents a single observation. Each observation is:
   *  - One point on the calibration target as seen by the camera with the given intrinsics
   *  - Associated with a given wrist position (which may have produced many such calibrations)
   * @param obs
   * @param intr
   * @param wrist_pose
   * @param point_in_target
   */
  ReprojectionCost(const Observation2d& obs, const CameraIntrinsics& intr, const Pose6d& wrist_to_base,
                   const Point3d& point_in_target)
    : obs_(obs), intr_(intr), wrist_pose_(wrist_to_base), target_pt_(point_in_target)
  {}

  template <typename T>
  bool operator() (const T* pose_camera_to_wrist, const T* pose_base_to_target, T* residual) const
  {
    const T* camera_angle_axis = pose_camera_to_wrist+ 0;
    const T* camera_position = pose_camera_to_wrist+ 3;

    const T* target_angle_axis = pose_base_to_target + 0;
    const T* target_position = pose_base_to_target + 3;

    T world_point[3]; // Point in world coordinates
    T link_point[3]; // Point in link coordinates
    T camera_point[3]; // Point in camera coordinates


    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_.values[0]);
    target_pt[1] = T(target_pt_.values[1]);
    target_pt[2] = T(target_pt_.values[2]);
    transformPoint(target_angle_axis, target_position, target_pt, world_point);

    poseTransformPoint(wrist_pose_, world_point, link_point);

    transformPoint(camera_angle_axis, camera_position, link_point, camera_point);
//    std::cout << "world_point" << world_point[0] << " " << world_point[1] << " " << world_point[2] << "\n";
//    std::cout << "in_camera " << camera_point[0] << " " << camera_point[1] << " " << camera_point[2] << "\n";

    T xy_image [2];
    projectPoint(intr_, camera_point, xy_image);

//    std::cout << "xy " << xy_image[0] << " " << xy_image[1] << "\n";

    residual[0] = xy_image[0] - obs_.x();
    residual[1] = xy_image[1] - obs_.y();

    return true;
  }

private:
  Observation2d obs_;
  CameraIntrinsics intr_;
  Pose6d wrist_pose_;
  Point3d target_pt_;
};

}

Eigen::Matrix3d getBasis(const Pose6d& p)
{
  Eigen::Matrix3d R;
  double angle = sqrt(p.rx() * p.rx() + p.ry()*p.ry() + p.rz()*p.rz());

  if (angle < .0001)
  {
    R(0,0) = 1.0;  R(0,1) = 0.0;  R(0,2) = 0.0;
    R(1,0) = 0.0;  R(1,1) = 1.0;  R(1,2) = 0.0;
    R(2,0) = 0.0;  R(2,1) = 0.0;  R(2,2) = 1.0;

    return R;
  }

  double cos_theta = cos(angle);
  double sin_theta = sin(angle);

  double wx = p.rx()/angle;
  double wy = p.ry()/angle;
  double wz = p.rz()/angle;
  double omct = 1.0 - cos_theta;

  R(0,0) = cos_theta + wx*wx*omct;
  R(0,1) = wx*wy*omct - wz*sin_theta;
  R(0,2) = wx*wz*omct + wy*sin_theta;
  R(1,0) = wy*wx*omct + wz*sin_theta;
  R(1,1) = cos_theta + wy*wy*omct;
  R(1,2) = wy*wz*omct - wx*sin_theta;
  R(2,0) = wz*wx*omct - wy*sin_theta;
  R(2,1) = wz*wy*omct + wx*sin_theta;
  R(2,2) = cos_theta + wz*wz*omct;

  return R;
}

Pose6d getInverse(const Pose6d& in)
{
  double newx,newy,newz;
  Eigen::Matrix3d R = getBasis(in);
  newx = -(R(0,0) * in.x() + R(1,0) * in.y() + R(2,0) * in.z());
  newy = -(R(0,1) * in.x() + R(1,1) * in.y() + R(2,1) * in.z());
  newz = -(R(0,2) * in.x() + R(1,2) * in.y() + R(2,2) * in.z());

  Pose6d new_pose ({-in.rx(), -in.ry(), -in.rz(), newx, newy, newz});

  return new_pose;
}

Pose6d inverse(const Pose6d& in) { return getInverse(in); }

rct_optimizations::ExtrinsicCameraOnWristResult rct_optimizations::optimize(const ExtrinsicCameraOnWristParameters& params)
{
  assert(params.image_observations.size() == params.wrist_poses.size());

  Pose6d internal_base_to_target = params.base_to_target_guess;
  Pose6d internal_camera_to_wrist = inverse(params.wrist_to_camera_guess);

  ceres::Problem problem;

  for (std::size_t i = 0; i < params.wrist_poses.size(); ++i) // For each wrist pose / image set
  {
    for (std::size_t j = 0; j < params.image_observations[i].size(); ++j) // For each 3D point seen in the 2D image
    {
      // Define
      const auto& img_obs = params.image_observations[i][j].in_image;
      const auto& point_in_target = params.image_observations[i][j].in_target;
      const auto wrist_to_base = inverse(params.wrist_poses[i]);

      // Allocate Ceres data structures
      auto* cost_fn = new ReprojectionCost(img_obs, params.intr, wrist_to_base, point_in_target);

      auto* cost_block = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 6, 6>(cost_fn);

      problem.AddResidualBlock(cost_block, NULL, internal_camera_to_wrist.values.data(),
                               internal_base_to_target.values.data());
    }
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";

  ExtrinsicCameraOnWristResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.base_to_target = internal_base_to_target;
  result.wrist_to_camera = inverse(internal_camera_to_wrist);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  return result;
}

//    Pose6<T> target_pose (pose_base_to_target);
//    Point3<T> world_point = target_pose * point_in_target.cast<T>();

//    Point3<T> link_point = wrist_pose_.cast<T>() * world_point;

//    Pose6<T> camera_pose (pose_wrist_to_camera);
//    Point3<T> camera_point = camera_pose * link_point;

//    Point2<T> image_space = intr_.project(camera_point);

//    residual[0] = image_space.x - obs_.x();
//    residual[1] = image_space.y - obs_.y();


