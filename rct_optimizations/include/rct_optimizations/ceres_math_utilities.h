#ifndef RCT_CERES_MATH_UTILITIES_H
#define RCT_CERES_MATH_UTILITIES_H

#include <ceres/rotation.h>
#include <rct_optimizations/types.h>

namespace rct_optimizations
{
template <typename T>
inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3])
{
  ceres::AngleAxisRotatePoint(angle_axis, point, t_point);

  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

template <typename T>
inline void poseTransformPoint(const Pose6d& pose, const T point[3], T t_point[3])
{
  T angle_axis[3];

  angle_axis[0] = T(pose.rx());
  angle_axis[1] = T(pose.ry());
  angle_axis[2] = T(pose.rz());

  T translation[3];
  translation[0] = T(pose.x());
  translation[1] = T(pose.y());
  translation[2] = T(pose.z());

  transformPoint(angle_axis, translation, point, t_point);
}

template <typename T>
inline void projectPoint(const CameraIntrinsics& intr, const T point[3], T xy_image[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  // Scale into the image plane by distance away from camera
  T xp, yp;

  if (zp1 == T(0))  // Avoid divide by zero
  {
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  // Perform projection using focal length and camera optical center into image
  // plane
  xy_image[0] = intr.fx() * xp + intr.cx();
  xy_image[1] = intr.fy() * yp + intr.cy();
}

template <typename T>
inline Eigen::Matrix<T, 2, 1> projectPoint(const rct_optimizations::CameraIntrinsics& intr,
                                           const Eigen::Matrix<T, 3, 1>& point)
{
  // Scale the input point by its distance from the camera (i.e. z-coordinate)
  Eigen::Matrix<T, 3, 1> scaled_point(point);
  scaled_point(2) = T(1.0);

  // Avoid divide by zero
  if (std::abs(point.z() - T(0.0) > T(1.e-10)))
  {
    scaled_point.x() = point.x() / point.z();
    scaled_point.y() = point.y() / point.z();
  }

  /* Create the camera parameters matrix
   * | fx  0   cx | * | x/z | = | u |
   * | 0   fy  cy | * | y/z | = | v |
   * | 0   0   1  | * |  1  | = | 1 |
   */
  Eigen::Matrix3d camera_matrix;
  camera_matrix << intr.fx(), 0.0, intr.cx(), 0.0, intr.fy(), intr.cy(), 0.0, 0.0, 1.0;

  // Perform projection using focal length and camera optical center into image plane
  Eigen::Matrix<T, 3, 1> image_pt = camera_matrix.cast<T>() * scaled_point;

  // Return only the top two elements of the vector
  return image_pt.template head<2>();
}

}  // namespace rct_optimizations

#endif  // RCT_CERES_MATH_UTILITIES_H
