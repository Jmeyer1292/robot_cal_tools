#ifndef RCT_CERES_MATH_UTILITIES_H
#define RCT_CERES_MATH_UTILITIES_H

#include <ceres/rotation.h>
#include <rct_optimizations/types.h>

namespace rct_optimizations
{

template <typename T>
inline void transformPose6d(const T angle_axis1[3], const T tx1[3], const T angle_axis2[3], const T tx2[3], T t_angle_axis[3], T t_tx[3])
{
  T rotation_matrix[9];
  ceres::AngleAxisToRotationMatrix(angle_axis2, rotation_matrix);

  T t_rotation_matrix[9];
  ceres::AngleAxisRotatePoint(angle_axis1, rotation_matrix, t_rotation_matrix);
  ceres::AngleAxisRotatePoint(angle_axis1, rotation_matrix + 3, t_rotation_matrix + 3);
  ceres::AngleAxisRotatePoint(angle_axis1, rotation_matrix + 6, t_rotation_matrix + 6);

  ceres::RotationMatrixToAngleAxis(t_rotation_matrix, t_angle_axis);

  ceres::AngleAxisRotatePoint(angle_axis1, tx2, t_tx);

  t_tx[0] += tx1[0];
  t_tx[1] += tx1[1];
  t_tx[2] += tx1[2];
}

template <typename T>
inline void transformPose6d(const Pose6d& pose, const T angle_axis[3], const T tx[3], T t_angle_axis[3], T t_tx[3])
{
  T pose_angle_axis[3];

  pose_angle_axis[0] = T(pose.rx());
  pose_angle_axis[1] = T(pose.ry());
  pose_angle_axis[2] = T(pose.rz());

  T pose_translation[3];
  pose_translation[0] = T(pose.x());
  pose_translation[1] = T(pose.y());
  pose_translation[2] = T(pose.z());

  transformPose6d(pose_angle_axis, pose_translation, angle_axis, tx, t_angle_axis, t_tx);
}

template <typename T>
inline void transformPose6d(const T angle_axis[3], const T tx[3], const Pose6d& pose, T t_angle_axis[3], T t_tx[3])
{
  T pose_angle_axis[3];

  pose_angle_axis[0] = T(pose.rx());
  pose_angle_axis[1] = T(pose.ry());
  pose_angle_axis[2] = T(pose.rz());

  T pose_translation[3];
  pose_translation[0] = T(pose.x());
  pose_translation[1] = T(pose.y());
  pose_translation[2] = T(pose.z());

  transformPose6d(angle_axis, tx, pose_angle_axis, pose_translation, t_angle_axis, t_tx);
}

template <typename T>
inline void transformPose6d(const Pose6d& pose1, const Pose6d& pose2, T t_angle_axis[3], T t_tx[3])
{
  T pose1_angle_axis[3];

  pose1_angle_axis[0] = T(pose1.rx());
  pose1_angle_axis[1] = T(pose1.ry());
  pose1_angle_axis[2] = T(pose1.rz());

  T pose1_translation[3];
  pose1_translation[0] = T(pose1.x());
  pose1_translation[1] = T(pose1.y());
  pose1_translation[2] = T(pose1.z());

  T pose2_angle_axis[3];

  pose2_angle_axis[0] = T(pose2.rx());
  pose2_angle_axis[1] = T(pose2.ry());
  pose2_angle_axis[2] = T(pose2.rz());

  T pose2_translation[3];
  pose2_translation[0] = T(pose2.x());
  pose2_translation[1] = T(pose2.y());
  pose2_translation[2] = T(pose2.z());

  transformPose6d(pose1_angle_axis, pose1_translation, pose2_angle_axis, pose2_translation, t_angle_axis, t_tx);
}

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

  // Perform projection using focal length and camera optical center into image
  // plane
  xy_image[0] = intr.fx() * xp + intr.cx();
  xy_image[1] = intr.fy() * yp + intr.cy();
}

}

#endif // RCT_CERES_MATH_UTILITIES_H
