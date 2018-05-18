#ifndef RCT_TYPES_H
#define RCT_TYPES_H

#include <array>

namespace rct_optimizations
{

struct Observation2d
{
  std::array<double, 2> values;
  Observation2d() = default;

  double& x() { return values[0]; }
  double& y() { return values[1]; }
  const double& x() const { return values[0]; }
  const double& y() const { return values[1]; }
};

struct CameraIntrinsics
{
  std::array<double, 4> values;

  double& fx() { return values[0]; }
  double& fy() { return values[1]; }
  double& cx() { return values[2]; }
  double& cy() { return values[3]; }

  const double& fx() const { return values[0]; }
  const double& fy() const { return values[1]; }
  const double& cx() const { return values[2]; }
  const double& cy() const { return values[3]; }
};

struct Pose6d
{
  Pose6d() = default;
  Pose6d(std::array<double, 6> l) : values(l) {}

  std::array<double, 6> values;

  double& rx() { return values[0]; }
  double& ry() { return values[1]; }
  double& rz() { return values[2]; }
  double& x() { return values[3]; }
  double& y() { return values[4]; }
  double& z() { return values[5]; }
  const double& rx() const { return values[0]; }
  const double& ry() const { return values[1]; }
  const double& rz()const  { return values[2]; }
  const double& x() const { return values[3]; }
  const double& y() const { return values[4]; }
  const double& z() const { return values[5]; }
};

struct Point3d
{
  std::array<double, 3> values;
};

}

#endif
