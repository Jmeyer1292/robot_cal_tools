#include <rct_image_tools/image_observation_finder.h>
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/experimental/rail_intrinsic.h>
#include "rct_ros_tools/data_set.h"
#include "rct_ros_tools/parameter_loaders.h"
#include "rct_ros_tools/print_utils.h"

#include <opencv2/highgui.hpp>
#include <ros/ros.h>

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

}

static
std::vector<cv::Point2d> getReprojections(const Eigen::Affine3d &camera_to_target,
                                          const rct_optimizations::CameraIntrinsics &intr,
                                          const std::array<double, 5>& dist,
                                          const std::vector<Eigen::Vector3d> &target_points)
{
  std::array<double, 9> camera_data;
  MutableCalibCameraIntrinsics<double> camera (camera_data.data());

  camera.fx() = intr.fx();
  camera.fy() = intr.fy();
  camera.cx() = intr.cx();
  camera.cy() = intr.cy();

  camera.k1() = dist[0];
  camera.k2() = dist[1];
  camera.p1() = dist[2];
  camera.p2() = dist[3];
  camera.k3() = dist[4];

  std::vector<cv::Point2d> reprojections;
  for (const auto& point_in_target : target_points)
  {
    Eigen::Vector3d in_camera = camera_to_target * point_in_target;

    double uv[2];
    projectPoints2(camera_data.data(), in_camera.data(), uv);

    reprojections.push_back(cv::Point2d(uv[0], uv[1]));
  }
  return reprojections;
}

// This cost has a custom file loader & format
struct RailCalibrationScene
{
  std::vector<double> rail_poses;
  std::vector<rct_optimizations::CorrespondenceSet> correspondences;
  std::vector<cv::Mat> images;
};

struct RailCalibrationData
{
  std::vector<RailCalibrationScene> scenes;
};

bool parseNameData(const std::string& name, int& scene_id, int& position_id)
{
  //  Only parses files of the following format: my_camera_000_000.jpg
  std::string scene_str(name.begin() + 10, name.begin() + 13);
  std::string pos_str(name.begin() + 14, name.begin() + 17);

  scene_id = std::stoi(scene_str);
  position_id = std::stoi(pos_str);

  return true;
}

rct_optimizations::CorrespondenceSet zip(const std::vector<Eigen::Vector2d>& image,
                                         const std::vector<Eigen::Vector3d>& target)
{
  rct_optimizations::CorrespondenceSet set;
  for (std::size_t i = 0; i < target.size(); ++i)
  {
    rct_optimizations::Correspondence2D3D c;
    c.in_image = image[i];
    c.in_target = target[i];
    set.push_back(c);
  }
  return set;
}

RailCalibrationData parseCalData(const std::string& base_path, const std::vector<std::string>& images,
                                 const rct_image_tools::ModifiedCircleGridTarget& target)
{
  std::map<int, RailCalibrationScene> scenes;
  rct_image_tools::ModifiedCircleGridObservationFinder finder (target);

  for (const auto& f : images)
  {
    const auto full_path = base_path + f;

    cv::Mat image = cv::imread(full_path);
    if (image.data == nullptr) {
      ROS_WARN_STREAM("Unable to load " << full_path);
      continue;
    }

    int scene_id = -1;
    int position_id = -1;

    if (!parseNameData(f, scene_id, position_id)) {
      ROS_WARN_STREAM("Unable to parse name: " << f);
      continue;
    }

    ROS_INFO_STREAM("Showing: " << f << " " << scene_id << " " << position_id);

    cv::imshow("in", image);
    auto maybe = finder.findObservations(image);
    if (maybe)
    {
      cv::imshow("found", finder.drawObservations(image, *maybe));

      const double rail_pos = -0.05 * static_cast<double>(position_id);

      if (scenes.count(scene_id) != 0)
      {
        auto& scene = scenes[scene_id];
        scene.rail_poses.push_back(rail_pos);
        scene.correspondences.push_back(zip(*maybe, target.points));
        scene.images.push_back(image);
      }
      else // make a new scene
      {
        RailCalibrationScene scene;
        scene.rail_poses.push_back(rail_pos);
        scene.correspondences.push_back(zip(*maybe, target.points));
        scene.images.push_back(image);
        scenes[scene_id] = scene;
      }
    }

    cv::waitKey(30);
  }

  // Assemble the images into a final set
  RailCalibrationData data;
  for (const auto& pair : scenes)
  {
    auto scene_id = pair.first;
    auto& scene_data = pair.second;
    data.scenes.push_back(scene_data);
    ROS_INFO_STREAM("Found scene: " << scene_id);
  }

  return data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rct_rail_intrinsic_ex", ros::init_options::AnonymousName);
  ros::NodeHandle pnh ("~");

  rct_image_tools::ModifiedCircleGridTarget target;
  if (!rct_ros_tools::loadTarget(pnh, "target", target))
  {
    return 1;
  }

  rct_optimizations::CameraIntrinsics intr;
  if (!rct_ros_tools::loadIntrinsics(pnh, "intrinsics", intr))
  {
    return 2;
  }

  // Load data sets
  std::vector<std::string> image_files;
  if (!pnh.getParam("images", image_files))
  {
    return 3;
  }

  std::string base_path;
  if (!pnh.getParam("image_base", base_path))
  {
    return 4;
  }

  auto data = parseCalData(base_path, image_files, target);

  // Now we construct our problem
  rct_optimizations::RailIntrinsicProblem problem;
  problem.intrinsics_guess = intr;
  problem.skew_x_guess = problem.skew_y_guess = 0.0;

  for (const auto& scene : data.scenes)
  {
    // set the target pose guess
    auto target_pose = Eigen::Affine3d::Identity();
    target_pose.translation() = Eigen::Vector3d(0, 0, 1.5);
    target_pose.matrix().col(0).head<3>() = Eigen::Vector3d(-1, 0, 0);
    target_pose.matrix().col(1).head<3>() = Eigen::Vector3d(0, 1, 0);
    target_pose.matrix().col(2).head<3>() = Eigen::Vector3d(0, 0, -1);
    problem.extrinsic_guesses.push_back(target_pose);

    problem.image_observations.push_back(scene.correspondences);
    problem.rail_distances.push_back(scene.rail_poses);
  }

  auto result = rct_optimizations::optimize(problem);

  // Print the results
  rct_ros_tools::printOptResults(result.converged, result.initial_cost_per_obs, result.final_cost_per_obs);
  rct_ros_tools::printNewLine();

  rct_ros_tools::printCameraIntrinsics(result.intrinsics.values, "RCT Intrinsics");
  rct_ros_tools::printNewLine();

  rct_ros_tools::printCameraDistortion(result.distortions, "RCT Distortion");
  rct_ros_tools::printNewLine();

  for (std::size_t i = 0; i < result.target_transforms.size(); ++i)
  {
    std::cout << "Target Origin Scene " << i << "\n";
    std::cout << result.target_transforms[i].matrix() << "\n";
  }

  std::cout << "Skew: " << result.skew_x << ", " << result.skew_y << "\n";

  // Reproject
  double aa[3] = {result.skew_x, result.skew_y, 0.0};
  double axis1[3] = {0, 0, 1};
  Eigen::Vector3d rail_axis;

  ceres::AngleAxisRotatePoint(aa, axis1, rail_axis.data());

  for (std::size_t i = 0; i < result.target_transforms.size(); ++i) // For each scene
  {
    const auto target_origin = result.target_transforms[i];
    std::cout << "** SCENE " << i  << " **\n";
    for (std::size_t j = 0; j < data.scenes[i].images.size(); ++j) // for each rail position
    {
      // Compute target in camera
      const Eigen::Vector3d v = problem.rail_distances[i][j] * rail_axis;
      auto target_in_camera = target_origin;
      target_in_camera .translation() += v;

      // Reproject and draw
      auto pts = getReprojections(target_in_camera, result.intrinsics, result.distortions, target.points);
      cv::Mat frame = data.scenes[i].images[j].clone();
      rct_image_tools::drawReprojections(pts, 3, cv::Scalar(0, 0, 255), frame);
      cv::imshow("repr", frame);
      cv::waitKey();
    }
  }

  return 0;
}
