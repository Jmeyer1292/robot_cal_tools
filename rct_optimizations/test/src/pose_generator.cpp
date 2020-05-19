#include "rct_optimizations_tests/observation_creator.h"
#include "rct_optimizations_tests/pose_generator.h"
#include <stdexcept>

//hemisphere

std::vector<Eigen::Isometry3d> rct_optimizations::test::genHemispherePose(Eigen::Isometry3d target_pose,
                                                                          double r,
                                                                          int theta_cnt,
                                                                          int phi_cnt
                                                                          )
{

  //all in the target coordinate system, with z forward
  std::vector<Eigen::Isometry3d>camera_positions;

  Eigen::VectorXd theta_range = Eigen::VectorXd::LinSpaced(theta_cnt, 0.0, M_PI);
  Eigen::VectorXd phi_range = Eigen::VectorXd::LinSpaced(phi_cnt, 0.0, M_PI);

  for (long theta_it = 0; theta_it < theta_range.size(); ++theta_it)
  {
    std::vector<Eigen::Isometry3d> local_scan_path;
    local_scan_path.reserve(static_cast<std::size_t>(phi_range.size() * theta_range.size()));
    for (long phi_it = 0; phi_it < phi_range.size(); ++phi_it)
    {

      double theta_cur = theta_range(theta_it);
      double phi_cur = phi_range(phi_it);

      //position in target coordinate frame
      Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();
      camera.translation()(0) = r * std::cos(theta_cur) * std::sin(phi_cur);
      camera.translation()(1) = r * std::sin(theta_cur) * std::sin(phi_cur);
      camera.translation()(2) = r * std::cos(phi_cur);

      //camera may not need to be isometry since we only use the translation portion
      //x is 'up' in target frame (confirm)
      Eigen::Isometry3d camera_oriented = rct_optimizations::test::lookAt(camera.translation(), target_pose.translation(), Eigen::Vector3d(1, 0, 0));

      //this vector is still in target spatial coordinates
      camera_positions.push_back(camera_oriented);
      //for world frame need target * position
      // camera_positions.push_back(focus_ * camera_oriented)?
    }
  }

  return camera_positions;
}

std::vector<Eigen::Isometry3d> rct_optimizations::test::genConicalPose(Eigen::Isometry3d target_pose,
                                                      int observations,
                                                      double r,
                                                      double h
                                                      )
{
  std::vector<Eigen::Isometry3d> camera_positions;
  //Generates positions in target frame; need to convert to world frame
  double dt = 2.0f * M_PI / (double)(observations); //delta theta about cone base

  for(int i = 0; i < observations; ++i)
  {
    Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
    camera_pose.translation() = target_pose.translation();

    //preserving target spatial coordinate frame:
    camera_pose.translate(Eigen::Vector3d{r * cos(i*dt), r*sin(i*dt), h});

    //change orientation to look at target
    Eigen::Isometry3d camera_oriented = rct_optimizations::test::lookAt( camera_pose.translation(), target_pose.translation(), Eigen::Vector3d(1, 0, 0));

    camera_positions.push_back(camera_oriented);
    //for world frame need target * position
    // camera_positions.push_back(target_pose * camera_oriented)?
  }

  return camera_positions;
}


