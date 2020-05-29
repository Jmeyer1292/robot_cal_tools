#include <Eigen/Dense>
#include <rct_image_tools/noise_qualifier.h>
#include <rct_optimizations/types.h>
#include <rct_optimizations_tests/utilities.h>

#include "rct_optimizations_tests/pose_generator.h"

//#include  "rct_optimizations_tests/observation_creator.h"

//collect 2d images
int main(int argc, char** argv)
{
  rct_optimizations::Observation2D3D::Set obs;
  obs.reserve(50);

  Eigen::Isometry3d target_loc = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d camera_loc =  Eigen::Isometry3d::Identity();

  //need a reasonable location
  camera_loc.translate(Eigen::Vector3d(1.0,1.0,1.0));
  camera_loc = rct_optimizations::test::lookAt(camera.translation(), target.translation(), Eigen::Vector3d(1.0,0.0,0.0));

  //camera intrinsics?
  rct_optimizarions::CameraIntrinsics CI;

  //large target
  rct_image_tools::ModifiedCircleGridTarget target(10, 10, 0.0254);

  //need observations
  for (std::size_t i = 0; i < 50; ++i)
  {
    Observation2D3D ob;
    ob.to_target_mount = target_loc;
    ob_to_camera_mount = camera_loc;
    //heres where we can add noise
    ob.correspondence_set = getCorrespondences(target_loc,
                                               camera_loc,
                                               camera,
                                               target,
                                               false);

    obs.push_back(rct_optimizations);
    //observations need camera pose, target pose, camera def, target def, and some bool?
  }

  rct_image_tools::NoiseQualParams2D3D param(obs, CI, guess);
  rct_image_tools::NoiseStatistics qualifyNoise(param);

}
