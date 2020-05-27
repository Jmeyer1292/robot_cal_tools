#include <Eigen/Dense>
#include <rct_image_tools/noise_qualifier.h>
#include <rct_optimizations/observation_creator.h>

//collect 2d images
int main()
{
  std::vector<rct_observations::Observation2D3D> obs;
  obs.reserve(50);

  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  Eigen::Isomety3d camera =  Eigen::Isometry3d::Identity();

  //need a reasonable location
  camera.translate(1.0,1.0,1.0);
  camera = rct_optimizations::test::looksAt(camera.translation(), target.translation(), Eigen::Vector3d(1.0,0.0,0.0));

  //camera intrinsics?
  rct_optimizarions::CameraIntrinsics CI;

  //target type?

  for (std::size_t i = 0; i < 50; ++i)
  {
    //take observation, push it back
    //observations need camera pose, target pose, camera def, target def, and some bool?
  }

  //run noise qualifier

}
