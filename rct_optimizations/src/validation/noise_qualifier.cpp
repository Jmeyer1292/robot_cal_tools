#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <rct_optimizations/validation/noise_qualifier.h>
#include <rct_optimizations/experimental/pnp.h>

//accumulators
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

namespace rct_optimizations
{

 PnPNoiseStat qualifyNoise2D(const std::vector<PnPProblem>& params)
 {

  PnPNoiseStat output;
  std::size_t count = params.size();

  std::vector<Eigen::Isometry3d> solution_transforms;
  solution_transforms.reserve(count);

  std::vector<Eigen::Vector3d> translations;
  translations.reserve(count);

  std::vector<Eigen::Vector4d> orientations;
  orientations.reserve(count);

  using namespace boost::accumulators;
  accumulator_set<double, stats<tag::mean, tag::variance>> x_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> y_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> z_acc;


  for (auto& prob : params)
  {
    rct_optimizations::PnPResult result;

    result = rct_optimizations::optimize(prob);

    if (result.converged)
    {
      //we will save the full result here for debugging purposes
      solution_transforms.push_back(result.camera_to_target);
      translations.push_back(solution_transforms.back().translation());

      x_acc(result.camera_to_target.translation()(0));
      y_acc(result.camera_to_target.translation()(1));
      z_acc(result.camera_to_target.translation()(2));

      Eigen::Quaterniond q;
      Eigen::Matrix3d m = result.camera_to_target.rotation();
      q = m;

      Eigen::Vector4d v(q.x(), q.y(), q.z(),q.w());

      orientations.push_back(v);
    }
  }

  output.x.mean = boost::accumulators::mean(x_acc);
  output.y.mean = boost::accumulators::mean(y_acc);
  output.z.mean = boost::accumulators::mean(z_acc);
  output.x.std_dev = sqrt(boost::accumulators::variance(x_acc));
  output.y.std_dev = sqrt(boost::accumulators::variance(y_acc));
  output.z.std_dev = sqrt(boost::accumulators::variance(z_acc));

  //Mean quaternion is found using method described by Markley et al: Quaternion Averaging
  //All quaternions are equally weighted

  //Accumulator Matrix
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

  for (std::size_t i = 0; i <= count; ++i)
  {
    //rank 1 update of 'A', the accumulator matrix, with the u,v = q,qT
    Eigen::Matrix4d temp = (orientations[i] * orientations[i].transpose());
    A += temp;
  }

  //scale A
  A = (1.0/double(count)) * A;

  //Eigenvectors,values should be strictly real
  Eigen::EigenSolver<Eigen::Matrix4d> E(A, true);

  //Each column of 4x4 vectors is an eigenvector; desired mean has max eigenvalue
  Eigen::Index max_evi, one;

  //find maximium eigenvalue, and store its index in max_evi
  E.eigenvalues().real().maxCoeff(&max_evi, &one);
  Eigen::Vector4d mean = E.eigenvectors().real().col(max_evi);

  output.i.mean = mean[0];
  output.j.mean = mean[1];
  output.k.mean = mean[2];
  output.w.mean = mean[3];

  //Manually calculate standard deviations from mean
  Eigen::Array4d std_dev = Eigen::Array4d::Zero();
  Eigen::Array4d sum = Eigen::Array4d::Zero();
  for (std::size_t i =0; i< count; ++i)
  {
   //absolute value taken because a quaternion is equal to its opposite
   Eigen::Array4d term = orientations[i].cwiseAbs() - mean.cwiseAbs();
   term = term.square();
   sum+= term;
  }

  std_dev = sum/double(count);
  std_dev = std_dev.sqrt();



  output.i.std_dev = std_dev.x();//(1);
  output.j.std_dev = std_dev.y();//(2);
  output.k.std_dev = std_dev.z();//(3);
  output.w.std_dev = std_dev.w();//(0);

  //Output: mean & standard deviation of x,y,z,i,j,k,w
  return output;
}

 PnPNoiseStat qualifyNoise3D(const std::vector<PnPProblem3D>& params)
 {

  PnPNoiseStat output;
  std::size_t count = params.size();

  std::vector<Eigen::Isometry3d> solution_transforms;
  solution_transforms.reserve(count);

  std::vector<Eigen::Vector3d> translations;
  translations.reserve(count);

  std::vector<Eigen::Vector4d> orientations;
  orientations.reserve(count);

  using namespace boost::accumulators;
  accumulator_set<double, stats<tag::mean, tag::variance>> x_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> y_acc;
  accumulator_set<double, stats<tag::mean, tag::variance>> z_acc;


  for (auto& prob : params)
  {
    rct_optimizations::PnPResult result;

    result = rct_optimizations::optimize(prob);

    if (result.converged)
    {
      //we will save the full result here for debugging purposes
      solution_transforms.push_back(result.camera_to_target);
      translations.push_back(solution_transforms.back().translation());

      x_acc(result.camera_to_target.translation()(0));
      y_acc(result.camera_to_target.translation()(1));
      z_acc(result.camera_to_target.translation()(2));

      Eigen::Quaterniond q;
      Eigen::Matrix3d m = result.camera_to_target.rotation();
      q = m;

      Eigen::Vector4d v(q.x(), q.y(), q.z(),q.w());

      orientations.push_back(v);
    }
  }

  output.x.mean = boost::accumulators::mean(x_acc);
  output.y.mean = boost::accumulators::mean(y_acc);
  output.z.mean = boost::accumulators::mean(z_acc);
  output.x.std_dev = sqrt(boost::accumulators::variance(x_acc));
  output.y.std_dev = sqrt(boost::accumulators::variance(y_acc));
  output.z.std_dev = sqrt(boost::accumulators::variance(z_acc));

  //Mean quaternion is found using method described by Markley et al: Quaternion Averaging
  //All quaternions are equally weighted

  //Accumulator Matrix
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

  for (std::size_t i = 0; i <= count; ++i)
  {
    //rank 1 update of 'A', the accumulator matrix, with the u,v = q,qT
    Eigen::Matrix4d temp = (orientations[i] * orientations[i].transpose());
    A += temp;
  }

  //scale A
  A = (1.0/double(count)) * A;

  //Eigenvectors,values should be strictly real
  Eigen::EigenSolver<Eigen::Matrix4d> E(A, true);

  //Each column of 4x4 vectors is an eigenvector; desired mean has max eigenvalue
  Eigen::Index max_evi, one;

  //find maximium eigenvalue, and store its index in max_evi
  E.eigenvalues().real().maxCoeff(&max_evi, &one);
  Eigen::Vector4d mean = E.eigenvectors().real().col(max_evi);

  output.i.mean = mean[0];
  output.j.mean = mean[1];
  output.k.mean = mean[2];
  output.w.mean = mean[3];

  //Manually calculate standard deviations from mean
  Eigen::Array4d std_dev = Eigen::Array4d::Zero();
  Eigen::Array4d sum = Eigen::Array4d::Zero();
  for (std::size_t i =0; i< count; ++i)
  {
   //absolute value taken because a quaternion is equal to its opposite
   Eigen::Array4d term = orientations[i].cwiseAbs() - mean.cwiseAbs();
   term = term.square();
   sum+= term;
  }

  std_dev = sum/double(count);
  std_dev = std_dev.sqrt();



  output.i.std_dev = std_dev.x();//(1);
  output.j.std_dev = std_dev.y();//(2);
  output.k.std_dev = std_dev.z();//(3);
  output.w.std_dev = std_dev.w();//(0);

  //Output: mean & standard deviation of x,y,z,i,j,k,w
  return output;
}

}//rct_optimizations
