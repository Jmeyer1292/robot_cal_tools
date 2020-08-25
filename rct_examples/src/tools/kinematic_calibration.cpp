#include <rct_optimizations/dh_chain_kinematic_calibration.h>
#include <rct_optimizations/pnp.h>
#include <rct_image_tools/image_observation_finder.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/exceptions.h>
#include <rct_ros_tools/target_loaders.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <opencv2/highgui.hpp>
#include <random>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

using namespace rct_optimizations;
using namespace rct_ros_tools;
using namespace rct_image_tools;

CircleDetectorParams createDetectorParams()
{
  CircleDetectorParams params;
  params.minThreshold = 50;
  params.maxThreshold = 220;
  params.nThresholds = 20;

  params.minRepeatability = 1;
  params.circleInclusionRadius = 10;
  params.maxRadiusDiff = 10;

  params.maxAverageEllipseError = 0.002;

  params.filterByArea = true;
  params.minArea = 200.0;
  params.maxArea = 10000.0;

  params.filterByConvexity = false;
  params.minConvexity = 0.95f;
  params.maxConvexity = 1.05f;

  params.filterByColor = false;
  params.circleColor = 0;

  params.filterByInertia = false;
  params.filterByCircularity = false;

  return params;
}

KinObservation2D3D::Set loadMeasurements(const std::string& filename, const ModifiedCircleGridObservationFinder& target_finder)
{
  KinObservation2D3D::Set measurements;
  std::vector<Eigen::Vector3d> target_points = target_finder.target().createPoints();

  // Load the data YAML file
  YAML::Node n = YAML::LoadFile(filename);
  measurements.reserve(n.size());

  // Loop over all observations
  for (auto it = n.begin(); it != n.end(); ++it)
  {
    KinObservation2D3D observation;

    // Load the image
    std::string image_file = "ros/" + it->first.as<std::string>() + "_image.png";
    cv::Mat image = cv::Scalar::all(255) - cv::imread(image_file, CV_LOAD_IMAGE_COLOR);  // TODO: Is CV_LOAD_IMAGE_COLOR needed?
    if (image.data == NULL)
      throw std::runtime_error("File failed to load or does not exist: " + image_file);

    // Find the target
    CircleDetectorParams detector_params = createDetectorParams();
    CircleDetector detector(detector_params);
    cv::Mat debug_image = detector.drawDetectedCircles(image);
    cv::namedWindow("circle_detection_debug", 0);
    cv::resizeWindow("circle_detection_debug", 1000, 800);
    cv::imshow("circle_detection_debug", debug_image);
    cv::waitKey();

    auto features = target_finder.findObservations(image, &detector_params);
    if (!features)
    {
      std::cout << "Failed to find observations for image '" << image_file << "'" << std::endl;
      cv::imshow("circle_detection_debug", image);
      cv::waitKey();
      continue;
    }
    else
    {
      cv::Mat obs_image = target_finder.drawObservations(image, features.get());
      cv::imshow("circle_detection_debug", obs_image);

      bool accepted = false;
      cv::MouseCallback cb = [](int event, int x, int y, int flags, void* userdata) {
        if(event == cv::EVENT_FLAG_LBUTTON)
        {
          bool* val = reinterpret_cast<bool*>(userdata);
          *val = true;
        }
      };

      cv::setMouseCallback("circle_detection_debug", cb, &accepted);
      cv::waitKey();

      if (!accepted)
        continue;

      std::cout << "Image accepted!" << std::endl;
    }

    // Create the correspondences
    observation.correspondence_set.reserve(features->size());
    for (std::size_t i = 0; i < target_points.size(); ++i)
    {
      Correspondence2D3D corr;
      corr.in_image = features->at(i);
      corr.in_target = target_points.at(i);
      observation.correspondence_set.push_back(corr);
    }

    // Target chain joints
    {
      YAML::Node joints = it->second["camera_joints"];
      observation.camera_chain_joints.resize(joints.size());
      for (std::size_t i = 0; i < joints.size(); ++i)
      {
        observation.camera_chain_joints[i] = joints[i].as<double>();
      }
    }

    // Camera chain joints
    {
      YAML::Node joints = it->second["target_joints"];
      observation.target_chain_joints.resize(joints.size());
      for (std::size_t i = 0; i < joints.size(); ++i)
      {
        observation.target_chain_joints[i] = joints[i].as<double>();
      }
    }

    // Add the observation to the set
    measurements.push_back(observation);
  }

  return measurements;
}

DHChain createTwoAxisPositioner()
{
  std::vector<DHTransform> transforms;
  transforms.reserve(2);

  Eigen::Vector4d p1, p2;
  p1 << 0.0, 0.0, 0.0, -M_PI / 2.0;
  p2 << -0.475, -M_PI / 2.0, 0.0, 0.0;

  // Add the first DH transform
  {
    DHTransform t(p1, DHJointType::REVOLUTE, "j1");
    t.max = M_PI;
    t.min = -M_PI;
    transforms.push_back(t);
  }
  // Add the second DH transform
  {
    DHTransform dh_transform(p2, DHJointType::REVOLUTE, "j2");
    dh_transform.max = 2.0 * M_PI;
    dh_transform.min = -2.0 * M_PI;
    transforms.push_back(dh_transform);
  }

  // Set an arbitrary base offset
  Eigen::Isometry3d base_offset(Eigen::Isometry3d::Identity());
  base_offset.translate(Eigen::Vector3d(2.2, 0.0, 1.6));
  base_offset.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));

  return DHChain(transforms, base_offset);
}

void test(const DHChain& camera_chain, const DHChain& target_chain,
          const KinematicCalibrationResult& result, const KinObservation2D3D::Set& observations,
          const CameraIntrinsics& intr)
{
  namespace ba = boost::accumulators;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> pos_acc;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> ori_acc;

  for (const KinObservation2D3D& obs : observations)
  {
    // Build the transforms from the camera chain base out to the camera
    Eigen::Isometry3d camera_chain_fk = camera_chain.getFK<double>(obs.camera_chain_joints);
    Eigen::Isometry3d camera_base_to_camera = camera_chain_fk * result.camera_mount_to_camera;

    // Build the transforms from the camera chain base out to the target
    Eigen::Isometry3d target_chain_fk = target_chain.getFK<double>(obs.target_chain_joints);
    Eigen::Isometry3d camera_base_to_target =
        result.camera_base_to_target_base * target_chain_fk * result.target_mount_to_target;

    // Now that we have two transforms in the same frame, get the target point in the camera frame
    Eigen::Isometry3d camera_to_target = camera_base_to_camera.inverse() * camera_base_to_target;

    PnPProblem pnp;
    pnp.camera_to_target_guess = camera_to_target;
    pnp.correspondences = obs.correspondence_set;
    pnp.intr = intr;
    PnPResult result = optimize(pnp);

    if(result.converged)
    {
      // Compare
      Eigen::Isometry3d diff = camera_to_target.inverse() * result.camera_to_target;
      pos_acc(diff.translation().norm());
      ori_acc(Eigen::Quaterniond(camera_to_target.linear()).angularDistance(Eigen::Quaterniond(result.camera_to_target.linear())));
    }
    else
    {
      std::cout << "PnP optimization failed" << std::endl;
    }
  }

  std::cout << "Position Difference Mean: " << ba::mean(pos_acc) << std::endl;
  std::cout << "Position Difference Std. Dev.: " << std::sqrt(ba::variance(pos_acc)) << std::endl;

  std::cout << "Orientation Difference Mean: " << ba::mean(ori_acc) << std::endl;
  std::cout << "Orientation difference Std. Dev.: " << std::sqrt(ba::variance(ori_acc)) << std::endl;
}

template <typename T>
bool get(const ros::NodeHandle& nh, const std::string& key, T& val)
{
  if (!nh.getParam(key, val))
  {
    ROS_ERROR_STREAM("Failed to get '" << key << "' parameter");
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  if (argc != 6)
  {
    std::cout << "Incorrect number of arguments: " << argc << std::endl;
    return -1;
  }

  ModifiedCircleGridTarget target = TargetLoader<ModifiedCircleGridTarget>::load(argv[2]);
  ModifiedCircleGridObservationFinder target_finder(target);

  // Create the problem
  KinematicCalibrationProblem2D3D problem(DHChain({}), createTwoAxisPositioner());

  // Load the camera intrinsics
  problem.intr = loadIntrinsics(argv[3]);

  // Load the pose guesses
  problem.target_mount_to_target_guess = loadPose(argv[4]);
  problem.camera_mount_to_camera_guess = loadPose(argv[5]);

  // Load the observations
  KinObservation2D3D::Set measurements = loadMeasurements(argv[1], target_finder);

  problem.observations = measurements;
  std::cout << "Performing calibration with " << problem.observations.size() << " observations" << std::endl;

  problem.camera_chain_dh_stdev_expectation = 0.001;
  problem.target_chain_dh_stdev_expectation = 0.005;

  // Mask a few DH parameters in the target chain (index 1)
  {
    Eigen::Matrix<bool, Eigen::Dynamic, 4> mask =
        Eigen::Matrix<bool, Eigen::Dynamic, 4>::Constant(problem.target_chain.dof(), 4, false);

    // Mask the last row because they duplicate the target mount to target transform
    mask.bottomRows(1) << true, true, true, true;

    // Add the mask to the problem
    problem.mask.at(1) = createDHMask(mask);
  }

  /* Mask the camera base to target base transform (duplicated by target mount to target transform when
   * the target chain has no joints */
  problem.mask.at(6) = { 0, 1, 2 };
  problem.mask.at(7) = { 0, 1, 2 };

  ceres::Solver::Options options;
  options.num_threads = 4;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 500;
  options.use_nonmonotonic_steps = true;
  options.function_tolerance = 1.0e-12;
  options.parameter_tolerance = 1.0e-20;
  options.gradient_tolerance = 1.0e-16;

  KinematicCalibrationResult result = optimize(problem, options);

  Eigen::IOFormat fmt(6, 0, "|", "\n", "|", "|");

  std::stringstream ss;
  ss << "\nCalibration " << (result.converged ? "did" : "did not") << " converge\n";
  ss << "Initial cost per observation: " << std::sqrt(result.initial_cost_per_obs) << "\n";
  ss << "Final cost per observation: " << std::sqrt(result.final_cost_per_obs) << "\n";

  ss << "\nCamera mount to camera\n" << result.camera_mount_to_camera.matrix().format(fmt) << "\n";
  ss << "Euler ZYX: " << result.camera_mount_to_camera.rotation().eulerAngles(2, 1, 0).transpose().format(fmt) << "\n";

  ss << "\nTarget mount to target\n" << result.target_mount_to_target.matrix().format(fmt) << "\n";
  ss << "Euler ZYX: " << result.target_mount_to_target.rotation().eulerAngles(2, 1, 0).transpose().format(fmt) << "\n";

  ss << "\nTarget chain DH parameter offsets\n" << result.target_chain_dh_offsets.matrix().format(fmt) << "\n";
  ss << "\nCamera chain DH parameter offsets\n" << result.camera_chain_dh_offsets.matrix().format(fmt) << "\n";
  ss << result.covariance.printCorrelationCoeffAboveThreshold(0.5);

  std::cout << ss.str() << std::endl;

  // Test the result by moving the robot around to a lot of positions and seeing of the results match
  DHChain optimized_camera_chain(problem.camera_chain, result.camera_chain_dh_offsets);
  DHChain optimized_target_chain(problem.target_chain, result.target_chain_dh_offsets);

  std::cout << "Optimized camera chain DH parameters\n" << optimized_camera_chain.getDHTable().format(fmt) << std::endl;
  std::cout << "Optimized target chain DH parameters\n" << optimized_target_chain.getDHTable().format(fmt) << std::endl;

  std::cout << "\nValidating calibration with " << measurements.size() << " observations" << std::endl;
  test(optimized_camera_chain, optimized_target_chain, result, measurements, problem.intr);

  return 0;
}
