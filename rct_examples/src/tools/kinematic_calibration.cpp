#include <rct_optimizations/dh_chain_kinematic_calibration.h>
#include <rct_optimizations/pnp.h>
#include <rct_image_tools/image_observation_finder.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/exceptions.h>

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
      ROS_WARN_STREAM("Failed to find observations for image '" << image_file << "'");
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
      {
        ROS_WARN_STREAM("Not accepted!");
        continue;
      }
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

std::pair<KinObservation2D3D::Set, KinObservation2D3D::Set> divide(const KinObservation2D3D::Set& measurements,
                                                                   const double training_pct)
{
  std::vector<std::size_t> cal_indices(measurements.size());
  std::iota(cal_indices.begin(), cal_indices.end(), 0);
  std::mt19937 mt_rand(std::random_device{}());
  std::shuffle(cal_indices.begin(), cal_indices.end(), mt_rand);

  std::size_t n = static_cast<std::size_t>(static_cast<double>(measurements.size()) * training_pct);

  KinObservation2D3D::Set training_set;
  {
    std::vector<std::size_t> training_indices(cal_indices.begin(), cal_indices.begin() + n);
    training_set.reserve(training_indices.size());
    for (const std::size_t idx : training_indices)
    {
      training_set.push_back(measurements.at(idx));
    }
  }

  KinObservation2D3D::Set test_set;
  {
    std::vector<std::size_t> test_indices(cal_indices.begin() + n, cal_indices.end());
    test_set.reserve(test_indices.size());
    for (const std::size_t idx : test_indices)
    {
      test_set.push_back(measurements.at(idx));
    }
  }

  return std::make_pair(training_set, test_set);
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

void test(const DHChain& initial_camera_chain, const DHChain& initial_target_chain,
          const KinematicCalibrationResult& result, const KinObservation2D3D::Set& observations,
          const CameraIntrinsics& intr)
{
  // Test the result by moving the robot around to a lot of positions and seeing of the results match
  DHChain camera_chain(initial_camera_chain, result.camera_chain_dh_offsets);
  DHChain target_chain(initial_target_chain, result.target_chain_dh_offsets);

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
      ROS_WARN_STREAM("PnP optimization failed");
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
  //  ros::init(argc, argv, "kinematic_calibration");
  //  ros::NodeHandle pnh("~");

  //  std::string data_file;
  //  if (!get(pnh, "data_file", data_file))
  //    return -1;

  ModifiedCircleGridTarget target(9, 7, 0.078581);
//  ModifiedCircleGridTarget target(7, 5, 0.06);
  ModifiedCircleGridObservationFinder target_finder(target);

  // Load the observations
  KinObservation2D3D::Set measurements = loadMeasurements(argv[1], target_finder);

  // Split the observations into a training and validation group
  std::pair<KinObservation2D3D::Set, KinObservation2D3D::Set> measurement_sets = divide(measurements, 0.8);

  KinematicCalibrationProblem2D3D problem(DHChain({}), createTwoAxisPositioner());
  problem.intr.fx() = 2246.59;
  problem.intr.fy() = 2245.66;
  problem.intr.cx() = 1039.16;
  problem.intr.cy() = 800.869;

  problem.observations = measurement_sets.first;
  ROS_INFO_STREAM("Performing calibration with " << problem.observations.size() << " observations");

  problem.target_mount_to_target_guess = Eigen::Isometry3d::Identity();

//  // Vicon
//  {
//    problem.target_mount_to_target_guess.translate(Eigen::Vector3d(0.17, -0.65, 0.5));
//    problem.target_mount_to_target_guess.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
//    problem.target_mount_to_target_guess.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));
//    problem.target_mount_to_target_guess.rotate(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));

//    problem.camera_mount_to_camera_guess = Eigen::Isometry3d::Identity();
//    problem.camera_mount_to_camera_guess.translate(Eigen::Vector3d(2.2, 0.7, 1.075));
//  }

//  // 7x5 Target
//  {
//    problem.target_mount_to_target_guess.translate(Eigen::Vector3d(-0.2, -0.15, 0.7));
//    problem.target_mount_to_target_guess.rotate(Eigen::Quaterniond(0.743, 0.0, 0.0, -0.669));

//    problem.camera_mount_to_camera_guess.translate(Eigen::Vector3d(2.58, -1.6, 2.83));
//    problem.camera_mount_to_camera_guess.rotate(Eigen::Quaterniond(-0.463, 0.883, 0.002, -0.075));
//  }

  // 9x7 Target
  {
    problem.target_mount_to_target_guess.translate(Eigen::Vector3d(0.2806, -0.5019, 0.7042));
//    problem.target_mount_to_target_guess.rotate(Eigen::Quaterniond(0.669, 0.0, 0.0, 0.743));
    problem.target_mount_to_target_guess.rotate(Eigen::Quaterniond(0.6744918, -0.0005498, -0.0008313, 0.7382817));

    problem.camera_mount_to_camera_guess.translate(Eigen::Vector3d(2.555, -1.762, 2.832));
//    problem.camera_mount_to_camera_guess.rotate(Eigen::Quaterniond(-0.463, 0.883, 0.002, -0.075));
    problem.camera_mount_to_camera_guess.rotate(Eigen::Quaterniond(0.4608509, -0.8808954, -0.0562409, 0.092069));
  }


  problem.camera_base_to_target_base_guess = Eigen::Isometry3d::Identity();

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

  // Mask the z-value of the camera mount to camera position
//  problem.mask.at(2) = { 0, 1, 2 };

  KinematicCalibrationResult result = optimize(problem);

  Eigen::IOFormat fmt(4, 0, "|", "\n", "|", "|");

  std::stringstream ss;
  ss << "\nCalibration " << (result.converged ? "did" : "did not") << " converge\n";
  ss << "Initial cost per observation: " << std::sqrt(result.initial_cost_per_obs) << "\n";
  ss << "Final cost per observation: " << std::sqrt(result.final_cost_per_obs) << "\n";

  ss << "\nCamera mount to camera\n" << result.camera_mount_to_camera.matrix().format(fmt) << "\n";
  ss << result.camera_mount_to_camera.rotation().eulerAngles(0, 1, 2).transpose() << "\n";

  ss << "\nTarget mount to target\n" << result.target_mount_to_target.matrix().format(fmt) << "\n";
  ss << result.target_mount_to_target.rotation().eulerAngles(0, 1, 2).transpose() << "\n";

  ss << "\nTarget chain DH parameter offsets\n" << result.target_chain_dh_offsets.matrix().format(fmt) << "\n";
  ss << "\nCamera chain DH parameter offsets\n" << result.camera_chain_dh_offsets.matrix().format(fmt) << "\n";
  ss << result.covariance.printCorrelationCoeffAboveThreshold(0.5);

  std::cout << ss.str() << std::endl;

  ROS_INFO_STREAM("Validating calibration with " << measurement_sets.second.size() << " observations");
  test(problem.camera_chain, problem.target_chain, result, measurement_sets.second, problem.intr);

  return 0;
}
