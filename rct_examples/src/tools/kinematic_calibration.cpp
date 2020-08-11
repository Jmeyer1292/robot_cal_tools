#include <rct_optimizations/dh_chain_kinematic_calibration.h>
#include <rct_ros_tools/parameter_loaders.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

#include <random>

using namespace rct_optimizations;
using namespace rct_ros_tools;

KinematicMeasurement::Set loadMeasurements(const std::string& filename)
{
  KinematicMeasurement::Set measurements;

  try
  {
    YAML::Node n = YAML::LoadFile(filename);
    measurements.reserve(n.size());

    for (auto it = n.begin(); it != n.end(); ++it)
    {
      KinematicMeasurement measurement;

      // Target chain joints
      {
        YAML::Node joints = it->second["camera_joints"];
        measurement.target_chain_joints.resize(joints.size());
        for (std::size_t i = 0; i < joints.size(); ++i)
        {
          measurement.target_chain_joints[i] = joints[i].as<double>();
        }
      }

      // Camera chain joints
      {
        YAML::Node joints = it->second["target_joints"];
        measurement.target_chain_joints.resize(joints.size());
        for (std::size_t i = 0; i < joints.size(); ++i)
        {
          measurement.target_chain_joints[i] = joints[i].as<double>();
        }
      }

      // Camera to target pose
      YAML::Node pose = it->second["pose"];
      double x = pose["x"].as<double>();
      double y = pose["y"].as<double>();
      double z = pose["z"].as<double>();
      double rx = pose["rx"].as<double>();
      double ry = pose["ry"].as<double>();
      double rz = pose["rz"].as<double>();

      measurement.camera_to_target = Eigen::Isometry3d::Identity();
      measurement.camera_to_target.translate(Eigen::Vector3d(x, y, z));

      // Euler XYZ
      measurement.camera_to_target.rotate(Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()));
      measurement.camera_to_target.rotate(Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()));
      measurement.camera_to_target.rotate(Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()));

      // Add the measurement to the set
      measurements.push_back(measurement);
    }
  }
  catch (YAML::Exception& ex)
  {
    throw rct_ros_tools::BadFileException(std::string("YAML failure: ") + ex.what());
  }

  return measurements;
}

std::pair<KinematicMeasurement::Set, KinematicMeasurement::Set> divide(const KinematicMeasurement::Set& measurements,
                                                                       const double training_pct)
{
  std::vector<std::size_t> cal_indices(measurements.size());
  std::iota(cal_indices.begin(), cal_indices.end(), 0);
  std::mt19937 mt_rand(std::random_device{}());
  std::random_shuffle(cal_indices.begin(), cal_indices.end()); //, mt_rand);

  std::size_t n = static_cast<std::size_t>(static_cast<double>(measurements.size()) * training_pct);

  std::vector<std::size_t> training_indices(cal_indices.begin(), cal_indices.begin() + n);
  KinematicMeasurement::Set training_set;
  training_set.reserve(training_indices.size());
  for (const std::size_t idx : training_indices)
  {
    training_set.push_back(measurements.at(idx));
  }

  std::vector<std::size_t> test_indices(cal_indices.begin() + n, cal_indices.end());
  KinematicMeasurement::Set test_set;
  test_set.reserve(test_indices.size());
  for (const std::size_t idx : test_indices)
  {
    test_set.push_back(measurements.at(idx));
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
          const KinematicCalibrationResult& result, const KinematicMeasurement::Set& measurements)
{
  // Test the result by moving the robot around to a lot of positions and seeing of the results match
  DHChain camera_chain(initial_camera_chain, result.camera_chain_dh_offsets);
  DHChain target_chain(initial_target_chain, result.target_chain_dh_offsets);

  namespace ba = boost::accumulators;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> pos_acc;
  ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> ori_acc;

  for (const KinematicMeasurement& m : measurements)
  {
    // Build the transforms from the camera chain base out to the camera
    Eigen::Isometry3d camera_chain_fk = camera_chain.getFK<double>(m.camera_chain_joints);
    Eigen::Isometry3d camera_base_to_camera = camera_chain_fk * result.camera_mount_to_camera;

    // Build the transforms from the camera chain base out to the target
    Eigen::Isometry3d target_chain_fk = target_chain.getFK<double>(m.target_chain_joints);
    Eigen::Isometry3d camera_base_to_target =
        result.camera_base_to_target_base * target_chain_fk * result.target_mount_to_target;

    // Now that we have two transforms in the same frame, get the target point in the camera frame
    Eigen::Isometry3d camera_to_target = camera_base_to_camera.inverse() * camera_base_to_target;

    // Compare
    Eigen::Isometry3d diff = camera_to_target.inverse() * m.camera_to_target;
    pos_acc(diff.translation().norm());
    ori_acc(Eigen::Quaterniond(camera_to_target.linear()).angularDistance(Eigen::Quaterniond(m.camera_to_target.linear())));
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


  // Load the observations
  KinematicMeasurement::Set measurements = loadMeasurements(argv[1]);

  // Split the observations into a training and validation group
  std::pair<KinematicMeasurement::Set, KinematicMeasurement::Set> measurement_sets = divide(measurements, 0.8);

  KinematicCalibrationProblemPose3D problem(DHChain({}), createTwoAxisPositioner());
  problem.observations = measurement_sets.first;
  problem.target_mount_to_target_guess = Eigen::Isometry3d::Identity();
  problem.target_mount_to_target_guess.translate(Eigen::Vector3d(0.17, -0.65, 0.5));
  problem.target_mount_to_target_guess.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
  problem.target_mount_to_target_guess.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));
  problem.target_mount_to_target_guess.rotate(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));

  problem.camera_mount_to_camera_guess = Eigen::Isometry3d::Identity();
  problem.camera_mount_to_camera_guess.translate(Eigen::Vector3d(2.2, 0.7, 1.075));

  problem.camera_base_to_target_base_guess = Eigen::Isometry3d::Identity();

  problem.camera_chain_offset_stdev = 0.001;
  problem.target_chain_offset_stdev = 0.005;

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
//  problem.mask.at(4) = { 0, 2 };

  KinematicCalibrationResult result = optimize(problem);

  Eigen::IOFormat fmt(4, 0, "|", "\n", "|", "|");

  std::stringstream ss;
  ss << "\nCalibration " << (result.converged ? "did" : "did not") << " converge\n";
  ss << "Initial cost per observation: " << std::sqrt(result.initial_cost_per_obs) << "\n";
  ss << "Final cost per observation: " << std::sqrt(result.final_cost_per_obs) << "\n";
  ss << "\nCamera mount to camera\n" << result.camera_mount_to_camera.matrix().format(fmt) << "\n";

  ss << result.camera_mount_to_camera.rotation().eulerAngles(0, 1, 2) << "\n";


  ss << "\nTarget mount to target\n" << result.target_mount_to_target.matrix().format(fmt) << "\n";

  ss << result.target_mount_to_target.rotation().eulerAngles(0, 1, 2) << "\n";


  ss << "\nDH parameter offsets\n" << result.target_chain_dh_offsets.matrix().format(fmt) << "\n";
  ss << result.covariance.printCorrelationCoeffAboveThreshold(0.5);

  std::cout << ss.str() << std::endl;

  test(problem.camera_chain, problem.target_chain, result, measurement_sets.second);

  return 0;
}
