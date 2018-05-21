#include <rct_image_tools/image_observation_finder.h>
#include <rct_optimizations/extrinsic_camera_on_wrist.h>
#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/highgui.hpp> // for imread

// YAML HELPERS FOR LOADING STUFF
typedef std::vector<double> JointStates;
typedef std::vector<double> Translation;
typedef std::vector<double> Quaternion;
typedef std::vector<double> RotationRad;
typedef std::vector<double> RotationDeg;

struct LinkData
{
  Translation translation;
  Quaternion rotation_quat;
  JointStates joint_states;
  RotationRad rotation_rad;
  RotationDeg rotation_deg;
};

bool parseYAML(const YAML::Node& node, const std::string& var_name, std::vector<double>& var_value)
{

  var_value.clear();
  if (node[var_name])
  {
    const YAML::Node n = node[var_name];
    var_value.reserve(n.size());
    for (std::size_t i = 0; i < n.size(); i++)
    {
      double value = n[i].as<double>();
      var_value.push_back(value);
    }
    if (var_value.size() == n.size())
    {
      return true;
    }
  }
  return false;
}

bool loadLinkData(const std::string& file_path, LinkData* link_data, const std::string& node)
{
  bool success = true;

  YAML::Node data_yaml;
  try
  {
    data_yaml = YAML::LoadFile(file_path);
    // if (!data_yaml["base_link_to_tool0"]) {return false;}
    if (!data_yaml[node])
    {
      return false;
    }
  }
  catch (YAML::BadFile& bf)
  {
    return false;
  }

  success &= parseYAML(data_yaml[node], "Translation", link_data->translation);
  success &= parseYAML(data_yaml[node], "Quaternion", link_data->rotation_quat);
  return success;
}

bool loadLinkData(const std::string& file_path, rct_optimizations::Pose6d& pose, const std::string& node)
{
  LinkData link_data;
  bool success = loadLinkData(file_path, &link_data, node);

  double tx, ty, tz, qx, qy, qz, qw;
  tx = link_data.translation[0];
  ty = link_data.translation[1];
  tz = link_data.translation[2];
  qx = link_data.rotation_quat[0];
  qy = link_data.rotation_quat[1];
  qz = link_data.rotation_quat[2];
  qw = link_data.rotation_quat[3];

  double angle = 2.0 * acos(qw);
  double ax = qx / sqrt(1 - qw * qw) * angle;
  double ay = qy / sqrt(1 - qw * qw) * angle;
  double az = qz / sqrt(1 - qw * qw) * angle;

  pose = rct_optimizations::Pose6d({ax, ay, az, tx, ty, tz});

  return success;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_on_wrist_extrinsic");
  ros::NodeHandle pnh("~");

  // Load Image Set
  std::string data_path;
  if (!pnh.getParam("data_path", data_path))
  {
    ROS_ERROR("Must set 'data_path' parameter");
    return 1;
  }

  const std::size_t num_images = 15;
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = data_path + "mcircles_10x10/extrinsic/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i) + ".png";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    calibration_images.push_back(image);
  }

  auto print_pose6d = [](const rct_optimizations::Pose6d& p) {
    ROS_INFO("%f %f %f - %f %f %f", p.x(), p.y(), p.z(), p.rx(), p.ry(), p.rz());
  };

  // Load Wrist Poses
  std::vector<rct_optimizations::Pose6d> link_data;
  link_data.resize(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    loadLinkData(data_path + "mcircles_10x10/extrinsic/tf/" + std::to_string(i) + ".yaml", link_data[i],
                 "base_link_to_tool0");
  }

  // Process each image into observations
  // Load Target Definition
  rct_image_tools::ModifiedCircleGridTarget target(10, 10, 0.0254);

  rct_image_tools::ImageObservationFinder obs_finder(target);

  // Construct problem
  rct_optimizations::CameraIntrinsics intr;
  intr.fx() = 510.0;
  intr.fy() = 510.0;
  intr.cx() = 320.2;
  intr.cy() = 208.9;

  rct_optimizations::ExtrinsicCameraOnWristProblem problem_def;
  problem_def.intr = intr;
  problem_def.wrist_to_camera_guess =
      rct_optimizations::poseCalToEigen(rct_optimizations::Pose6d({0, 0, -M_PI_2, 0.019, 0, 0.15}));

  problem_def.base_to_target_guess =
      rct_optimizations::poseCalToEigen(rct_optimizations::Pose6d({0, 0, -M_PI_2, 0.75, 0, 0}));

  for (std::size_t i = 0; i < calibration_images.size(); ++i)
  {
    // Extract observations
    auto maybe_obs = obs_finder.findObservations(calibration_images[i]);
    if (!maybe_obs)
    {
      continue;
    }

    // Show drawing
    cv::imshow("points", obs_finder.drawObservations(calibration_images[i], *maybe_obs));
    cv::waitKey();

    // We got observations, let's process
    const rct_optimizations::Pose6d& wrist_pose = link_data[i];
    problem_def.wrist_poses.push_back(rct_optimizations::poseCalToEigen(wrist_pose));

    rct_optimizations::ObservationSet obs_set;

    assert(maybe_obs->size() == target.points.size());
    for (std::size_t j = 0; j < maybe_obs->size(); ++j)
    {
      rct_optimizations::ObservationPair pair;
      pair.in_image = maybe_obs->at(j);
      pair.in_target = target.points[j];

      obs_set.push_back(pair);
    }

    problem_def.image_observations.push_back(obs_set);
  }

  // Run optimization
  auto opt_result = rct_optimizations::optimize(problem_def);

  // Report results
  std::cout << "Did converge?: " << opt_result.converged << "\n";
  std::cout << "Initial cost?: " << opt_result.initial_cost_per_obs << "\n";
  std::cout << "Final cost?: " << opt_result.final_cost_per_obs << "\n";

  auto c = opt_result.wrist_to_camera;
  auto t = opt_result.base_to_target;

  std::cout << c.matrix() << "\n";
  std::cout << t.matrix() << "\n";
  print_pose6d(rct_optimizations::poseEigenToCal(c));
  print_pose6d(rct_optimizations::poseEigenToCal(t));

  return 0;
}
