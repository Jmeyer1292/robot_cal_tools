#include "rct_examples/data_set.h"
#include <yaml-cpp/yaml.h>
#include <ros/console.h>

#include <opencv2/highgui.hpp>

#include <fstream>
#include <sys/stat.h>

static std::string rootPath(const std::string& filename)
{
  const auto last_idx = filename.find_last_of('/');
  assert(last_idx != std::string::npos);
  return filename.substr(0, last_idx);
}

static std::string combine(const std::string& dir, const std::string& rel_path)
{
  return dir + "/" + rel_path;
}

static cv::Mat readImageOpenCV(const std::string& path)
{
  return cv::imread(path, CV_LOAD_IMAGE_COLOR); // TODO: Is CV_LOAD_IMAGE_COLOR needed?
}

static Eigen::Affine3d readPoseFromYaml(const std::string& path)
{
  YAML::Node n = YAML::LoadFile(path);
  Eigen::Vector3d position;

  position(0) = n["x"].as<double>();
  position(1) = n["y"].as<double>();
  position(2) = n["z"].as<double>();

  double qw, qx, qy, qz;
  qw = n["qw"].as<double>();
  qx = n["qx"].as<double>();
  qy = n["qy"].as<double>();
  qz = n["qz"].as<double>();

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() = position;
  pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
  return pose;
}

static rct_examples::ExtrinsicDataSet parse(const YAML::Node& root, const std::string& root_path)
{
  rct_examples::ExtrinsicDataSet data;

  for (std::size_t i = 0; i < root.size(); ++i)
  {
    // Each entry should have a pose and image path. This path is relative to the root_path directory!
    const auto img_path = root[i]["image"].as<std::string>();
    const auto pose_path = root[i]["pose"].as<std::string>();
    cv::Mat image = readImageOpenCV(combine(root_path, img_path));
    Eigen::Affine3d p = readPoseFromYaml(combine(root_path, pose_path));

    data.images.push_back(image);
    data.tool_poses.push_back(p);
  }

  return data;
}


boost::optional<rct_examples::ExtrinsicDataSet> rct_examples::parseFromFile(const std::string &path)
{
  try
  {
    YAML::Node root = YAML::LoadFile(path);
    const std::string root_path = rootPath(path);
    return parse(root, root_path);
  }
  catch (const YAML::Exception& ex)
  {
    ROS_ERROR_STREAM("Error while parsing YAML file: " << ex.what());
    return {};
  }
}

void writePose(const std::string& path, const Eigen::Affine3d& pose)
{
  YAML::Node root;
  root["x"] = pose.translation().x();
  root["y"] = pose.translation().y();
  root["z"] = pose.translation().z();

  Eigen::Quaterniond q (pose.linear());
  root["qx"] = q.x();
  root["qy"] = q.y();
  root["qz"] = q.z();
  root["qw"] = q.w();

  std::ofstream ofh (path);
  ofh << root;
}

void writeDirectory(const std::string& path, const rct_examples::ExtrinsicDataSet& data)
{
  YAML::Node root;

  for (std::size_t i = 0; i < data.images.size(); ++i)
  {
    YAML::Node n;
    n["pose"] = "poses/" + std::to_string(i) + ".yaml";
    n["image"] = "images/" + std::to_string(i) + ".png";

    root.push_back(n);
  }

  std::ofstream ofh (path);
  ofh << root;
}

bool rct_examples::saveToDirectory(const std::string& path, const rct_examples::ExtrinsicDataSet& data)
{
  mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir((path + "/images").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir((path + "/poses").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  for (std::size_t i = 0; i < data.images.size(); ++i)
  {
    auto name = path + "/images/" + std::to_string(i) + ".png";
    cv::imwrite(name, data.images[i]);
  }

  for (std::size_t i = 0; i < data.tool_poses.size(); ++i)
  {
    auto name = path + "/poses/" + std::to_string(i) + ".yaml";
    writePose(name, data.tool_poses[i]);
  }

  writeDirectory(path + "/data.yaml", data);

  return true;
}
