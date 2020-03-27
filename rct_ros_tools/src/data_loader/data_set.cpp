#include "rct_ros_tools/data_set.h"
#include "rct_ros_tools/parameter_loaders.h"
#include "rct_image_tools/image_utils.h"

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
  cv::Mat image = cv::imread(path, CV_LOAD_IMAGE_COLOR); // TODO: Is CV_LOAD_IMAGE_COLOR needed?
  if (image.data == NULL)
  {
    ROS_ERROR("File failed to load or does not exist: %s", path.c_str());
  }
  return image;
}

static rct_ros_tools::ExtrinsicDataSet parse(const YAML::Node& root, const std::string& root_path)
{
  rct_ros_tools::ExtrinsicDataSet data;

  for (std::size_t i = 0; i < root.size(); ++i)
  {
    // Each entry should have a pose and image path. This path is relative to the root_path directory!
    const auto img_path = root[i]["image"].as<std::string>();
    const auto pose_path = root[i]["pose"].as<std::string>();
    cv::Mat image = readImageOpenCV(combine(root_path, img_path));
    Eigen::Isometry3d p;
    rct_ros_tools::loadPose(combine(root_path, pose_path), p);

    data.images.push_back(image);
    data.tool_poses.push_back(p);
  }

  return data;
}


boost::optional<rct_ros_tools::ExtrinsicDataSet> rct_ros_tools::parseFromFile(const std::string &path)
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

void writePose(const std::string& path, const Eigen::Isometry3d& pose)
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

void writeDirectory(const std::string& path, const rct_ros_tools::ExtrinsicDataSet& data)
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

bool rct_ros_tools::saveToDirectory(const std::string& path, const rct_ros_tools::ExtrinsicDataSet& data)
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

rct_ros_tools::ExtrinsicCorrespondenceDataSet::ExtrinsicCorrespondenceDataSet(const std::vector<rct_ros_tools::ExtrinsicDataSet> &extrinsic_data_set,
                                                                              const rct_image_tools::ModifiedCircleGridObservationFinder &obs_finder,
                                                                              bool debug)
{
  correspondences_.resize(extrinsic_data_set.size(), extrinsic_data_set[0].images.size());
  mask_.resize(extrinsic_data_set.size(), extrinsic_data_set[0].images.size());
  for (std::size_t c = 0; c < extrinsic_data_set.size(); ++c)
  {
    // We know it exists, so define a helpful alias
    const rct_ros_tools::ExtrinsicDataSet& data_set = extrinsic_data_set[c];

    // Finally, we need to process our images into correspondence sets: for each dot in the
    // target this will be where that dot is in the target and where it was seen in the image.
    // Repeat for each image. We also tell where the wrist was when the image was taken.
    for (std::size_t i = 0; i < data_set.images.size(); ++i)
    {
      mask_(c, i) = 1;
      // Try to find the circle grid in this image:
      rct_optimizations::CorrespondenceSet obs_set = rct_image_tools::getCorrespondenceSet(obs_finder, data_set.images[i]);
      if (obs_set.empty())
      {
        ROS_WARN_STREAM("Unable to find the circle grid in image: " << i);
        mask_(c, i) = 0;
      }

      if (debug)
      {
        // Show the points we detected
        std::vector<Eigen::Vector2d> observations(obs_set.size());
        std::transform(obs_set.begin(), obs_set.end(), observations.begin(),
                       [](const rct_optimizations::Correspondence2D3D& o) { return o.in_image; });

        cv::imshow("points", obs_finder.drawObservations(data_set.images[i], observations));
        cv::waitKey();
      }

      correspondences_(c, i) = obs_set;
    }
  }
}

std::size_t rct_ros_tools::ExtrinsicCorrespondenceDataSet::getCameraCount() const
{
  return correspondences_.rows();
}

std::size_t rct_ros_tools::ExtrinsicCorrespondenceDataSet::getImageCount() const
{
  return correspondences_.cols();
}

std::size_t rct_ros_tools::ExtrinsicCorrespondenceDataSet::getImageCameraCount(std::size_t image_index) const
{
  return mask_.col(image_index).sum();
}

std::size_t rct_ros_tools::ExtrinsicCorrespondenceDataSet::getCameraImageCount(std::size_t camera_index) const
{
  return mask_.row(camera_index).sum();
}

bool rct_ros_tools::ExtrinsicCorrespondenceDataSet::foundCorrespondence(std::size_t camera_index, std::size_t image_index) const
{
  return static_cast<bool>(mask_(camera_index, image_index));
}

const rct_optimizations::CorrespondenceSet&
rct_ros_tools::ExtrinsicCorrespondenceDataSet::getCorrespondenceSet(std::size_t camera_index, std::size_t image_index) const
{
  return correspondences_(camera_index, image_index);
}
