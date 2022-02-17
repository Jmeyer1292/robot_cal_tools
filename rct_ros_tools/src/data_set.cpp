#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_image_tools/image_utils.h>
#include <rct_optimizations/serialization/eigen.h>

#include <fstream>
#include <opencv2/highgui.hpp>
#include <ros/console.h>
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

cv::Mat rct_ros_tools::readImageOpenCV(const std::string& path)
{
  cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
  if (image.data == NULL)
  {
    ROS_ERROR("File failed to load or does not exist: %s", path.c_str());
  }
  return image;
}

rct_ros_tools::ExtrinsicDataSet parse(const YAML::Node& root, const std::string& root_path)
{
  rct_ros_tools::ExtrinsicDataSet data;

  for (std::size_t i = 0; i < root.size(); ++i)
  {
    // Each entry should have a pose and image path. This path is relative to the root_path directory!
    const auto img_path = root[i]["image"].as<std::string>();
    const auto pose_path = root[i]["pose"].as<std::string>();
    cv::Mat image = rct_ros_tools::readImageOpenCV(combine(root_path, img_path));

    if (image.empty())
    {
      ROS_WARN_STREAM("Failed to load image " << i << ". Skipping...");
      continue;
    }

    Eigen::Isometry3d p;
    if (!rct_ros_tools::loadPose(combine(root_path, pose_path), p))
    {
      ROS_WARN_STREAM("Failed to load pose " << i << ". Skipping...");
      continue;
    }

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
  YAML::Node root(pose);
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
                                                                              const rct_image_tools::TargetFinder &target_finder,
                                                                              bool debug)
{
  static const std::string WINDOW = "window";
  if (debug)
    cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);

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
      // Try to find the circle grid in this image:
      try
      {
        mask_(c, i) = 1;

        rct_image_tools::TargetFeatures target_features = target_finder.findTargetFeatures(data_set.images[i]);
        if (target_features.empty())
          throw std::runtime_error("Failed to find any target features in image " + std::to_string(i));
        ROS_INFO_STREAM("Found " << target_features.size() << " target features");

        correspondences_(c, i) = target_finder.target().createCorrespondences(target_features);

        if (debug)
        {
          // Show the points we detected
          std::vector<Eigen::Vector2d> observations(correspondences_(c, i).size());
          std::transform(correspondences_(c, i).begin(), correspondences_(c, i).end(), observations.begin(),
                         [](const rct_optimizations::Correspondence2D3D& o) { return o.in_image; });

          cv::imshow(WINDOW, target_finder.drawTargetFeatures(data_set.images[i], target_features));
          cv::waitKey();
        }
      }
      catch (const std::exception& ex)
      {
        mask_(c, i) = 0;
        ROS_ERROR_STREAM(ex.what());
      }
    }
  }

  if (debug)
    cv::destroyWindow(WINDOW);
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

const rct_optimizations::Correspondence2D3D::Set&
rct_ros_tools::ExtrinsicCorrespondenceDataSet::getCorrespondenceSet(std::size_t camera_index, std::size_t image_index) const
{
  return correspondences_(camera_index, image_index);
}
