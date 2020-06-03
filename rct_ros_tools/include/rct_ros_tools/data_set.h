#ifndef RCT_DATA_SET_H
#define RCT_DATA_SET_H

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <rct_optimizations/types.h>
#include <rct_image_tools/image_observation_finder.h>

namespace rct_ros_tools
{

struct ExtrinsicDataSet
{
  std::vector<cv::Mat> images;
  std::vector<Eigen::Isometry3d> tool_poses;
};

boost::optional<ExtrinsicDataSet> parseFromFile(const std::string& path);

cv::Mat readImageOpenCV(const std::string& path);

bool saveToDirectory(const std::string& path, const ExtrinsicDataSet& data);

/**
 * @brief This class is used to generate correspondence sets for one/multiple static cameras
 * and a single moveing target.
 */
class ExtrinsicCorrespondenceDataSet
{
public:
  ExtrinsicCorrespondenceDataSet(const std::vector<rct_ros_tools::ExtrinsicDataSet> &extrinsic_data_set,
                                 const rct_image_tools::ModifiedCircleGridObservationFinder &obs_finder,
                                 bool debug = false);

  /** @brief Get the number of cameras */
  std::size_t getCameraCount() const;

  /** @brief Get the number of images taken by each camera */
  std::size_t getImageCount() const;

  /** @brief Get numbers of cameras that found a image */
  std::size_t getImageCameraCount(std::size_t image_index) const;

  /** @brief Get number of images found by a camera */
  std::size_t getCameraImageCount(std::size_t camera_index) const;

  /** @brief Get if a correspondence set was found camera image pair */
  bool foundCorrespondence(std::size_t camera_index, std::size_t image_index) const;

  /** @brief Get the correspondence set for a given camera and image index */
  const rct_optimizations::Correspondence2D3D::Set& getCorrespondenceSet(std::size_t camera_index, std::size_t image_index) const;

private:
  /** @brief Correspondence pairs for a given image and camera */
  Eigen::Matrix<rct_optimizations::Correspondence2D3D::Set, Eigen::Dynamic, Eigen::Dynamic> correspondences_;

  /** @brief Mask matrix indicating if the target was found */
  Eigen::Matrix<unsigned, Eigen::Dynamic, Eigen::Dynamic> mask_;

};

}

#endif
