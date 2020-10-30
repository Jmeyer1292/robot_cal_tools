#pragma once

#include <rct_image_tools/target_features.h>
#include <opencv2/core/mat.hpp>

namespace rct_image_tools
{
/**
 * @brief Base class for target finders
 */
template<typename TargetT>
class TargetFinder
{
public:
  TargetFinder(const TargetT& target)
    : target_(target)
  {
  }

  virtual ~TargetFinder() = default;

  /**
   * @brief Finds the features of the target in an image
   * @param image
   * @return
   */
  virtual TargetFeatures findTargetFeatures(const cv::Mat& image) const = 0;

  /**
   * @brief Draws the target features on an input image
   * @param image
   * @param target_features
   * @return
   */
  virtual cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures& target_features) const = 0;

  const TargetT& target() const
  {
    return target_;
  }

protected:
  TargetT target_;
};

} // namespace rct_image_tools
