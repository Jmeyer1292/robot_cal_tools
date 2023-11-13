#pragma once

#include <rct_image_tools/target_finder.h>
#include <rct_image_tools/modified_circle_grid_target.h>
#include <rct_image_tools/circle_detector.h>

namespace rct_image_tools
{
/**
 * @brief This class finds 2D features (circle centers) from images of a known ModifiedCircleGridTarget.
 * All points must be seen or it will fail. Features are returned in the same order as points are defined in the target.
 */
class ModifiedCircleGridTargetFinder : public TargetFinder
{
public:
  ModifiedCircleGridTargetFinder(const ModifiedCircleGridTarget& target);
  ModifiedCircleGridTargetFinder(const ModifiedCircleGridTarget& target, const CircleDetectorParams& params);

  /**
   * @brief Finds target features in an input image
   * @param image
   * @return
   */
  virtual TargetFeatures findTargetFeatures(const cv::Mat& image) const override;

  /**
   * @brief A debugging utility that will draw target features onto an image for display purposes.
   * Usually you want to call findTargetFeatures() above then this with the result.
   */
  virtual cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures& target_features) const override;

  virtual const Target& target() const override { return target_; }

  inline const CircleDetectorParams& getCircleDetectorParams() const { return params_; }

protected:
  const ModifiedCircleGridTarget target_;
  const CircleDetectorParams params_;
};

}  // namespace rct_image_tools
