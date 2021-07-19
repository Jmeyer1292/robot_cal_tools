#pragma once

#include <ros/node_handle.h>
#include <string>
#include <rct_image_tools/target.h>
#include <rct_image_tools/target_finder.h>

namespace rct_ros_tools
{
class TargetFinderPlugin : public rct_image_tools::TargetFinder
{
public:
  using TargetFinder::TargetFinder;

  rct_image_tools::TargetFeatures findTargetFeatures(const cv::Mat& image) const override
  {
    return finder_->findTargetFeatures(image);
  }

  cv::Mat drawTargetFeatures(const cv::Mat& image,
                             const rct_image_tools::TargetFeatures& target_features) const override
  {
    return finder_->drawTargetFeatures(image, target_features);
  }

  const rct_image_tools::Target& target() const override
  {
    return finder_->target();
  }

  virtual void init(const XmlRpc::XmlRpcValue& config) = 0;
  virtual void init(const std::string& file) = 0;

protected:
  std::shared_ptr<const rct_image_tools::TargetFinder> finder_;
};

class ModifiedCircleGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  using TargetFinderPlugin::TargetFinderPlugin;

  void init(const XmlRpc::XmlRpcValue& config) override;
  void init(const std::string& file) override;
};

class CharucoGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  using TargetFinderPlugin::TargetFinderPlugin;

  void init(const XmlRpc::XmlRpcValue& config) override;
  void init(const std::string& file) override;
};

} // namespace rct_ros_tools
