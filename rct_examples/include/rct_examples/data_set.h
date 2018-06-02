#ifndef RCT_DATA_SET_H
#define RCT_DATA_SET_H

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace rct_examples
{

struct ExtrinsicDataSet
{
  std::vector<cv::Mat> images;
  std::vector<Eigen::Affine3d> tool_poses;
};

boost::optional<ExtrinsicDataSet> parseFromFile(const std::string& path);

}

#endif
