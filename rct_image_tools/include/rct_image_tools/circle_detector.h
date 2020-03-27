#ifndef RCT_IMAGE_TOOLS_CIRCLE_DETECTOR_H
#define RCT_IMAGE_TOOLS_CIRCLE_DETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace rct_image_tools
{

class CircleDetector : public cv::FeatureDetector
{
public:
  struct Params
  {
    Params();

    float thresholdStep;
    float minThreshold;
    float maxThreshold;
    size_t minRepeatability;
    float minDistBetweenCircles;
    float minRadiusDiff;

    bool filterByColor;
    uchar circleColor;

    bool filterByArea;
    float minArea, maxArea;

    bool filterByCircularity;
    float minCircularity, maxCircularity;

    bool filterByInertia;
    float minInertiaRatio, maxInertiaRatio;

    bool filterByConvexity;
    float minConvexity, maxConvexity;
  };

  static cv::Ptr<CircleDetector> create(const CircleDetector::Params& params = CircleDetector::Params());

  /**
   * @brief loadParams Load CircleDetector parameters from a yaml file.  Only specified values will be modified.
   * @param path The file to load
   * @param params Params struct that will be modified
   * @return True if the file existed and and no exceptions generated
   * @throw std::runtime_error if a parameter in the file exists and fails to be parsed.  Putting a string instead of a
   * float, etc.
   */
  static bool loadParams(const std::string& path, CircleDetector::Params& params);
};
}  // namespace rct_image_tools
#endif // RCT_IMAGE_TOOLS_CIRCLE_DETECTOR_H
