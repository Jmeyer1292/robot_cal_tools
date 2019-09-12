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

  static cv::Ptr<CircleDetector> create(const CircleDetector::Params& parameters = CircleDetector::Params());
};
}
#endif // RCT_IMAGE_TOOLS_CIRCLE_DETECTOR_H
