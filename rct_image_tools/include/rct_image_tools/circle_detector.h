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

    /** @brief The minimum grayscale pixel intensity value at which to start the image thresholding */
    float minThreshold;
    /** @brief The maximum grayscale pixel intensity value at which to stop the image thresholding */
    float maxThreshold;
    /** @brief The thresholding step */
    float thresholdStep;

    /** @brief The number of times a particular circle must be identified to be considered valid (must be <= the number of threshold steps) */
    size_t minRepeatability;
    /** @brief The radius (pixels) around an identified circle within which new detected blobs will be considered to be
     * the same feature as the circle */
    float circleInclusionRadius;
    /** @brief The maximum difference in radius (pixels) between an identified circle and a detected blob, above which
     * the blob will not be considered to be the same feature as the previously identified circle */
    float maxRadiusDiff;

    /** @brief The maximum average deviation of the contour of a blob from its calculated ellipse fit (percentage)
     *  Testing has shown that ellipses can be reliably detected with ~0.2% error (i.e. 0.002) */
    float maxAverageEllipseError;

    /** @brief Flag for color filtering */
    bool filterByColor;
    /** @brief Color intensity of circle center in the binary image (value must be 0 or 255) */
    uchar circleColor;

    /** @brief Flag for filtering by area */
    bool filterByArea;
    /** @brief Minimum blob area (px^2) */
    float minArea;
    /** @brief Maximum blob area (px^2) */
    float maxArea;

    /** @brief Flag for circularity filtering */
    bool filterByCircularity;
    /** @brief Minimum blob circularity ratio - for a perfect circle this value is 1.0 / PI (~0.333) */
    float minCircularity;
    /**@ brief Maximum blob circularity ratio */
    float maxCircularity;

    /** @brief Flag for inertia filtering */
    bool filterByInertia;
    /** @brief Minimum blob inertia ratio */
    float minInertiaRatio;
    /** @brief Maximum blob inertia ratio - for a perfect circle, this value is 1.0 */
    float maxInertiaRatio;

    /** @brief Flag for convexity filtering */
    bool filterByConvexity;
    /** @brief Minimum blob convexity */
    float minConvexity;
    /** @brief Maximum blob convexity */
    float maxConvexity;
  };

  /**
   * @brief Creates a circle detector pointer from a parameter structure
   * @param params
   * @return
   */
  static cv::Ptr<CircleDetector> create(const CircleDetector::Params& params = CircleDetector::Params(), const bool debug = false);

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
