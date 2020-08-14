/*
 IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.

 By downloading, copying, installing or using the software you agree
 to this license.
 If you do not agree to this license, do not download, install,
 copy or use the software.


                          License Agreement
               For Open Source Computer Vision Library

Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
Copyright (C) 2009, Willow Garage Inc., all rights reserved.
Copyright (C) 2014, Southwest Research Institute, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistribution's of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistribution's in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * The name of the copyright holders may not be used to endorse or
    promote products
    derived from this software without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed.
In no event shall the Intel Corporation or contributors be liable for any
direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

// Slight Modification of OpenCV function to use ellipse fitting rather than
// center of mass of contour to provide the location of the circle.

#include "rct_image_tools/circle_detector.h"

#include <algorithm>
#include <iterator>
#include <yaml-cpp/yaml.h>

namespace rct_image_tools
{

CircleDetector::Params::Params()
{
  minThreshold = 50;
  maxThreshold = 220;
  thresholdStep = 10;

  minRepeatability = 2;
  minDistBetweenCircles = 10;
  minRadiusDiff = 10;

  filterByColor = true;
  circleColor = 0;

  filterByArea = true;
  minArea = 25;
  maxArea = 5000;

  filterByCircularity = false;
  minCircularity = 0.8f;
  maxCircularity = std::numeric_limits<float>::max();

  filterByInertia = true;
  minInertiaRatio = 0.1f;
  maxInertiaRatio = std::numeric_limits<float>::max();

  filterByConvexity = true;
  minConvexity = 0.95f;
  maxConvexity = std::numeric_limits<float>::max();
}

class CircleDetectorImpl : public CircleDetector
{
public:
  explicit CircleDetectorImpl(const CircleDetector::Params& parameters = CircleDetector::Params());

protected:
  struct CV_EXPORTS Center
  {
    cv::Point2d location;
    double radius;
    double confidence;
  };

  /**
   * @brief Overload of detect function, used to find keypoints in an image
   * @param image
   * @param keypoints
   * @param mask
   */
  virtual void detect(cv::InputArray image, std::vector<cv::KeyPoint>& keypoints, cv::InputArray mask = cv::noArray()) override;

  /**
   * @brief Helper function for finding circles within an image
   * This function thresholds the input image at the input intensity, applies filtering, and attempts to find circles from contours in the image
   * @param image - grayscale image
   * @param threshold - threshold intensity
   * @return
   */
  virtual std::vector<Center> findCircles(const cv::Mat& image, const double threshold) const;

  Params params;
};

CircleDetectorImpl::CircleDetectorImpl(const CircleDetector::Params& parameters) : params(parameters) {}

std::vector<CircleDetectorImpl::Center> CircleDetectorImpl::findCircles(const cv::Mat& image, const double threshold) const
{
  // Threshold the image
  cv::Mat binarized_image;
  cv::threshold(image, binarized_image, threshold, 255, cv::THRESH_BINARY);

  // Get the contours of the image
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binarized_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  // Loop on all contours
  std::vector<Center> centers;
  double confidence = 1.0;
  for (std::size_t i = 0; i < contours.size(); ++i)
  {
    // Each if statement may eliminate a contour through the continue function
    const std::vector<cv::Point>& contour = contours[i];

    // Calculate the moment of inertia of the contour
    cv::Moments moms = cv::moments(cv::Mat(contour));

    // Area filter
    if (params.filterByArea)
    {
      double area = moms.m00;
      if (area < params.minArea || area >= params.maxArea)
        continue;
    }

    // Circularity filter
    if (params.filterByCircularity)
    {
      double area = moms.m00;
      double perimeter = cv::arcLength(cv::Mat(contour), true);
      double ratio = 4 * CV_PI * area / (perimeter * perimeter);
      if (ratio < params.minCircularity || ratio >= params.maxCircularity)
        continue;
    }

    // Inertia filter
    if (params.filterByInertia)
    {
      double denominator = sqrt(pow(2 * moms.mu11, 2) + pow(moms.mu20 - moms.mu02, 2));
      const double eps = 1e-2;
      double ratio;
      if (denominator > eps)
      {
        double cosmin = (moms.mu20 - moms.mu02) / denominator;
        double sinmin = 2 * moms.mu11 / denominator;
        double cosmax = -cosmin;
        double sinmax = -sinmin;

        double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
        double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
        ratio = imin / imax;
      }
      else
      {
        ratio = 1;
      }

      if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio)
        continue;

      confidence = ratio * ratio;
    }

    // Convexity filter
    if (params.filterByConvexity)
    {
      std::vector<cv::Point> hull;
      cv::convexHull(cv::Mat(contour), hull);
      double area = cv::contourArea(cv::Mat(contour));
      double hullArea = cv::contourArea(cv::Mat(hull));
      double ratio = area / hullArea;
      if (ratio < params.minConvexity || ratio >= params.maxConvexity)
        continue;
    }

    // Fit an ellipse to the contour
    cv::Mat pointsf;
    cv::Mat(contour).convertTo(pointsf, CV_32F);
    if (pointsf.rows < 5)
      continue;
    cv::RotatedRect box = cv::fitEllipse(pointsf);

    // Check that the color of the center pixel matches the expectation
    if (params.filterByColor)
    {
      if (binarized_image.at<uchar>(cvRound(box.center.y), cvRound(box.center.x)) != params.circleColor)
        continue;
    }

    // Create the center
    Center center;
    center.location = box.center;
    center.confidence = confidence;
    center.radius = (box.size.height + box.size.width) / 4.0;
    centers.push_back(center);
  }

  return centers;
}

void CircleDetectorImpl::detect(cv::InputArray input, std::vector<cv::KeyPoint>& keypoints, cv::InputArray)
{
  cv::Mat image = input.getMat();
  keypoints.clear();
  cv::Mat grayscale_image;

  if (image.channels() == 3)
    cv::cvtColor(image, grayscale_image, CV_BGR2GRAY);
  else
    grayscale_image = image;

  // Create a container for all of the circle centers detected at the different threshold levels
  std::vector<std::vector<Center>> centers;

  // Threshold the image per the input parameters and attempt to find all circles
  for (double threshold = params.minThreshold; threshold < params.maxThreshold; threshold += params.thresholdStep)
  {
    // Find all the circles in the image
    std::vector<Center> new_centers = findCircles(grayscale_image, threshold);

    // Lambda for determining if the input circle center already exists in the list of known circle centers
    auto exists = [this](const Center& c, const std::vector<std::vector<Center>>& known_centers) -> int {
      for (std::size_t i = 0; i < known_centers.size(); ++i)
      {
        const std::vector<Center>& existing_centers = known_centers.at(i);
        if (!existing_centers.empty())
        {
          // Compare the identified center with the first existing center
          const Center& existing = existing_centers.front();
          double pos_diff = cv::norm(existing.location - c.location);
          double radius_diff = std::abs(existing.radius - c.radius);

          // If the position and radius differences are within tolerance, add this point to this list of same centers
          // (to be weight-averaged later)
          if (pos_diff >= params.minDistBetweenCircles || radius_diff >= params.minRadiusDiff)
          {
            continue;
          }
          else
          {
            return i;
          }
        }
      }

      return -1;
    };

    // Add the newly identified circle centers to the container of all circle centers
    for (const Center& c : new_centers)
    {
      // Check if this circle center has already been identified
      int match_idx = exists(c, centers);

      // If so, then add this center to the vector of other circle centers at approximately the same location
      if (match_idx >= 0)
      {
        centers.at(match_idx).push_back(c);
      }
      else
      {
        // Otherwise, add it in a new vector to the circle centers list
        centers.push_back({ c });
      }
    }
  }

  // Once all circle centers have been found in the variously thresholded image, create a keypoint based on the weighted average of each center
  for (std::size_t i = 0; i < centers.size(); i++)
  {
    // Check that the circle center meets the minimum repeatability criteria
    if (centers[i].size() >= params.minRepeatability)
    {
      // Get the weighted average of the circle center's location
      cv::Point2d sum_point(0, 0);
      double sum_radius = 0.0;
      double normalizer = 0;
      for (std::size_t j = 0; j < centers[i].size(); j++)
      {
        sum_point += centers[i][j].confidence * centers[i][j].location;
        sum_radius += centers[i][j].confidence * centers[i][j].radius;
        normalizer += centers[i][j].confidence;
      }
      sum_point /= normalizer;
      sum_radius /= normalizer;

      // Create a key point and add it to the output parameter
      keypoints.emplace_back(sum_point, sum_radius * 2.0);
    }
  }
}

cv::Ptr<CircleDetector> CircleDetector::create(const CircleDetector::Params& params)
{
  return cv::makePtr<CircleDetectorImpl>(params);
}

template <typename T>
bool optionalLoad(YAML::Node& n, const std::string& key, T& value)
{
  if (n[key])
  {
    try
    {
      value = n[key].as<T>();
    }
    catch (const YAML::BadConversion& ex)
    {
      std::stringstream ss;
      ss << "(Key: " << key << "): ";
      YAML::RepresentationException ex_key =
          YAML::RepresentationException(YAML::Mark::null_mark(), std::string(ss.str() + ex.what()));
      throw ex_key;
    }

    return true;
  }
  else
  {
    return false;
  }
}

bool CircleDetector::loadParams(const std::string& path, CircleDetector::Params& params)
{
  YAML::Node n = YAML::LoadFile(path);

  CircleDetector::Params p;

  try
  {
    optionalLoad(n, "thresholdStep", p.thresholdStep);
    optionalLoad(n, "minThreshold", p.minThreshold);
    optionalLoad(n, "maxThreshold", p.maxThreshold);
    optionalLoad(n, "minRepeatability", p.minRepeatability);
    optionalLoad(n, "minDistBetweenCircles", p.minDistBetweenCircles);
    optionalLoad(n, "minRadiusDiff", p.minRadiusDiff);

    optionalLoad(n, "filterByColor", p.filterByColor);
    unsigned short circleColor;
    if (optionalLoad(n, "circleColor", circleColor))
      p.circleColor = circleColor;

    optionalLoad(n, "filterByArea", p.filterByArea);
    optionalLoad(n, "minArea", p.minArea);
    optionalLoad(n, "maxArea", p.maxArea);

    optionalLoad(n, "filterByCircularity", p.filterByCircularity);
    optionalLoad(n, "minCircularity", p.minCircularity);
    optionalLoad(n, "maxCircularity", p.maxCircularity);

    optionalLoad(n, "filterByInertia", p.filterByInertia);
    optionalLoad(n, "minInertiaRatio", p.minInertiaRatio);
    optionalLoad(n, "maxInertiaRatio", p.maxInertiaRatio);

    optionalLoad(n, "filterByConvexity", p.filterByConvexity);
    optionalLoad(n, "minConvexity", p.minConvexity);
    optionalLoad(n, "maxConvexity", p.maxConvexity);
  }
  catch (const YAML::Exception& ex)
  {
    throw std::runtime_error(std::string("CircleDetector::loadParams failed to load: ") + ex.what());
  }

  params = p;
  return true;
}

} // namespace cv
