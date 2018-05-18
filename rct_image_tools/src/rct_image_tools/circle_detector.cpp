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
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall the Intel Corporation or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

// Slight Modification of OpenCV function to use ellipse fitting rather than
// center of mass of contour to provide the location of the circle.

#include <rct_image_tools/circle_detector.h>

#include <algorithm>
#include <iterator>

namespace rct_image_tools
{

class CircleDetectorImpl : public CircleDetector
{
public:
  using CircleDetector::detect; // Note(gChiou): Fixes woverloaded-virtual warnings

  explicit CircleDetectorImpl(const CircleDetector::Params &parameters
    = CircleDetector::Params());

//  virtual void read(const FileNode &fn);
//  virtual void write(FileStorage &fs) const;

protected:
  struct CV_EXPORTS Center
  {
    cv::Point2d location;
    double radius;
    double confidence;
  };

  virtual void detect(cv::InputArray image, std::vector<cv::KeyPoint> &keypoints,
    cv::InputArray mask = cv::noArray());

  virtual void findCircles(cv::InputArray image, cv::InputArray binaryImage, std::vector<Center> &centers) const;

  Params params;
};

CircleDetector::Params::Params()
{
  thresholdStep = 10;
  minThreshold = 50;
  maxThreshold = 220;
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

CircleDetectorImpl::CircleDetectorImpl(const CircleDetector::Params &parameters) :
  params(parameters) { }


void CircleDetectorImpl::findCircles(cv::InputArray _image, cv::InputArray _binaryImage,
  std::vector<Center> &centers) const
{
  cv::Mat image       = _image.getMat(); // Oh so much  cleaner this way :(
  cv::Mat binaryImage = _binaryImage.getMat();

  (void)image;
  centers.clear();

  std::vector<std::vector<cv::Point>> contours;
  cv::Mat tmpBinaryImage = binaryImage.clone();
  cv::findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  // Loop on all contours
  for (std::size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
  {
    // Each if statement may eliminate a contour through the continue function
    // Some if statements may also set the confidence whose default is 1.0
    Center center;
    center.confidence = 1;
    cv::Moments moms = cv::moments(cv::Mat(contours[contourIdx]));
    if (params.filterByArea)
    {
      double area = moms.m00;
      if (area < params.minArea || area >= params.maxArea)
        continue;
    }

    if (params.filterByCircularity)
    {
      double area = moms.m00;
      double perimeter = cv::arcLength(cv::Mat(contours[contourIdx]), true);
      double ratio = 4 * CV_PI * area / (perimeter * perimeter);
      if (ratio < params.minCircularity || ratio >= params.maxCircularity)
        continue;
    }

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

      center.confidence = ratio * ratio;
    }

    if (params.filterByConvexity)
    {
      std::vector<cv::Point> hull;
      cv::convexHull(cv::Mat(contours[contourIdx]), hull);
      double area = cv::contourArea(cv::Mat(contours[contourIdx]));
      double hullArea = cv::contourArea(cv::Mat(hull));
      double ratio = area / hullArea;
      if (ratio < params.minConvexity || ratio >= params.maxConvexity)
        continue;
    }
    cv::Mat pointsf;
    cv::Mat(contours[contourIdx]).convertTo(pointsf, CV_32F);
    if(pointsf.rows<5) continue;
    cv::RotatedRect box = cv::fitEllipse(pointsf);

    // Find center
    center.location = box.center;

    // One more filter by color of central pixel
    if (params.filterByColor)
    {
      if (binaryImage.at<uchar>(cvRound(center.location.y), cvRound(center.location.x)) != params.circleColor)
        continue;
    }

    center.radius = (box.size.height+box.size.width) / 4.0;
    centers.push_back(center);
  }
}

void CircleDetectorImpl::detect(cv::InputArray _image, std::vector<cv::KeyPoint> &keypoints,
  cv::InputArray mask)
{
  cv::Mat image = _image.getMat();
  keypoints.clear();
  cv::Mat grayscaleImage;

  if (image.channels() == 3)
    cv::cvtColor(image, grayscaleImage, CV_BGR2GRAY);
  else
    grayscaleImage = image;

  std::vector<std::vector<Center>> centers;
  for (double thresh = params.minThreshold; thresh < params.maxThreshold;
    thresh += params.thresholdStep)
  {
    cv::Mat binarizedImage;
    cv::threshold(grayscaleImage, binarizedImage, thresh, 255, cv::THRESH_BINARY);
    std::vector<Center> curCenters;
    findCircles(grayscaleImage, binarizedImage, curCenters);
    std::vector<std::vector<Center>> newCenters;
    for (std::size_t i = 0; i < curCenters.size(); i++)
    {
      bool isNew = true;
      for (std::size_t j = 0; j < centers.size(); j++)
      {
        double dist = norm(centers[j][ centers[j].size() / 2 ].location
          - curCenters[i].location);
        double rad_diff = fabs(centers[j][ centers[j].size() / 2 ].radius
          - curCenters[i].radius);

        isNew = dist >= params.minDistBetweenCircles || rad_diff >= params.minRadiusDiff;

        if (!isNew)
        {
          centers[j].push_back(curCenters[i]);

          std::size_t k = centers[j].size() - 1;
          while( k > 0 && centers[j][k].radius < centers[j][k-1].radius )
          {
            centers[j][k] = centers[j][k-1];
            k--;
          }
          centers[j][k] = curCenters[i];
          break;
        }
      }

      if (isNew)
      {
        newCenters.push_back(std::vector<Center> (1, curCenters[i]));
      }
    }
  std::copy(newCenters.begin(), newCenters.end(), std::back_inserter(centers));
  }

  for (std::size_t i = 0; i < centers.size(); i++)
  {
    if (centers[i].size() < params.minRepeatability)
      continue;
    cv::Point2d sumPoint(0, 0);
    double normalizer = 0;
    for (std::size_t j = 0; j < centers[i].size(); j++)
    {
      sumPoint += centers[i][j].confidence * centers[i][j].location;
      normalizer += centers[i][j].confidence;
    }
    sumPoint *= (1. / normalizer);
    cv::KeyPoint kpt(sumPoint, static_cast<float>(centers[i][centers[i].size() / 2].radius*2.0));
    keypoints.push_back(kpt);
  }
}

cv::Ptr<CircleDetector> CircleDetector::create(const CircleDetector::Params &params)
{
  return cv::makePtr<CircleDetectorImpl>(params);
}

} // namespace cv
