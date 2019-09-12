#include "rct_image_tools/image_observation_finder.h"
#include "rct_image_tools/circle_detector.h"
#include <opencv2/calib3d.hpp>

using ObservationPoints = std::vector<cv::Point2d>;

static void drawPointLabel(const std::string& label, const cv::Point2d& position, const CvScalar& color, cv::Mat& image)
{
  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
  const double font_scale = 0.75;
  const int line_thickness = 2;

  CvSize label_size;
  int label_baseline;
  cvGetTextSize(label.c_str(), &font, &label_size, &label_baseline);

  cv::Point label_origin = cv::Point(position.x, position.y);

  cv::putText(image, label, label_origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, line_thickness);

  // Draw dot
  const int RADIUS = 5.0;
  cv::circle(image, position, RADIUS, color, -1);
}

static cv::Mat renderObservations(const cv::Mat& input, const ObservationPoints& observation_points,
                                  const rct_image_tools::ModifiedCircleGridTarget& target)
{
  cv::Mat output_image;
  input.copyTo(output_image);

  if (!observation_points.empty())
  {
    cv::Size pattern_size(target.cols, target.rows);

    cv::Mat center_image = cv::Mat(observation_points);
    cv::Mat center_converted;
    center_image.convertTo(center_converted, CV_32F);
    cv::drawChessboardCorners(output_image, pattern_size, center_converted, true);

    // Draw point labels // TODO
    drawPointLabel("First Point", observation_points[0], CvScalar(0, 255, 0), output_image);
    drawPointLabel("Origin", observation_points[target.rows * target.cols - target.cols], CvScalar(255, 0, 0),
                   output_image);
    drawPointLabel("Last Point", observation_points[observation_points.size() - 1], CvScalar(0, 0, 255), output_image);
  }

  return output_image;
}

template <typename DETECTOR_PTR>
static bool extractKeyPoints(const ObservationPoints& centers, ObservationPoints& observation_points,
                             cv::Ptr<DETECTOR_PTR>& detector_ptr, std::size_t rows, std::size_t cols, bool flipped,
                             const cv::Mat& image)
{
  /*
    Note(cLewis):
    This is the same method called in the beginning of findCirclesGrid,
    unfortunately, they don't
    return their keypoints. If OpenCV changes, the keypoint locations may not
    match, which has the
    risk of failing with updates to OpenCV.
  */

  // Extract KeyPoints
  std::vector<cv::KeyPoint> keypoints;
  cv::Point large_point;

  std::size_t start_first_row = 0;
  std::size_t end_first_row = cols - 1;
  std::size_t start_last_row = rows * cols - cols;
  std::size_t end_last_row = rows * cols - 1;

  for (double alpha = 1.0; alpha <= 3.0; alpha += 0.01)
  {
    bool found = false;
    for (int beta = 0; beta <= 100; beta++)
    {
      cv::Mat altered_image;
      image.convertTo(altered_image, -1, alpha, beta);
      detector_ptr->detect(altered_image, keypoints);
      if (keypoints.size() >= rows * cols)
      {
        found = true;
        break;
      }
    }
    if (found)
    {
      break;
    }
  }

  // If a flipped pattern is found, flip the rows/columns
  std::size_t temp_rows = flipped ? cols : rows;
  std::size_t temp_cols = flipped ? rows : cols;

  // Determine which circle is the largest
  double start_first_row_size = -1.0;
  double start_last_row_size = -1.0;
  double end_first_row_size = -1.0;
  double end_last_row_size = -1.0;

  cv::Point2d start_last_row_pt = centers[start_last_row];
  cv::Point2d end_last_row_pt = centers[end_last_row];
  cv::Point2d start_first_row_pt = centers[start_first_row];
  cv::Point2d end_first_row_pt = centers[end_first_row];

  for (std::size_t i = 0; i < keypoints.size(); i++)
  {
    double x = keypoints[i].pt.x;
    double y = keypoints[i].pt.y;
    double ksize = keypoints[i].size;

    if (x == start_last_row_pt.x && y == start_last_row_pt.y)
    {
      start_last_row_size = ksize;
    }
    if (x == end_last_row_pt.x && y == end_last_row_pt.y)
    {
      end_last_row_size = ksize;
    }
    if (x == start_first_row_pt.x && y == start_first_row_pt.y)
    {
      start_first_row_size = ksize;
    }
    if (x == end_first_row_pt.x && y == end_first_row_pt.y)
    {
      end_first_row_size = ksize;
    }
  }

  // No keypoint match for one or more corners
  if (start_last_row_size < 0.0 || start_first_row_size < 0.0 || end_last_row_size < 0.0 || end_first_row_size < 0.0)
  {
    return false;
  }

  /*
    Note(cLewis):
    Determine if ordering is usual by computing cross product of two vectors.
    Normal ordering has
    z-axis positive in cross. The most common ordering is with points going from
    left to right then
    top to bottom.
  */
  bool usual_ordering = true;
  double v1x, v1y, v2x, v2y;

  v1x = end_last_row_pt.x - start_last_row_pt.x;
  v1y = -end_last_row_pt.y + start_last_row_pt.y;
  v2x = end_first_row_pt.x - end_last_row_pt.x;
  v2y = -end_first_row_pt.y + end_last_row_pt.y;

  double cross = v1x * v2y - v1y * v2x;
  if (cross < 0.0)
  {
    usual_ordering = false;
  }

  /*
    Note(cLewis): Largest circle at start of last row
    ......
    ......
    ......
    o.....
  */
  if (start_last_row_size > start_first_row_size && start_last_row_size > end_first_row_size &&
      start_last_row_size > end_last_row_size)
  {
    large_point.x = start_last_row_pt.x;
    large_point.y = start_last_row_pt.y;
    if (usual_ordering)
    {
      for (std::size_t j = 0; j < centers.size(); j++)
      {
        observation_points.push_back(centers[j]);
      }
    }
    else // unusual ordering
    {
      for (int j = temp_cols - 1; j >= 0; j--)
      {
        for (int k = temp_rows - 1; k >= 0; k--)
        {
          observation_points.push_back(centers[static_cast<std::size_t>(k) * temp_cols + static_cast<std::size_t>(j)]);
        }
      }
    }
  }

  /*
    Note(cLewis): Largest circle at end of first row
    .....o
    ......
    ......
    ......
  */
  else if (end_first_row_size > end_last_row_size && end_first_row_size > start_last_row_size &&
           end_first_row_size > start_first_row_size)
  {
    large_point.x = end_first_row_pt.x;
    large_point.y = end_first_row_pt.y;
    if (usual_ordering)
    {
      for (int j = (centers.size() - 1); j >= 0; j--)
      {
        observation_points.push_back(centers[static_cast<std::size_t>(j)]);
      }
    }
    else // unusual ordering
    {
      for (std::size_t j = 0; j < temp_cols; j++)
      {
        for (std::size_t k = 0; k < temp_rows; k++)
        {
          observation_points.push_back(centers[k * temp_cols + j]);
        }
      }
    }
  }

  /*
    Note(cLewis): Largest circle at end of last row
    ......
    ......
    ......
    .....o
  */
  else if (end_last_row_size > start_last_row_size && end_last_row_size > end_first_row_size &&
           end_last_row_size > start_first_row_size)
  {
    large_point.x = end_last_row_pt.x;
    large_point.y = end_last_row_pt.y;

    if (usual_ordering)
    {
      for (std::size_t j = 0; j < temp_cols; j++)
      {
        for (int k = temp_rows - 1; k >= 0; k--)
        {
          observation_points.push_back(centers[static_cast<std::size_t>(k) * temp_cols + j]);
        }
      }
    }
    else // unusual ordering
    {
      for (std::size_t j = 0; j < temp_cols; j++)
      {
        for (std::size_t k = 0; k < temp_rows; k++)
        {
          observation_points.push_back(centers[k * temp_cols + j]);
        }
      }
    }
  }

  /*
    Note(cLewis): Largest circle at start of first row
    o.....
    ......
    ......
    ......
  */
  else if (start_first_row_size > end_last_row_size && start_first_row_size > end_first_row_size &&
           start_first_row_size > start_last_row_size)
  {
    large_point.x = start_first_row_pt.x;
    large_point.y = start_first_row_pt.y;
    if (usual_ordering)
    {
      for (int j = temp_cols - 1; j >= 0; j--)
      {
        for (std::size_t k = 0; k < temp_rows; k++)
        {
          observation_points.push_back(centers[k * temp_cols + static_cast<std::size_t>(j)]);
        }
      }
    }
    else // unusual ordering
    {
      for (int j = temp_cols - 1; j >= 0; j--)
      {
        for (int k = temp_rows - 1; k >= 0; k--)
        {
          observation_points.push_back(centers[static_cast<std::size_t>(k) * temp_cols + static_cast<std::size_t>(j)]);
        }
      }
    }
  }

  else
  {
    return false;
  }

  return true;
}

template <typename PARAMS, typename DETECTOR_PTR, typename DETECTOR>
static bool extractModifiedCircleGrid(const cv::Mat& image, const rct_image_tools::ModifiedCircleGridTarget& target,
                                      ObservationPoints& observation_points)
{
  PARAMS detector_params;

  cv::Ptr<DETECTOR_PTR> detector_ptr = DETECTOR::create(detector_params);

  bool flipped = false;

  std::size_t cols = target.cols;
  std::size_t rows = target.rows;

  cv::Size pattern_size(cols, rows);
  cv::Size pattern_size_flipped(rows, cols);

  ObservationPoints centers;

  bool regular_pattern_found = cv::findCirclesGrid(image, pattern_size, centers,
                                                   cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, detector_ptr);
  if (regular_pattern_found && (centers.size() == rows * cols))
  {
    // Do nothing, there used to be something in here, may have
    // to re-structure this in the future.
  }
  else // Try flipped pattern size
  {

    bool flipped_pattern_found = cv::findCirclesGrid(
        image, pattern_size_flipped, centers, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, detector_ptr);
    if (flipped_pattern_found && (centers.size() == rows * cols))
    {
      flipped = true;
    }
  }

  if (centers.size() == 0)
  {
    return false;
  }

  observation_points.reserve(centers.size());

  return extractKeyPoints(centers, observation_points, detector_ptr, rows, cols, flipped, image);
}

rct_image_tools::ModifiedCircleGridObservationFinder::ModifiedCircleGridObservationFinder(const ModifiedCircleGridTarget& target)
    : target_(target)
{
  assert(target.cols != 0);
  assert(target.rows != 0);
}

boost::optional<std::vector<Eigen::Vector2d>>
rct_image_tools::ModifiedCircleGridObservationFinder::findObservations(const cv::Mat& image) const
{
  // Call modified circle finder
  ObservationPoints points;

  if (!extractModifiedCircleGrid<CircleDetector::Params, CircleDetector, CircleDetector>(image, target_, points))
  {
    // If we fail to detect the grid, then return an empty optional
    return {};
  }

  // Otherwise, package the observation results
  std::vector<Eigen::Vector2d> observations(points.size());
  std::transform(points.begin(), points.end(), observations.begin(),
                 [](const cv::Point2d& p) { return Eigen::Vector2d(p.x, p.y); });

  return boost::optional<std::vector<Eigen::Vector2d>>(observations);
}

cv::Mat
rct_image_tools::ModifiedCircleGridObservationFinder::drawObservations(const cv::Mat& image,
                                                          const std::vector<Eigen::Vector2d>& observations) const
{
  ObservationPoints cv_obs(observations.size());
  std::transform(observations.begin(), observations.end(), cv_obs.begin(),
                 [](const Eigen::Vector2d& o) { return cv::Point2d(o.x(), o.y()); });
  return renderObservations(image, cv_obs, target_);
}
