#include <rct_image_tools/modified_circle_grid_finder.h>
#include <rct_image_tools/circle_detector.h>
#include <opencv2/calib3d.hpp>
#include <memory>

template <typename T>
bool isEqual(const T& a, const T& b, const T& tol = std::numeric_limits<T>::epsilon())
{
  return abs(a - b) < tol;
}

static void
drawPointLabel(const std::string& label, const cv::Point2d& position, const cv::Scalar& color, cv::Mat& image)
{
  const double font_scale = 0.75;
  const int line_thickness = 2;

  cv::Point label_origin = cv::Point(static_cast<int>(position.x), static_cast<int>(position.y));

  cv::putText(image, label, label_origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, line_thickness);

  // Draw dot
  const int radius = 5.0;
  cv::circle(image, position, radius, color, -1);
}

static cv::Mat renderObservations(const cv::Mat& input,
                                  const std::vector<cv::Point2d>& observation_points,
                                  const rct_image_tools::ModifiedCircleGridTarget& target)
{
  cv::Mat output_image;
  input.copyTo(output_image);

  if (!observation_points.empty())
  {
    cv::Size pattern_size(static_cast<int>(target.cols), static_cast<int>(target.rows));

    cv::Mat center_image = cv::Mat(observation_points);
    cv::Mat center_converted;
    center_image.convertTo(center_converted, CV_32F);
    cv::drawChessboardCorners(output_image, pattern_size, center_converted, true);

    // Draw point labels // TODO
    drawPointLabel("First Point", observation_points[0], cv::Scalar(0, 255, 0), output_image);
    drawPointLabel(
        "Origin", observation_points[target.rows * target.cols - target.cols], cv::Scalar(255, 0, 0), output_image);
    drawPointLabel(
        "Last Point", observation_points[observation_points.size() - 1], cv::Scalar(0, 0, 255), output_image);
  }

  return output_image;
}

template <typename DETECTOR_PTR>
static std::vector<cv::Point2d> extractKeyPoints(const cv::Mat& image,
                                                 const std::vector<cv::Point2d>& feature_coordinates,
                                                 cv::Ptr<DETECTOR_PTR>& detector_ptr,
                                                 const std::size_t rows,
                                                 const std::size_t cols,
                                                 const bool flipped)
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

  // OpenCV creates duplicate feature_coordinates sometimes.  https://github.com/opencv/opencv/issues/4775
  // Make sure every center has a matching keypoint.  It may be sufficient just to check feature_coordinates for
  // duplicates.
  std::vector<cv::Point2d> feature_coordinates_tmp = feature_coordinates;
  for (auto&& keypoint : keypoints)
  {
    auto it = std::find_if(feature_coordinates_tmp.begin(), feature_coordinates_tmp.end(), [&](const cv::Point2f& pt) {
      return (isEqual(pt.x, keypoint.pt.x) && isEqual(pt.y, keypoint.pt.y));
    });

    if (it != feature_coordinates_tmp.end())
    {
      feature_coordinates_tmp.erase(it);
    }
  }

  if (!feature_coordinates_tmp.empty())
    throw std::runtime_error("feature_coordinates and keypoints did not match. Check this issue: "
                             "'https://github.com/opencv/opencv/issues/4775'");

  // If a flipped pattern is found, flip the rows/columns
  std::size_t temp_rows = flipped ? cols : rows;
  std::size_t temp_cols = flipped ? rows : cols;

  std::size_t start_first_row = 0;
  std::size_t end_first_row = cols - 1;
  std::size_t start_last_row = rows * cols - cols;
  std::size_t end_last_row = rows * cols - 1;

  // Determine which circle is the largest
  double start_first_row_size = -1.0;
  double start_last_row_size = -1.0;
  double end_first_row_size = -1.0;
  double end_last_row_size = -1.0;

  double start_first_row_avg_rel_size = 0;
  double start_last_row_avg_rel_size = 0;
  double end_first_row_avg_rel_size = 0;
  double end_last_row_avg_rel_size = 0;

  cv::Point2d start_last_row_pt = feature_coordinates[start_last_row];
  cv::Point2d end_last_row_pt = feature_coordinates[end_last_row];
  cv::Point2d start_first_row_pt = feature_coordinates[start_first_row];
  cv::Point2d end_first_row_pt = feature_coordinates[end_first_row];

  std::vector<double> feature_sizes(feature_coordinates.size(), 1.0);

  for (std::size_t i = 0; i < keypoints.size(); i++)
  {
    double x = static_cast<double>(keypoints[i].pt.x);
    double y = static_cast<double>(keypoints[i].pt.y);
    double ksize = static_cast<double>(keypoints[i].size);

    for (std::size_t j = 0; j < feature_coordinates.size(); j++)
    {
      auto center = feature_coordinates[j];
      if (center.x == x && center.y == y)
      {
        feature_sizes[j] = ksize;
        break;
      }
    }
  }

  for (std::size_t i = 0; i < feature_coordinates.size(); i++)
  {
    double x = feature_coordinates[i].x;
    double y = feature_coordinates[i].y;
    double ksize = feature_sizes[i];

    if (x == start_last_row_pt.x && y == start_last_row_pt.y)
    {
      start_last_row_size = ksize;
      if (i + 1 < feature_sizes.size() && i - cols < feature_sizes.size())
      {
        double start_last_row_rel_row_size = start_last_row_size / feature_sizes[i + 1];
        double start_last_row_rel_col_size = start_last_row_size / feature_sizes[i - cols];
        start_last_row_avg_rel_size = (start_last_row_rel_row_size + start_last_row_rel_col_size) / 2.0;
      }
    }
    if (x == end_last_row_pt.x && y == end_last_row_pt.y)
    {
      end_last_row_size = ksize;
      if (i - 1 < feature_sizes.size() && i - cols < feature_sizes.size())
      {
        double end_last_row_rel_row_size = end_last_row_size / feature_sizes[i - 1];
        double end_last_row_rel_col_size = end_last_row_size / feature_sizes[i - cols];
        end_last_row_avg_rel_size = (end_last_row_rel_row_size + end_last_row_rel_col_size) / 2.0;
      }
    }
    if (x == start_first_row_pt.x && y == start_first_row_pt.y)
    {
      start_first_row_size = ksize;
      if (i + 1 < feature_sizes.size() && i + cols < feature_sizes.size())
      {
        double start_first_row_rel_row_size = start_first_row_size / feature_sizes[i + 1];
        double start_first_row_rel_col_size = start_first_row_size / feature_sizes[i + cols];
        start_first_row_avg_rel_size = (start_first_row_rel_row_size + start_first_row_rel_col_size) / 2.0;
      }
    }
    if (x == end_first_row_pt.x && y == end_first_row_pt.y)
    {
      end_first_row_size = ksize;
      if (i - 1 < feature_sizes.size() && i + cols < feature_sizes.size())
      {
        double end_first_row_rel_row_size = end_first_row_size / feature_sizes[i - 1];
        double end_first_row_rel_col_size = end_first_row_size / feature_sizes[i + cols];
        end_first_row_avg_rel_size = (end_first_row_rel_row_size + end_first_row_rel_col_size) / 2.0;
      }
    }
  }

  // Comparing the start last row corner to the second point in the last row
  double start_last_row_rel_row_size = feature_sizes[start_last_row] / feature_sizes[start_last_row + 1];
  // Comparing the start last row corner to the start of the second to last row
  double start_last_row_rel_col_size = feature_sizes[start_last_row] / feature_sizes[start_last_row - cols];
  // Find the average relative size of the start last row corner to its two adjacent circles
  start_last_row_avg_rel_size = (start_last_row_rel_row_size + start_last_row_rel_col_size) / 2.0;

  // Comparing the end last row corner to the second to last point in the last row
  double end_last_row_rel_row_size = feature_sizes[end_last_row] / feature_sizes[end_last_row - 1];
  // Comparing the end last row corner to the end of the second to last row
  double end_last_row_rel_col_size = feature_sizes[end_last_row] / feature_sizes[end_last_row - cols];
  // Find the average relative size of the end last row corner to its two adjacent circles
  end_last_row_avg_rel_size = (end_last_row_rel_row_size + end_last_row_rel_col_size) / 2.0;

  // Comparing the start fisrt row corner to the second point in the first row
  double start_first_row_rel_row_size = feature_sizes[start_first_row] / feature_sizes[start_first_row + 1];
  // Comparing the start first row corner to the start of the second  row
  double start_first_row_rel_col_size = feature_sizes[start_first_row] / feature_sizes[start_first_row + cols];
  // Find the average relative size of the start first row corner to its two adjacent circles
  start_first_row_avg_rel_size = (start_first_row_rel_row_size + start_first_row_rel_col_size) / 2.0;

  // Comparing the end first row corner to the second to last point in the first row
  double end_first_row_rel_row_size = feature_sizes[end_first_row] / feature_sizes[end_first_row - 1];
  // Comparing the end first row corner to the end of the second row
  double end_first_row_rel_col_size = feature_sizes[end_first_row] / feature_sizes[end_first_row + cols];
  // Find the average relative size of the end first row corner to its two adjacent circles
  end_first_row_avg_rel_size = (end_first_row_rel_row_size + end_first_row_rel_col_size) / 2.0;

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

  // Construct the output object
  std::vector<cv::Point2d> observation_points;
  observation_points.reserve(feature_coordinates.size());

  /*
    Note(cLewis): Largest circle at start of last row
    ......
    ......
    ......
    o.....
  */
  if (start_last_row_avg_rel_size > start_first_row_avg_rel_size &&
      start_last_row_avg_rel_size > end_first_row_avg_rel_size &&
      start_last_row_avg_rel_size > end_last_row_avg_rel_size)
  {
    large_point.x = start_last_row_pt.x;
    large_point.y = start_last_row_pt.y;
    if (usual_ordering)
    {
      for (std::size_t j = 0; j < feature_coordinates.size(); j++)
      {
        observation_points.push_back(feature_coordinates[j]);
      }
    }
    else  // unusual ordering
    {
      for (int j = static_cast<int>(temp_cols) - 1; j >= 0; j--)
      {
        for (int k = static_cast<int>(temp_rows) - 1; k >= 0; k--)
        {
          observation_points.push_back(
              feature_coordinates[static_cast<std::size_t>(k) * temp_cols + static_cast<std::size_t>(j)]);
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
  else if (end_first_row_avg_rel_size > end_last_row_avg_rel_size &&
           end_first_row_avg_rel_size > start_last_row_avg_rel_size &&
           end_first_row_avg_rel_size > start_first_row_avg_rel_size)
  {
    large_point.x = end_first_row_pt.x;
    large_point.y = end_first_row_pt.y;
    if (usual_ordering)
    {
      for (int j = (static_cast<int>(feature_coordinates.size()) - 1); j >= 0; j--)
      {
        observation_points.push_back(feature_coordinates[static_cast<std::size_t>(j)]);
      }
    }
    else  // unusual ordering
    {
      for (std::size_t j = 0; j < temp_cols; j++)
      {
        for (std::size_t k = 0; k < temp_rows; k++)
        {
          observation_points.push_back(feature_coordinates[k * temp_cols + j]);
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
  else if (end_last_row_avg_rel_size > start_last_row_avg_rel_size &&
           end_last_row_avg_rel_size > end_first_row_avg_rel_size &&
           end_last_row_avg_rel_size > start_first_row_avg_rel_size)
  {
    large_point.x = end_last_row_pt.x;
    large_point.y = end_last_row_pt.y;

    if (usual_ordering)
    {
      for (std::size_t j = 0; j < temp_cols; j++)
      {
        for (int k = static_cast<int>(temp_rows) - 1; k >= 0; k--)
        {
          observation_points.push_back(feature_coordinates[static_cast<std::size_t>(k) * temp_cols + j]);
        }
      }
    }
    else  // unusual ordering
    {
      for (std::size_t j = 0; j < temp_cols; j++)
      {
        for (std::size_t k = 0; k < temp_rows; k++)
        {
          observation_points.push_back(feature_coordinates[k * temp_cols + j]);
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
  else if (start_first_row_avg_rel_size > end_last_row_avg_rel_size &&
           start_first_row_avg_rel_size > end_first_row_avg_rel_size &&
           start_first_row_avg_rel_size > start_last_row_avg_rel_size)
  {
    large_point.x = start_first_row_pt.x;
    large_point.y = start_first_row_pt.y;
    if (usual_ordering)
    {
      for (int j = static_cast<int>(temp_cols) - 1; j >= 0; j--)
      {
        for (std::size_t k = 0; k < temp_rows; k++)
        {
          observation_points.push_back(feature_coordinates[k * temp_cols + static_cast<std::size_t>(j)]);
        }
      }
    }
    else  // unusual ordering
    {
      for (int j = static_cast<int>(temp_cols) - 1; j >= 0; j--)
      {
        for (int k = static_cast<int>(temp_rows) - 1; k >= 0; k--)
        {
          observation_points.push_back(
              feature_coordinates[static_cast<std::size_t>(k) * temp_cols + static_cast<std::size_t>(j)]);
        }
      }
    }
  }

  else
  {
    throw std::runtime_error("No matching configuration");
  }

  return observation_points;
}

template <typename PARAMS, typename DETECTOR_PTR, typename DETECTOR>
static std::vector<cv::Point2d> extractModifiedCircleGrid(const cv::Mat& image,
                                                          const rct_image_tools::ModifiedCircleGridTarget& target,
                                                          const PARAMS& params)
{
  cv::Ptr<DETECTOR_PTR> detector_ptr;
  detector_ptr = DETECTOR::create(params);

  bool flipped = false;

  std::size_t cols = target.cols;
  std::size_t rows = target.rows;

  cv::Size pattern_size(cols, rows);
  cv::Size pattern_size_flipped(rows, cols);

  std::vector<cv::Point2d> centers;

  bool regular_pattern_found = cv::findCirclesGrid(
      image, pattern_size, centers, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, detector_ptr);
  if (regular_pattern_found && (centers.size() == rows * cols))
  {
    // Do nothing, there used to be something in here, may have
    // to re-structure this in the future.
  }
  else  // Try flipped pattern size
  {
    bool flipped_pattern_found = cv::findCirclesGrid(
        image, pattern_size_flipped, centers, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, detector_ptr);
    if (flipped_pattern_found && (centers.size() == rows * cols))
    {
      flipped = true;
    }
  }

  if (centers.size() == 0)
    throw std::runtime_error("Failed to find circle centers");

  return extractKeyPoints(image, centers, detector_ptr, rows, cols, flipped);
}

namespace rct_image_tools
{
ModifiedCircleGridTargetFinder::ModifiedCircleGridTargetFinder(const ModifiedCircleGridTarget& target,
                                                               const CircleDetectorParams& params)
  : TargetFinder(), target_(target), params_(params)
{
  assert(target_.cols > 0);
  assert(target_.rows > 0);
}

ModifiedCircleGridTargetFinder::ModifiedCircleGridTargetFinder(const ModifiedCircleGridTarget& target)
  : ModifiedCircleGridTargetFinder(target, CircleDetectorParams())
{
}

TargetFeatures ModifiedCircleGridTargetFinder::findTargetFeatures(const cv::Mat& image) const
{
  // Call modified circle finder
  std::vector<cv::Point2d> points =
      extractModifiedCircleGrid<CircleDetectorParams, CircleDetector, CircleDetector>(image, target_, params_);

  // Construct the output map
  TargetFeatures observations;
  for (unsigned i = 0; i < points.size(); ++i)
  {
    const cv::Point2d& pt = points.at(i);
    VectorEigenVector<2> v_obs;
    v_obs.push_back(Eigen::Vector2d(pt.x, pt.y));
    observations.emplace(i, v_obs);
  }

  return observations;
}

cv::Mat ModifiedCircleGridTargetFinder::drawTargetFeatures(const cv::Mat& image,
                                                           const TargetFeatures& target_features) const
{
  // Draw all detected circles
  rct_image_tools::CircleDetector circle_detector_(params_);
  cv::Mat out_image = circle_detector_.drawDetectedCircles(image);

  // Draw the detected circle centers
  std::vector<cv::Point2d> cv_obs;
  cv_obs.reserve(target_features.size());
  for (auto it = target_features.begin(); it != target_features.end(); ++it)
  {
    const Eigen::Vector2d& pt = it->second.at(0);
    cv_obs.push_back(cv::Point2d(pt.x(), pt.y()));
  }

  return renderObservations(out_image, cv_obs, target_);
}

}  // namespace rct_image_tools
