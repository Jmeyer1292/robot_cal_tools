#include "rct_image_tools/modified_circle_grid_finder.h"
#include <iostream>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv)
{
  if (argc != 4)
  {
    std::cout << "Usage: rosrun rct_image_tools rct_image_tools_test "
                 "<PATH_TO_IMAGE_FILE> <NUM_ROWS> <NUM_COLS>\n";
    return 1;
  }

  // Parse parameters
  std::string path(argv[1]);
  int rows = std::stoi(argv[2]);
  int cols = std::stoi(argv[3]);

  // Load Image
  cv::Mat m = cv::imread(path);

  // Define target - I hard-code the spacing parameter because it doesn't matter for this example
  rct_image_tools::ModifiedCircleGridTarget target(rows, cols, 0.01);

  // Create a finder that works with this target
  rct_image_tools::ModifiedCircleGridTargetFinder target_finder(target);

  // Attempt to find the grid: The optional will be set if it succeeded.
  try
  {
    rct_image_tools::TargetFeatures obs = target_finder.findTargetFeatures(m);
    std::cout << "Grid observed: Found " << obs.size() << " features!\n";

    // For debug purposes, let's draw the dots
    cv::Mat dots = target_finder.drawTargetFeatures(m, obs);
    // And display the resulting image
    cv::imshow("out", dots);
    cv::moveWindow("out", 400, 400);
    cv::waitKey();
  }
  catch (const std::exception& ex)
  {
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
