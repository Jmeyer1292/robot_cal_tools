#include "rct_image_tools/image_observation_finder.h"
#include <iostream>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: rosrun rct_image_tools rct_image_tools_test "
                 "<PATH_TO_IMAGE_FILE>\n";
    return 1;
  }

  std::string path(argv[1]);

  cv::Mat m = cv::imread(path);

  rct_image_tools::ModifiedCircleGridTarget target;
  target.rows = 9;
  target.cols = 12;

  rct_image_tools::ModifiedCircleGridObservationFinder obs_finder(target);

  auto maybe_obs = obs_finder.findObservations(m);

  if (maybe_obs)
  {
    std::cout << "Grid observed: Found " << maybe_obs->size() << " observations!\n";
  }
  else
  {
    std::cout << "Failed to find observations\n";
  }

  // TODO Replace the drawing API
  //  cv::imshow("out", out);
  //  cv::moveWindow("out", 400, 400);
  //  cv::waitKey();
  return 0;
}
