#include "rct_image_tools/image_observation_finder.h"
#include <opencv2/highgui.hpp>

int main(int argc, char** argv)
{
  std::string path (argv[1]);

  cv::Mat m = cv::imread(path);

  rct_image_tools::TargetDefinition target;
  rct_image_tools::ImageObservationFinder obs_finder (target);

  cv::Mat out;
  obs_finder.findObservations(m, out);

  cv::imshow("out", out);
  cv::waitKey();
  return 0;
}
