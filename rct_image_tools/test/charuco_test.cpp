#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <rct_image_tools/charuco_finder.h>
#include <rct_optimizations/types.h>
#include <map>
#include <Eigen/Core>
#include <iostream>
#include <gtest/gtest.h>
#include <rct_image_tools/charuco_grid_target.h>

using namespace cv;
using namespace rct_image_tools;

// Runs charuco_finder for a completely unobscured ChAruco target. Should detect 24 chessboard corners
TEST(CharucoChain, TestUnobscured)
{
    int squaresX = 5;
    int squaresY = 7;
    double squareLength = 0.0200;
    double markerLength = 0.0120;

    // Creating the CharucoGridTarget with the above parameters
    CharucoGridTarget target(squaresX, squaresY, squareLength, markerLength);
    const std::string path = std::string(TEST_SUPPORT_DIR) + "charuco_unobscured.jpg";
    //Load Image
    cv::Mat m = imread(path);
    EXPECT_FALSE(m.empty());

    rct_image_tools::CharucoGridBoardObservationFinder charuco_finder(target);
    // Find the chessboard corners (if any)
    std::map<int, Eigen::Vector2d> maybe_charuco = charuco_finder.findObservations(m);

    EXPECT_FALSE(maybe_charuco.empty());
    // Should have detected 24 corners
    EXPECT_EQ(maybe_charuco.size(), 24);
    // Iterate through the corners to see if they match the ID (should see IDs 0-23)
    auto id_iterator = maybe_charuco.begin();
    for(int i = 0; i < maybe_charuco.size(); i++)
    {
      EXPECT_EQ(i, id_iterator->first);
      std::advance(id_iterator, 1);
    }
    return;
}

// Runs charuco_finder for a partially obscured (blocked IDs 0-6) ChAruco target. Should detect 17 chessboard corners
TEST(CharucoChain, TestObscured)
{
    int squaresX = 5;
    int squaresY = 7;
    double squareLength = 0.0200;
    double markerLength = 0.0120;

    // Creating the CharucoGridTarget with the above parameters
    CharucoGridTarget target(squaresX, squaresY, squareLength, markerLength);

    const std::string path = std::string(TEST_SUPPORT_DIR) + "charuco_obscured.jpg";
    //Load Image
    cv::Mat m = imread(path);
    EXPECT_FALSE(m.empty());

    rct_image_tools::CharucoGridBoardObservationFinder charuco_finder(target);
    // Find the chessboard corners (if any)
    std::map<int,Eigen::Vector2d> maybe_charuco = charuco_finder.findObservations(m);

    EXPECT_FALSE(maybe_charuco.empty());
    // Should have detected 17 corners
    EXPECT_EQ(maybe_charuco.size(), 17);
    // Iterate through the corners to see if they match the ID (should see IDs 7-23)
    auto id_iterator = maybe_charuco.begin();
    for(int i = 0; i < maybe_charuco.size(); i++)
    {
      // These IDs were obscured so they shouldn't have been detected
      if(i < 7)
      {
        EXPECT_NE(i, id_iterator->first);
      }
      // The rest of the IDs should've been detected
      else
      {
        EXPECT_EQ(i, id_iterator->first);
        std::advance(id_iterator, 1);
      }
    }
    return;
}

// Runs charuco_finder for an almost entirely obscured (blocked IDs 1-23) ChAruco target. Should detect only 1 chessboard corner
TEST(CharucoChain, TestOneCorner)
{
    int squaresX = 5;
    int squaresY = 7;
    double squareLength = 0.0200;
    double markerLength = 0.0120;

    // Creating the CharucoGridTarget with the above parameters
    CharucoGridTarget target(squaresX, squaresY, squareLength, markerLength);

    const std::string path = std::string(TEST_SUPPORT_DIR) + "one_corner.jpg";
    //Load Image
    cv::Mat m = imread(path);
    EXPECT_FALSE(m.empty());

    rct_image_tools::CharucoGridBoardObservationFinder charuco_finder(target);
    // Find the chessboard corners (if any)
    std::map<int, Eigen::Vector2d> maybe_charuco = charuco_finder.findObservations(m);

    EXPECT_FALSE(maybe_charuco.empty());
    // Should have detected only one corner
    EXPECT_EQ(maybe_charuco.size(), 1);
    // Iterate through the corners to see if they match the ID (should see only ID 0)
    auto id_iterator = maybe_charuco.begin();
    for(int i = 0; i < maybe_charuco.size(); i++)
    {
      // The only corner that wasn't obscured should be detected
      if(i < 1)
      {
        EXPECT_EQ(i, id_iterator->first);
      }
      // The rest were obscured and thus shouldn't be detected
      else
      {
        EXPECT_NE(i, id_iterator->first);
      }
      std::advance(id_iterator, 1);
    }
    return;
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
