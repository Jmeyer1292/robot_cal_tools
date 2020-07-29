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
#include <opencv2/opencv.hpp>

using namespace cv;

TEST(CharucoChain, TestUnobscured)
{
    int squaresX = 5;
    int squaresY = 7;
    float squareLength = 200;
    float markerLength = 120;
    int dictionaryId = 10;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // Define Target - create charuco board object
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    cv::Ptr<cv::aruco::Board> board = charucoboard.staticCast<cv::aruco::Board>();

    std::string path = "/home/jberkebile/robot_tools/src/robot_cal_tools/rct_image_tools/test/images/charuco_unobstructed.jpg";
    //Load Image
    cv::Mat m = imread(path);
    EXPECT_FALSE(m.empty());

    rct_image_tools::CharucoGridBoardObservationFinder charuco_finder(charucoboard);

    std::map<int, std::vector<Eigen::Vector2f>> maybe_charuco = charuco_finder.findObservations(m);

    EXPECT_FALSE(maybe_charuco.empty());
    EXPECT_EQ(maybe_charuco.size(), 24);

    auto id_iterator = maybe_charuco.cbegin();
    for(int i = 0; i < maybe_charuco.size(); i++)
    {
      int key_id = id_iterator->first;
      EXPECT_EQ(i, key_id);
      std::advance(id_iterator, 1);
    }
    return;
}

TEST(CharucoChain, TestObscured)
{
    int squaresX = 5;
    int squaresY = 7;
    float squareLength = 200;
    float markerLength = 120;
    int dictionaryId = 10;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // Define Target - create charuco board object
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    cv::Ptr<cv::aruco::Board> board = charucoboard.staticCast<cv::aruco::Board>();

    std::string path = "/home/jberkebile/robot_tools/src/robot_cal_tools/rct_image_tools/test/images/charuco_obstructed.jpg";
    //Load Image
    cv::Mat m = imread(path);
    EXPECT_FALSE(m.empty());

    rct_image_tools::CharucoGridBoardObservationFinder charuco_finder(charucoboard);

    std::map<int, std::vector<Eigen::Vector2f>> maybe_charuco = charuco_finder.findObservations(m);

    EXPECT_FALSE(maybe_charuco.empty());
    EXPECT_EQ(maybe_charuco.size(), 17);

    auto id_iterator = maybe_charuco.cbegin();
    for(int i = 0; i < maybe_charuco.size(); i++)
    {
      int key_id = id_iterator->first;
      if(i < 7)
      {
        EXPECT_NE(i, key_id);
      }
      else
      {
        EXPECT_EQ(i, key_id);
        std::advance(id_iterator, 1);
      }
    }
    return;
}

TEST(CharucoChain, TestOneCorner)
{
    int squaresX = 5;
    int squaresY = 7;
    float squareLength = 200;
    float markerLength = 120;
    int dictionaryId = 10;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // Define Target - create charuco board object
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    cv::Ptr<cv::aruco::Board> board = charucoboard.staticCast<cv::aruco::Board>();

    std::string path = "/home/jberkebile/robot_tools/src/robot_cal_tools/rct_image_tools/test/images/one_corner.jpg";
    //Load Image
    cv::Mat m = imread(path);
    EXPECT_FALSE(m.empty());

    rct_image_tools::CharucoGridBoardObservationFinder charuco_finder(charucoboard);

    std::map<int, std::vector<Eigen::Vector2f>> maybe_charuco = charuco_finder.findObservations(m);

    EXPECT_FALSE(maybe_charuco.empty());
    EXPECT_EQ(maybe_charuco.size(), 1);

    auto id_iterator = maybe_charuco.cbegin();
    for(int i = 0; i < maybe_charuco.size(); i++)
    {
      int key_id = id_iterator->first;
      if(i < 1)
      {
        EXPECT_EQ(i, key_id);
      }
      else
      {
        EXPECT_NE(i, key_id);
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
