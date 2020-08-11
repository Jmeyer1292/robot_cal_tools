#include <rct_image_tools/charuco_finder.h>

#include <numeric>
#include <gtest/gtest.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using namespace rct_image_tools;

class ChArUcoTest : public ::testing::Test
{
public:
  ChArUcoTest() : ::testing::Test(), target(7, 5, 0.02, 0.01, 10), finder(target)
  {
  }

  void runTest()
  {
    // Load the test image
    cv::Mat m = cv::imread(filename);
    ASSERT_FALSE(m.empty());

    // Find the chessboard corners
    std::map<int, Eigen::Vector2d> features = this->finder.findObservations(m);

    // Should have detected 24 corners
    ASSERT_EQ(features.size(), expected_ids.size());

    // Iterate through the corners to see if they match the ID (should see IDs 0-23)
    for (const unsigned id : expected_ids)
    {
      EXPECT_TRUE(features.find(id) != features.end());
    }
  }

  const CharucoGridTarget target;
  const CharucoGridBoardObservationFinder finder;
  std::string filename;
  std::vector<unsigned> expected_ids;
};

class UnobscuredChArUcoTest : public ChArUcoTest
{
public:
  using ChArUcoTest::ChArUcoTest;

  virtual void SetUp() override final
  {
    filename = std::string(TEST_SUPPORT_DIR) + "charuco_unobscured.jpg";

    // Expect all corners to be detected
    expected_ids.resize(target.board->chessboardCorners.size());
    std::iota(expected_ids.begin(), expected_ids.end(), 0);
  }
};

class ObscuredChArUcoTest : public ChArUcoTest
{
public:
  using ChArUcoTest::ChArUcoTest;

  virtual void SetUp() override final
  {
    filename = std::string(TEST_SUPPORT_DIR) + "charuco_obscured.jpg";

    // Expect the first seven corners not to be detected
    expected_ids.resize(target.board->chessboardCorners.size() - 7);
    std::iota(expected_ids.begin(), expected_ids.end(), 7);
  }
};

class OneFeatureChArUcoTest : public ChArUcoTest
{
public:
  using ChArUcoTest::ChArUcoTest;

  virtual void SetUp() override final
  {
    filename = std::string(TEST_SUPPORT_DIR) + "one_corner.jpg";

    // Expect to only see the first corner (ID = 0)
    expected_ids.resize(1);
    expected_ids.front() = 0;
  }
};

TEST_F(UnobscuredChArUcoTest, TestDetection)
{
  this->runTest();
}

TEST_F(ObscuredChArUcoTest, TestDetection)
{
  this->runTest();
}

TEST_F(OneFeatureChArUcoTest, TestDetection)
{
  this->runTest();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
