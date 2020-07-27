#include "rct_image_tools/charuco_finder.h"

int main(int argc, char** argv)
{
	if(argc < 6)
	{
		std::cout << "rosrun rct_image_tools rct_image_charuco_test <PATH_TO_IMAGE_FILE> -w=xx -h=xx -sl=xx -ml=xx -d=xx\n"
		"{w        |       | Number of squares in X direction }\n"
        "{h        |       | Number of squares in Y direction }\n"
        "{sl       |       | Square side length (in meters) }\n"
        "{ml       |       | Marker side length (in meters) }\n"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,\n"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7,\n "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,\n"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}\n";
		
		return 1;
	}
	
	CommandLineParser parser(argc, argv);
	std::string path(argv[1]);
    int squaresX = parser.get<int>("w");
    int squaresY = parser.get<int>("h");
    float squareLength = parser.get<float>("sl");
    float markerLength = parser.get<float>("ml");
    int dictionaryId = parser.get<int>("d");
    
    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
	
    float axisLength = 0.5f * ((float)min(squaresX, squaresY) * (squareLength));

    // Define Target - create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard =
        aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();
    
    //Load Image
    cv::Mat m = cv::imread(path);

	rct_image_tools::CharucoGridBoardObservationFinder charuco_finder(board);
	
	boost::optional<std::vector<Eigen::Vector2d>> maybe_charuco = charuco_finder.findObservations(m);
	
	if(maybe_charuco)
	{
		std::cout << "Grid observed: Found " << maybe_charuco->size() << " charuco target!\n";
		
		cv::Mat dots = charuco_finder.drawObservations(m, *maybe_charuco);
		
		cv::imshow("out", dots);
		cv::moveWindow("out", 400, 400);
		cv:waitKey();
	}
	else
	{
		std::cout << "Failed to find charuco target\n";
	}
	return 0;
}
