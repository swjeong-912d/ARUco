#include <opencv2/opencv.hpp>
#include "opencv2/core/hal/hal.hpp"
#include "dictionary.hpp"
#include "MarkerDetector.hpp"
#include <vector>
using namespace cv;


// wait time constant.
const int WAIT_TIME = 0;

namespace {
	const char* about = "Detect ArUco markers from image";
	const char* keys =
		"{@outfile |<none> | Output image }"
		"{name     |       | Input filename }"
		"{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
		"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
		"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
		"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
		"{id       |       | Marker id in the dictionary }"
		"{bb       | 1     | Number of bits in marker borders }"
		"{si       | false | show generated image }";
}


int main(int argc, char* argv[]) {
	CommandLineParser parser(argc, argv, keys);
	parser.about(about);

	if (argc < 4) {
		parser.printMessage();
		return 0;
	}
	MarkerDetector detector;

	String filename = parser.get<String>("name");
	int dictionaryId = parser.get<int>("d");
	int markerId   = parser.get<int>("id");
	int borderBits = parser.get<int>("bb");
	bool showImage = parser.get<bool>("si");

	String out = parser.get<String>(0);

	if (!parser.check()) {
		parser.printErrors();
		return 0;
	}



	param params;
	params.showImage = showImage;
	params.borderBits = borderBits;
	params.dictionaryId = dictionaryId;
	detector.setParameters(params);
	vector<MarkerInfo> finalDetectedMarkers;
	detector.detectMarkers(filename, finalDetectedMarkers, params);

}
