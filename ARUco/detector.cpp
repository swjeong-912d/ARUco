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
void drawAxis(Mat& inputImage, InputArray _cameraMatrix, InputArray _distCoeffs, InputArray _rvec, InputArray _tvec, float length)
{
	drawFrameAxes(inputImage, _cameraMatrix, _distCoeffs, _rvec, _tvec, length, 3);
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



	Mat camMatrix, distCoeffs;
	FileStorage fs("output.yml", FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;


	vector<cv::Point3f> markerCorners3d;
	markerCorners3d.push_back(cv::Point3f(-0.5f, 0.5f, 0));
	markerCorners3d.push_back(cv::Point3f(0.5f, 0.5f, 0));
	markerCorners3d.push_back(cv::Point3f(0.5f, -0.5f, 0));
	markerCorners3d.push_back(cv::Point3f(-0.5f, -0.5f, 0));


	Mat InputImage;
	detector.getInputImage(InputImage);
	for (const auto& markers : finalDetectedMarkers)
	{
		if (markers.markerId > 100) continue;
		//카메라와 마커사이의 rotation 및 translation 벡터를 구함
		Mat rotation_vector, translation_vector;
		solvePnP(markerCorners3d, markers.markerContours, camMatrix, distCoeffs, rotation_vector, translation_vector);

		cout << "markerID " << markers.markerId << endl;
		cout << "rotation_vector" << endl << rotation_vector << endl;
		cout << "translation_vector" << endl << translation_vector << endl;

		//aruco 모듈에서 제공하는 함수를 이용하여 마커위에 좌표축을 그림
		drawAxis(InputImage,camMatrix, distCoeffs, rotation_vector, translation_vector, 1.0);
	}
	
	imshow("Axis Image", InputImage);
	waitKey(WAIT_TIME);

}
