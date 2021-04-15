#include <opencv2/opencv.hpp>
#include "opencv2/core/hal/hal.hpp"
#include "dictionary.hpp"
#include "MarkerDetector.hpp"
#include <vector>
#include <sstream>
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
		"{param    |       | marker detection parameter filename }"
		"{cam      |       | camera parameter filename }"
		"{si       | false | show generated image }"
		"{verb     | false | print pipeline completion message }";
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
	String camParams = parser.get<String>("cam");
	int dictionaryId = parser.get<int>("d");
	bool showImage = parser.get<bool>("si");

	if (!parser.check()) {
		parser.printErrors();
		return 0;
	}


	param params;
	params.showImage = showImage;
	params.dictionaryId = dictionaryId;
	detector.setParameters(params);
	vector<MarkerInfo> finalDetectedMarkers;
	detector.detectMarkers(filename, finalDetectedMarkers, params);



	Mat camMatrix, distCoeffs;
	FileStorage fs(camParams, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;


	vector<cv::Point3f> markerCorners3d;
	markerCorners3d.push_back(cv::Point3f(-0.5f, 0.5f, 0));
	markerCorners3d.push_back(cv::Point3f(0.5f, 0.5f, 0));
	markerCorners3d.push_back(cv::Point3f(0.5f, -0.5f, 0));
	markerCorners3d.push_back(cv::Point3f(-0.5f, -0.5f, 0));


	Mat OutputImage;
	detector.getInputImage(OutputImage);
	for (const auto& marker : finalDetectedMarkers)
	{
		//Compute translation and rotation vectors
		Mat rotation_vector, translation_vector;
		solvePnP(markerCorners3d, marker.markerCorners, camMatrix, distCoeffs, rotation_vector, translation_vector);

		cout << "markerID " << marker.markerId << endl;
		cout << "rotation_vector" << endl << rotation_vector << endl;
		cout << "translation_vector" << endl << translation_vector << endl;

		drawFrameAxes(OutputImage, camMatrix, distCoeffs, rotation_vector, translation_vector, 0.8, 4);


		// Find Id display position using projection matrix
		vector<Point3f> idPos3d;
		vector<Point2f> idPos;
		idPos3d.push_back(Point3f(0, 0, 0));
		projectPoints(idPos3d, rotation_vector, translation_vector, camMatrix, distCoeffs, idPos);

		std::stringstream s;
		s << "Id=" << marker.markerId;
		putText(OutputImage, s.str(), idPos[0], FONT_HERSHEY_SIMPLEX, 0.6,
			cv::Scalar(100,200, 0), 2);

	}
	
	imshow("Axis Image", OutputImage);
	waitKey(WAIT_TIME);
	imwrite("detection_result.png", OutputImage);

}
